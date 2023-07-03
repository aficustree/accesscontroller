#!/usr/bin/python3

# Documentation
# https://developer.apple.com/homekit/specification/
# 9.30 (current door state), 9.118 (target door state), 9.65 (Obstruction Detected), 9.62 (Name)
# https://developers.homebridge.io/#/service/GarageDoorOpener

# automation hat documentation: https://github.com/pimoroni/automation-hat
# GPIO Input Documentation: https://sourceforge.net/p/raspberry-gpio-python/wiki/Inputs/

# Physical Wiring
# needs pull-up resistor 
# when gate is open, input pin is connected HIGH to VCC
# when gate is closed, input pin is pulled LOW to GND 
# not using internal pull-up resistor as it's unbuffered and very voltage intollerant leading to lightning damage
# also, not consistently strong enough to pull up floating circuit
# as automationHat has internal voltage dividers

   
import automationhat
import RPi.GPIO as GPIO
from enum import IntEnum, Enum
import argparse
import logging
import time
from urllib.parse import urlparse
import asyncio
from bottle import route, run, abort
from threading import Lock
import paho.mqtt.publish as mqttpublish
import paho.mqtt.subscribe as mqttsubscribe

log = logging.getLogger(__name__)

class AccessControl:

    class INPUT(Enum):
        INPUT1 = (26, 0)
        INPUT2 = (20, 1)
        INPUT3 = (21, 2)
        def __init__(self, bcmPin, index) -> None:
            self.bcmPin = bcmPin
            self.index = index
        @property
        def state(self):
            return automationhat.input[self.index].read()

    class RELAY(Enum):
        RELAY1 = (13, 0)
        RELAY2 = (19, 1)
        RELAY3 = (16, 2)
        def __init__(self, bcmPin, index) -> None:
            self.bcmPin = bcmPin
            self.index = index
        def on(self):
            return automationhat.relay[self.index].on()
        def off(self):
            return automationhat.relay[self.index].off()
        def toggle(self):
            return automationhat.relay[self.index].toggle()
        def flick(self):
            automationhat.relay[self.index].on()
            time.sleep(1)
            automationhat.relay[self.index].off()
            return True

    class RESISTOR(IntEnum):
        UP = GPIO.PUD_UP
        DOWN = GPIO.PUD_DOWN
        OFF = GPIO.PUD_OFF

    class CurrentDoorState(IntEnum):
        OPEN = 0
        CLOSED = 1
        OPENING = 2
        CLOSING = 3
        STOPPED = 4

    class TargetDoorState(IntEnum):
        OPEN = 0
        CLOSED = 1

    @property
    def currentDoorState(self):
        return self._currentDoorState
    
    @property
    def targetDoorState(self):
        return self._targetDoorState

    @targetDoorState.setter
    def targetDoorState(self, value):
        try:
            assert(value in AccessControl.TargetDoorState)
            self._targetDoorState = AccessControl.TargetDoorState(value)
            self._transition_state()
        except Exception as err:
            log.debug(err)

    @property
    def obstructionDetected(self):
        return (self._currentDoorState == AccessControl.CurrentDoorState.STOPPED)

    @property
    def name(self):
        return self._name
    
    def background(f):
        from functools import wraps
        @wraps(f)
        def wrapped(*args, **kwargs):
            loop = asyncio.get_event_loop()
            if callable(f):
                return loop.run_in_executor(None, f, *args, **kwargs)
            else:
                raise TypeError('Task must be a callable')    
        return wrapped

    
    def _read_input(self):
        temp=transformed=self.input.state
        if self._flipInput is True:
            transformed = temp ^ 1 # the pull_up will make closed = 1 which is the opposite so flip it using bitwise XOR
        return self.CurrentDoorState(transformed)

    def _push_button(self):
        self.relay.flick()
        log.debug('flicked relay for input {0}'.format(self.name))
        
    def _input_change(self, channel):
        if self._lock.acquire(blocking=False):
            log.debug(self._returnState('detected edge not in conjunction with transition, must be external'))
            self._currentDoorState = AccessControl.CurrentDoorState(self._read_input())
            self._targetDoorState = AccessControl.TargetDoorState(self._read_input())
            if self._mqttSvr is not None: self._pushMqttState()
            self._lock.release()
        else:
            log.debug(self._returnState('edge detected related to transition'))

    @background
    def _resync(self, interval=30):
        while True:
            log.debug(self._returnState('starting resync'))
            if self._lock.acquire(blocking=False): #resync job, intended to fire every TIMEOUT to resync state in event of power outage or missed event
                self._currentDoorState = AccessControl.CurrentDoorState(self._read_input())
                self._targetDoorState = AccessControl.TargetDoorState(self._currentDoorState)
                if self._mqttSvr is not None: self._pushMqttState()
                self._lock.release()
            time.sleep(interval) 

    @background
    def _awaitMqtt(self):
        while True:      
            try:
                topic = self.name+'/'+'setTargetDoorState'
                log.debug('awaiting mqtt subscription on topic: '+topic+' on '+self._mqttSvr+':'+str(self._mqttPort))
                msg = mqttsubscribe.simple(topic,hostname=self._mqttSvr, port=self._mqttPort, client_id=self.name+'_subscriber')
                msg = msg.payload.decode("utf-8")
                if msg == 'OPEN':
                    state = AccessControl.TargetDoorState.OPEN
                elif msg == 'CLOSED':
                    state = AccessControl.TargetDoorState.CLOSED
                log.debug('mqtt change detected, setting state to '+state.name)
                self._targetDoorState = state
                self._transition_state2()
            except Exception as err:
                log.debug('await except'+err)

    def _pushMqttState(self):
        try:
            log.debug(self._returnState('pushing to MQTT'))
            msg = [{'topic':self.name+'/'+'CurrentDoorState','payload':self._currentDoorState.name},
                {'topic':self.name+'/'+'TargetDoorState','payload':self._targetDoorState.name},
                {'topic':self.name+'/'+'ObstructionDetected','payload':self.obstructionDetected}]

            mqttpublish.multiple(msg,hostname=self._mqttSvr,
                                port=self._mqttPort,
                                client_id=self.name)
        except Exception as err:
            log.debug(self._mqttSvr)
            log.debug(err)

    def _returnState(self, description=""):
        return  {
            "controller" : self._name,
            "description" : description,
            "_currentDoorState" : self._currentDoorState.name,
            "_targetDoorState" : self._targetDoorState.name,
            "_readInput" : self._read_input().name
        }
    
    @background
    def _transition_state(self):
        self._transition_state2()
        lock = self._lock.acquire(blocking=True, timeout=1) #acquire a lock in case there's already a progressing state change notification or another task is already running
        if not lock:
            log.debug('transition aborted due to failed lock')
            return
        if self._targetDoorState.value != self._currentDoorState.value:
            log.debug(self._returnState('transition thread detects state mismatch'))
            possiblyRetry = (self._currentDoorState == AccessControl.CurrentDoorState.STOPPED)
            if self._targetDoorState == AccessControl.TargetDoorState.CLOSED:
                self._currentDoorState = AccessControl.CurrentDoorState.CLOSING
                if self._mqttSvr is not None: self._pushMqttState()
            elif self._targetDoorState == AccessControl.TargetDoorState.OPEN:
                self._currentDoorState = AccessControl.CurrentDoorState.OPENING
                if self._mqttSvr is not None: self._pushMqttState()
            while True:
                self.relay.flick()
                time.sleep(self._expectedDuration)
                if self._read_input().value == self._targetDoorState.value:
                    log.debug(self._returnState('transition thread confirms state equality'))
                    self._currentDoorState = AccessControl.CurrentDoorState(self._targetDoorState)
                    if self._mqttSvr is not None: self._pushMqttState()
                    break
                else:
                    log.debug(self._returnState('transition state shows mismatch'))
                    self._currentDoorState = AccessControl.CurrentDoorState.STOPPED 
                    if self._mqttSvr is not None: self._pushMqttState()
                    if possiblyRetry:
                        log.debug(self._returnState('transition state is retrying to clear error'))
                        possiblyRetry = False
                    else:
                        break
        else:
            log.debug(self._returnState('transition thread confirms no action'))
        self._lock.release()

    def _transition_state2(self):
        lock = self._lock.acquire(blocking=True, timeout=1) #acquire a lock in case there's already a progressing state change notification or another task is already running
        if not lock:
            log.debug('transition aborted due to failed lock')
            return
        if self._targetDoorState.value != self._currentDoorState.value:
            log.debug(self._returnState('transition thread detects state mismatch'))
            possiblyRetry = (self._currentDoorState == AccessControl.CurrentDoorState.STOPPED)
            if self._targetDoorState == AccessControl.TargetDoorState.CLOSED:
                self._currentDoorState = AccessControl.CurrentDoorState.CLOSING
                if self._mqttSvr is not None: self._pushMqttState()
            elif self._targetDoorState == AccessControl.TargetDoorState.OPEN:
                self._currentDoorState = AccessControl.CurrentDoorState.OPENING
                if self._mqttSvr is not None: self._pushMqttState()
            while True:
                self.relay.flick()
                time.sleep(self._expectedDuration)
                if self._read_input().value == self._targetDoorState.value:
                    log.debug(self._returnState('transition thread confirms state equality'))
                    self._currentDoorState = AccessControl.CurrentDoorState(self._targetDoorState)
                    if self._mqttSvr is not None: self._pushMqttState()
                    break
                else:
                    log.debug(self._returnState('transition state shows mismatch'))
                    self._currentDoorState = AccessControl.CurrentDoorState.STOPPED 
                    if self._mqttSvr is not None: self._pushMqttState()
                    if possiblyRetry:
                        log.debug(self._returnState('transition state is retrying to clear error'))
                        possiblyRetry = False
                    else:
                        break
        else:
            log.debug(self._returnState('transition thread confirms no action'))
        self._lock.release()
 
    def __init__(self, name, input, controlRelay, expectedDuration, resistorType = RESISTOR.OFF, flip = False, bounceTime = None, mqttSvr = None):
        automationhat.enable_auto_lights(True)
        GPIO.setmode(GPIO.BCM)
          
        assert(isinstance(input, self.INPUT))
        assert(isinstance(controlRelay, self.RELAY))
        assert(isinstance(name, str))
        assert(isinstance(resistorType, self.RESISTOR))
        assert(isinstance(expectedDuration, int))

        self.input = input
        self.relay = controlRelay
        self._resistorType = resistorType
        self._name = name.lower()
        self._flipInput = flip
        self._expectedDuration = expectedDuration
        self._currentDoorState = AccessControl.CurrentDoorState(self._read_input())
        self._targetDoorState = AccessControl.TargetDoorState(self._currentDoorState)
        self._obstructionDetected = False
        self._mqttSvr = None
        self._mqttPort = None
        self._lock = Lock()

        GPIO.setup(input.bcmPin, GPIO.IN, pull_up_down=resistorType)
        if resistorType is self.RESISTOR.UP or flip is True:
            self._flipInput = True

        if bounceTime is not None:
            assert(isinstance(bounceTime, int))    
            assert(bounceTime > 0 and bounceTime < 5000)
            GPIO.add_event_detect(self.input.bcmPin, GPIO.BOTH, callback=self._input_change, bouncetime=bounceTime)
        else:
            GPIO.add_event_detect(self.input.bcmPin, GPIO.BOTH, callback=self._input_change)

        self._resync() #start resync task

        if mqttSvr is not None:
            mqttUri = urlparse(mqttSvr)
            if mqttUri.scheme != 'mqtt':
                raise ValueError('ensure path start with mqtt://')
            self._mqttSvr = mqttUri.hostname
            if mqttSvr == '':
                raise ValueError('mqtt server hostname not set')
            self._mqttPort = 1883 if mqttUri.port is None else mqttUri.port
            self._awaitMqtt() #start subscription poller


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("name", nargs="?", default="access controler")
    parser.add_argument("-d", "--debug", action="store_true", default=False)
    parser.add_argument("-s", "--server", type=str)
    parser.add_argument("-m", "--mqtt", type=str)

    args = parser.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)
    else:
        logging.basicConfig(level=logging.INFO)

    accessInstances = []
    garage = AccessControl("garage", AccessControl.INPUT.INPUT1, AccessControl.RELAY.RELAY1, 18, AccessControl.RESISTOR.OFF, True, 200, args.mqtt) 
    gate = AccessControl("gate", AccessControl.INPUT.INPUT2, AccessControl.RELAY.RELAY2, 18, AccessControl.RESISTOR.OFF, True, 200, args.mqtt)  
    accessInstances.append(garage)
    accessInstances.append(gate)        

    if args.server:

        serverUri = urlparse(args.server)
        if serverUri.netloc == '':
            args.server = '//' + args.server
            serverUri = urlparse(args.server)

        if serverUri.scheme == 'http' and serverUri.port is None:
            serverUri.port = 80
        if serverUri.scheme == 'https' and serverUri.port is None:
            serverUri.port = 443

        @route('')
        @route('/')
        @route('/<controller>')
        @route('/<controller>/')
        @route('/<controller>/<var>')
        @route('/<controller>/<var>/')
        @route('/<controller>/<var>/<state>')
        def webcontroller(controller=None, var=None, state=None):
            controller = controller.lower() if controller is not None else None
            state = state.upper() if state is not None else None
            try:
                returnString = {}
                if controller is None: #return all controller info
                    if not len(accessInstances) > 0: raise TypeError('no controllers configured')
                    for x in accessInstances:
                        returnString.update({x.name: x._returnState("webcall")})
                    return returnString
                else: #match the controller
                    matches = [x for x in accessInstances if x.name == controller]
                    if not len(matches) == 1: raise TypeError('controller not found or matches more than one')
                    matchedController = matches[0]

                if var is None: #return controller info
                    returnString = matchedController._returnState("webcall")
                elif state is None: #return var value
                    try:    
                        returnString = {var : getattr(matchedController,var).name}
                    except:
                        pass
                    try:
                        returnString = {var : getattr(matchedController,var)}
                    except:
                        raise AttributeError('requested attribute should be targetDoorState or currentDoorState or obstructionDetected')
                else: #attempt to set var
                    try:
                        if (state.isdigit()):
                            setattr(matchedController,var,AccessControl.TargetDoorState(int(state)))
                        else:
                            setattr(matchedController,var,AccessControl.TargetDoorState[state])
                        returnString = {'Status': 'OK'}
                    except:
                        raise TypeError('state not valid for controller, must be OPEN/CLOSED/1/0')
            except TypeError as err:
                abort(400, err)
            except AttributeError as err:
                abort(400, err)
            except Exception as err:
                abort(400, err)
            return returnString 
        run(host=serverUri.hostname,port=serverUri.port, debug=args.debug)

if __name__ == '__main__':
    main()
    
