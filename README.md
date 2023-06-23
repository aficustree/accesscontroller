# AccessController

AccessController is a Python library for interacting with a garage / gate / door controller through raspberry PI GPIOs. This version uses higher-order commands provided via an [AutomationHAT](https://github.com/pimoroni/automation-hat) connected to these pins. AccessController may be used as a package but also comes with a webserver and CLI interface

## Installation

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install accesscontroller.

```bash
pip3 install accesscontroller @ git+https://github.com/aficustree/accesscontroller.git
```

or via poetry
```bash
git clone https://github.com/aficustree/accesscontroller.git
cd ./accesscontroller
poetry install .
poetry run accesscontroller
```

## Usage

```
usage: accesscontroller.py [-h] [-d] [-s SERVER] [name]

positional arguments:
  name

optional arguments:
  -h, --help            show this help message and exit
  -d, --debug
  -s SERVER, --server SERVER
```

### Package Usage

Instantiate an instance of AccessController such as,

```
gate = AccessController(
    name, 
    input, 
    controlRelay, 
    expectedDuration, 
    resistorType = RESISTOR.OFF, 
    flip = False, 
    bounceTime = None)`
```
where

- `name` = the name you wish your controller to be known as in the CLI or server

- `input` = an instance of the Enum Type INPUT used to detect state
controlRelay = an instance of the Enum type RELAY used to trigger the relay

- `expectedDuration` = the expected duration of the door opening/closing in seconds

- `resistorType` = an instance of the RESISTOR type reference to the internal pullup/down resistors in the RPi UP (pull-up), DOWN (pull-down) or NONE (resistor provided externally)

- `flip` = By default it assumes closed is pulled HIGH (1). If it's pulled LOW (0) when closed, then set flip to `True`

- `bounceTime` = time in MILLISECONDS for the system to supress any relay bounce (recommend 500), from 0-5000.

The controller works in a manner suitable for direct integration into the Apple HomeKit ecosystem but should align well to other platforms or DIY.

The main functions of use for `gate=AccessController(...)` would look like

- property `gate.doorCurrentState` returns a  `AccessController.DoorCurrentState` used to indicate the state of the door
- property `gate.doorTargetState` returns a `AccessController.TargetDoorState` used to indicate the state the door should be in.
- setting the property `gate.doorTargetState` to a `AccessController.TargetDoorState` different than the `gate.doorCurrentState` will trigger a transition event attempting to move the door into the desired state. This call is *non-blocking*. In the event that the door fails to transition to the diesired state by the `expectedDuration`, then `gate.obstructionDetected` is set to `True` and `gate.doorTargetState` is reset to the current status of the input sensor.

### Sever Usage

Pass the `-s` flag with a port number (example: `-s 8080`) and the development server will await responses at `http://a.b.c.d:8080`. 

```
http://a.b.c.d:8080/[<controller_name>]/[<variable_name>]/[action]
```
All paramters are optional and work similar to the platform use-case.

- `http://a.b.c.d:8080/` returns a JSON of all configured controllers and their debugh output (read of internal state)
- `http://a.b.c.d:8080/gate` would return a JSON of the specified controller and its debug output
- `http://a.b.c.d:8080/gate/[doorTargetState|currentTargetState]` would return the respective state object as a JSON (example: `{'doorTargetState' : 'OPEN'}`)
- `http://a.b.c.d:8080/gate/doorTargetState/[1|0|OPEN|CLOSED]` would trigger a transition using the rules as described in true platform use.
- `http://a.b.c.d:8080/gate/obstructionDetected` would return True or False as a JSON

You may use a similar form to return all internal state variables provided case sensativity is respected

## To-Dos

- TODO: Gracefully close bottle and resync tasks on sigterm
- TODO: Handle case insensitivity for paths
- TODO: Create CLI
- TODO: Replace asserts
- TODO: Add MQTT

## Contributing

Pull requests are welcome. Submitting ideas via issues are also welcome. 

## License

[MIT](https://choosealicense.com/licenses/mit/). A [copy of this license](./LICENSE) is attached to this repo. 

Copyright (c) [2023] aficustree