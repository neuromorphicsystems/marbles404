- [Basic usage](#basic-usage)
  - [Software dependencies](#software-dependencies)
  - [Examples](#examples)
- [Advanced configuration](#advanced-configuration)
  - [Hardware components](#hardware-components)
  - [Motors configuration](#motors-configuration)


# Basic usage

## Software dependencies

```sh
python3.13 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
brew install hidapi # macOS
```

## Examples

To run the demo, make sure that the controller, the motors board, and the EVK4 are connected to the machine.

```sh
python demo.py
```

# Advanced configuration

## Hardware components

__Motors__: Dynamixel XL430-W250-T (https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)
__Board__: OpenRB-150 (https://emanual.robotis.com/docs/en/parts/controller/openrb-150/)
__Controller__: GameSir Nova Lite (https://gamesir.com/products/gamesir-nova-lite)


## Motors configuration

The Dynamixel Wizard 2.0 (https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/) can be used to change the motor properties (PID parameters, LED...).

The firmware installed on the OpenRB-150 board (__usb_to_dynamixel__, https://github.com/ROBOTIS-GIT/OpenRB-150/tree/3e2b07eb676097503d443f702d2a64519e7ae8f8/libraries/OpenRB-150/examples/usb_to_dynamixel) is a simple pass-through between the serial connection (computer to board) and the DXL bus (board to motors).

Note that this firmware must be installed for the Dynamixel Wizard to work. If the sketch needs to be re-flashed, see https://emanual.robotis.com/docs/en/parts/controller/openrb-150/#development-environment for details on how to install the development environment.

The system is configured as follows:

- The baud rate is `57600`
- The front motor (left-right tilt) has the ID `001`
- The right motor (front-back tilt) has the ID `002`
- The marble elevator motor (back) has the ID `003`

The bus' protocol is described at https://emanual.robotis.com/docs/en/dxl/protocol2/.
