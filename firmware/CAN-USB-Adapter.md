# CAN-USB-Adapter

This adapter follows the python-can serial bus API packet format to transmit CAN frames over UART.

## Usage

```Python

COM_PORT = "COMx"  # COM port of the dongle

bus = can.Bus(interface="serial", channel=COM_PORT, baudrate=1000000)
```

## TODO

- [ ] support USB interface

