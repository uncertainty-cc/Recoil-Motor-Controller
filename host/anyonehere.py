import time

import serial
import can
import can.interfaces.serial

import recoil

transport = recoil.SerialCANTransport(port="COM34", baudrate=1000000)

controller2 = recoil.MotorController(transport, device_id=2)
controller3 = recoil.MotorController(transport, device_id=4)

transport.start()

try:
    while True:
        print("ping:", controller2.ping())
        print("ping:", controller3.ping())

        time.sleep(0.1)
except KeyboardInterrupt:
    transport.stop()
