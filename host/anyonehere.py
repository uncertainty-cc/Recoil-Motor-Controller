import time

import serial
import can
import can.interfaces.serial

import recoil

transport = recoil.SerialCANTransport(port="COM34", baudrate=1000000)

motor = recoil.MotorController(transport, device_id=11)
# controller3 = recoil.MotorController(transport, device_id=3)

transport.start()

try:
    while True:
        print("ping:", motor.ping())
        # print("ping:", controller3.ping())

        time.sleep(0.02)
except KeyboardInterrupt:
    transport.stop()
