import time

import serial
import can
import can.interfaces.serial

import recoil

transport = recoil.SerialCANTransport(port="COM34", baudrate=1000000)

controller2 = recoil.MotorController(transport, device_id=2)

transport.start()

try:
    while True:
        t = time.time()
        for i in range(1000):
            controller2.ping()
        print("ping:", time.time() - t)

except KeyboardInterrupt:
    transport.stop()