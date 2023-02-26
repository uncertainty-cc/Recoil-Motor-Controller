import time

import recoil

transport = recoil.SPICANTransport(port="can0", baudrate=1000000)

controller1 = recoil.MotorController(transport, device_id=1)
controller3 = recoil.MotorController(transport, device_id=3)
controller5 = recoil.MotorController(transport, device_id=5)
controller7 = recoil.MotorController(transport, device_id=7)
controller9 = recoil.MotorController(transport, device_id=9)
controller11 = recoil.MotorController(transport, device_id=11)

controller2 = recoil.MotorController(transport, device_id=2)
controller4 = recoil.MotorController(transport, device_id=4)
controller6 = recoil.MotorController(transport, device_id=6)
controller8 = recoil.MotorController(transport, device_id=8)
controller10 = recoil.MotorController(transport, device_id=10)
controller12 = recoil.MotorController(transport, device_id=12)

transport.start()

try:
    while True:
        print(time.time(), end="\t")
        print("1:", controller1.ping(), end="\t")
        print("3:", controller3.ping(), end="\t")
        print("5:", controller5.ping(), end="\t")
        print("7:", controller7.ping(), end="\t")
        print("9:", controller9.ping(), end="\t")
        print("11:", controller11.ping(), end="\t")
        print("2:", controller2.ping(), end="\t")
        print("4:", controller4.ping(), end="\t")
        print("6:", controller6.ping(), end="\t")
        print("8:", controller8.ping(), end="\t")
        print("10:", controller10.ping(), end="\t")
        print("12:", controller12.ping())

        time.sleep(1)
except KeyboardInterrupt:
    transport.stop()
