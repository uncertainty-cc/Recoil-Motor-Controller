import time
import math

import recoil


transport = recoil.SPICANTransport(port="can0", baudrate=500000)

ms = [recoil.MotorController(transport, device_id=i+1) for i in range(12)]

m1 = ms[0]
m3 = ms[2]
m5 = ms[4]
m7 = ms[6]
m9 = ms[8]
m11 = ms[10]
m2 = ms[1]
m4 = ms[3]
m6 = ms[5]
m8 = ms[7]
m10 = ms[9]
m12 = ms[11]

transport.start()

# init zero position
for m in ms:
    m.setPositionOffset(0)
    m.setPositionOffset(-m.getPositionMeasured())
    time.sleep(0.01)


try:
    while True:
        for m in ms:
            pos = m.getPositionMeasured()
            print("{:.3f}".format(pos), end="\t")

            time.sleep(0.01)
        print()

except KeyboardInterrupt:
    print("Stopped")
    

transport.stop()
