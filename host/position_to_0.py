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

print("set mode")
for m in ms:
    m.setMode(recoil.Mode.IDLE)

time.sleep(0.1)

print("set position")
for m in ms:
    m.setTorqueVelocityLimit(0.01, 20)
    m.setPositionTarget(0)
    time.sleep(0.01)


for m in ms:
    m.setMode(recoil.Mode.POSITION)
    # m.setMode(recoil.Mode.DAMPING)
    pass

t = time.time()


def generateZeroTragectory(t):
    target_positions = [0] * 12
    return target_positions

def generateTaiTuiTragectory(t):
    target_positions = [0] * 12
    
    target_positions[0] = 10 * t
    target_positions[1] = -10 * t

    target_positions[6] = -20 * t
    target_positions[7] = 20 * t

    target_positions[10] = 10 * t
    target_positions[11] = -10 * t

    # target_positions[0] = 5 * t
    # target_positions[1] = -5 * t

    # target_positions[6] = -10 * t
    # target_positions[7] = 10 * t

    # target_positions[10] = 5 * t
    # target_positions[11] = -5 * t

    return target_positions


def timetag():
    return 0.5 * (math.sin(time.time() * 1.5) + 1)

stop = 0

try:
    max_torque_limit = 1.

    target_positions = [0] * 12

    print("Ramping up...")

    for i in range(100):
        if stop:
            break
        try:
            # target_positions = generateZeroTragectory(timetag())
            target_positions = generateTaiTuiTragectory(timetag())

            torque_limit = i / 100. * max_torque_limit
            
            for i, m in enumerate(ms):
                m.setPositionTarget(target_positions[i])

            time.sleep(0.01)
            for m in ms:
                m.setTorqueVelocityLimit(torque_limit, 20)
                    
            print("{:.3f} / {:.3f}".format(torque_limit, max_torque_limit))

            for m in ms:
                m.feed()

            time.sleep(0.01)
        except KeyboardInterrupt:
            stop = 1
    
    print("Finish ramping up")
    while not stop:
        try:
            # target_positions = generateZeroTragectory(timetag())
            target_positions = generateTaiTuiTragectory(timetag())

            for i, m in enumerate(ms):
                m.setPositionTarget(target_positions[i])
            
            # for m in ms:
            #     print(m.getPositionMeasured(), end="\t")
            # print()
            time.sleep(0.01)

            for m in ms:
                m.feed()

            time.sleep(0.01)
        except KeyboardInterrupt:
            stop = 1

except KeyboardInterrupt:
    pass

time.sleep(0.01)
for m in ms:
    m.setMode(recoil.Mode.DAMPING)
print("waiting to stop...")
print("Press Ctrl+C again to stop")
time.sleep(0.01)
try:
    while True:
        for m in ms:
            m.feed()
        time.sleep(0.05)
except KeyboardInterrupt:
    for m in ms:
        m.setMode(recoil.Mode.IDLE)
    print("stopped.")
    pass


transport.stop()
