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
    m.setPositionTarget(0)

for m in ms:
    # m.setMode(recoil.Mode.POSITION)
    m.setMode(recoil.Mode.DAMPING)

target_position = 0
t = time.time()



def generateSinCosTrajectory():
    return [
        math.sin(time.time() * 4) * 5,
        math.cos(time.time() * 4) * 5,
        math.sin(time.time() * 4) * 5
    ]




try:
    max_torque_limit = 1
    for i in range(100):
        torque_limit = i / 100. * max_torque_limit
        m2.setTorqueVelocityLimit(torque_limit, 20)
        target_positions = [
            math.sin(time.time() * 2) * 14 - 12, #0,
            0, 
            0, #math.sin(time.time()) * 3, 
            -math.sin(time.time() * 2) * 10 + 11, 
            0]
        
        # m2.setPositionTarget(target_positions[0])
        # m4.setPositionTarget(target_positions[1])
        # m6.setPositionTarget(target_positions[2])
        # m8.setPositionTarget(target_positions[3])
        # m10.setPositionTarget(target_positions[4])

        for m in ms:
            m.feed()

        time.sleep(0.02)
    
    while True:
        # if time.time() - t > 1:
        #     target_position = 0
        # if time.time() - t > 2:
        #     target_position = 5
        #     t = time.time()

        # target_positions = generateSinCosTrajectory()
        # target_positions = [
        #     math.sin(time.time() * 2) * 10 - 12,
        #     0,
        #     0]

        target_positions = [
            math.sin(time.time() * 2) * 14 - 12, #0,
            0, 
            0, #math.sin(time.time()) * 3, 
            -math.sin(time.time() * 2) * 10 + 11, 
            0]

        # m2.setPositionTarget(target_positions[0])
        # m4.setPositionTarget(target_positions[1])
        # m6.setPositionTarget(target_positions[2])
        # m8.setPositionTarget(target_positions[3])
        # m10.setPositionTarget(target_positions[4])
        
        print(m2.getPositionMeasured(), m4.getPositionMeasured(),  m6.getPositionMeasured())

        for m in ms:
            m.feed()

        time.sleep(0.02)

except KeyboardInterrupt:
    for m in ms:
        m.setMode(recoil.Mode.DAMPING)
    print("waiting to stop...")
    print("Press Ctrl+C again to stop")
    try:
        while True:
            for m in ms:
                m.feed()
            time.sleep(0.02)
    except KeyboardInterrupt:
        for m in ms:
            m.setMode(recoil.Mode.IDLE)
        print("stopped.")
        pass


transport.stop()
