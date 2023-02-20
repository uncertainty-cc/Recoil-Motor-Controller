import time

import serial
import can
import can.interfaces.serial

import recoil


info = {
    "position_measured": 0,
    "position_setpoint": 0
}


transport = recoil.SerialCANTransport(port="COM34", baudrate=1000000)

controller2 = recoil.MotorController(transport, device_id=2)
controller3 = recoil.MotorController(transport, device_id=3)

transport.start()

print("set mode")
controller2.setMode(recoil.Mode.IDLE)
controller3.setMode(recoil.Mode.IDLE)

time.sleep(0.1)

print("set position")
controller2.setPositionTarget(0)
controller3.setPositionTarget(0)
controller2.setMode(recoil.Mode.POSITION)
controller3.setMode(recoil.Mode.POSITION)

target_position = 0
t = time.time()

try:
    while True:
        if time.time() - t > 1:
            target_position = 0
        if time.time() - t > 2:
            target_position = 5
            t = time.time()
        
        controller2.setPositionTarget(target_position)
        controller3.setPositionTarget(target_position)
        
        print(controller2.getPositionMeasured(), controller3.getPositionMeasured())

        controller2.feed()
        controller3.feed()
        time.sleep(0.01)

except KeyboardInterrupt:
    controller2.setMode(recoil.Mode.IDLE)
    controller3.setMode(recoil.Mode.IDLE)
    pass

transport.stop()