import os
import time
import struct
import threading

import math

import rath_recoil as recoil

DEVICE_ID = 10


transport = recoil.SocketCANTransport()
controller = recoil.MotorController(transport=transport, device_id=DEVICE_ID)

transport.enable()

do_calibration = False
if do_calibration:
    finished = threading.Event()
    
    controller.setMode(recoil.MotorController.MODE_CALIBRATION)

    def getFinishedState(controller, mode):
        if mode == recoil.MotorController.MODE_IDLE:
            finished.set()
            
    while not finished.is_set():
        controller.getMode(getFinishedState)
        time.sleep(0.1)

controller.setMode(recoil.MotorController.MODE_IDLE)
time.sleep(0.5)
controller.setMode(recoil.MotorController.MODE_POSITION)
# controller.setMode(recoil.MotorController.MODE_TORQUE)

controller_stat = {
    "velocity": 0,
    "position": 0,
    "torque": 0,
}

def velocityHandler(controller, data):
    controller_stat["velocity"] = data

def positionHandler(controller, data):
    controller_stat["position"] = data

def torqueHandler(controller, data):
    controller_stat["torque"] = data
    # if abs(data) > 0.02:
    #     controller1.setMode(recoil.MotorController.MODE_IDLE)
    # else:
    #     controller1.setMode(recoil.MotorController.MODE_POSITION)

try:
    while True:
        print(controller_stat["position"], controller_stat["velocity"], controller_stat["torque"])
        # controller.getMode(lambda controller, frame: print("mode:", frame.data))
        controller.getPositionMeasured(positionHandler)
        # controller1.getVelocityMeasured(velocityHandler)
        controller.getTorqueMeasured(torqueHandler)
        
        controller.setPositionTarget(5 * math.pi*math.sin(time.time()))
        # controller.setTorqueTarget(0.01)
        
        controller.feed()
        time.sleep(0.1)
except KeyboardInterrupt:
    controller.setMode(recoil.MotorController.MODE_IDLE)
    transport.disable()
