import os
import time
import struct
import threading

import math

import rath_recoil as recoil


transport = recoil.SocketCANTransport()

controller1 = recoil.MotorController(transport=transport, device_id=1)
controller2 = recoil.MotorController(transport=transport, device_id=2)
controller3 = recoil.MotorController(transport=transport, device_id=3)

transport.enable()

do_calibration = False
if do_calibration:
    finished = threading.Event()
    
    controller1.setMode(recoil.MotorController.MODE_CALIBRATION)

    def setstate(controller, mode):
        if mode == recoil.MotorController.MODE_IDLE:
            finished.set()
            
    while not finished.is_set():
        controller1.getMode(setstate)
        time.sleep(0.1)

controller1.setMode(recoil.MotorController.MODE_IDLE)
controller2.setMode(recoil.MotorController.MODE_IDLE)
controller3.setMode(recoil.MotorController.MODE_IDLE)
time.sleep(0.5)
controller1.setMode(recoil.MotorController.MODE_POSITION)
controller2.setMode(recoil.MotorController.MODE_POSITION)
controller3.setMode(recoil.MotorController.MODE_POSITION)

try:
    while True:
        # controller.getMode(lambda controller, frame: print("mode:", frame.data))
        controller1.getPositionMeasured(lambda controller, position: print("position:", position))
        
        controller1.setPositionTarget(8*math.sin(10*time.time()))
        controller2.setPositionTarget(-2)
        controller3.setPositionTarget(0)
        #controller1.setPositionTarget(10*math.sin(2*time.time()))
        #controller2.setPositionTarget(10*(math.sin(time.time())-2))
        #controller3.setPositionTarget(20*math.sin(time.time()))

        # controller1.setPositionTarget(0)
        # controller2.setPositionTarget(0)
        # controller3.setPositionTarget(0)
        controller1.feed()
        controller2.feed()
        controller3.feed()
        time.sleep(0.01)
except KeyboardInterrupt:
    transport.disable()
