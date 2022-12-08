import os
import time
import struct
import threading

import math

import rath_recoil as recoil

transport = recoil.SocketCANTransport()
shooter_motor_0 = recoil.MotorController(transport=transport, device_id=10)
shooter_motor_1 = recoil.MotorController(transport=transport, device_id=11)
pitch_motor = recoil.MotorController(transport=transport, device_id=12)
yaw_motor = recoil.MotorController(transport=transport, device_id=13)

transport.enable()


shooter_motor_0.setMode(recoil.MotorController.MODE_IDLE)
shooter_motor_1.setMode(recoil.MotorController.MODE_IDLE)
pitch_motor.setMode(recoil.MotorController.MODE_IDLE)
yaw_motor.setMode(recoil.MotorController.MODE_IDLE)

time.sleep(0.5)
shooter_motor_0.setMode(recoil.MotorController.MODE_TORQUE)
shooter_motor_1.setMode(recoil.MotorController.MODE_TORQUE)
pitch_motor.setMode(recoil.MotorController.MODE_IDLE)
yaw_motor.setMode(recoil.MotorController.MODE_POSITION)


def torqueHandler(controller, data):
    controller.torque_measured = data

def velocityHandler(controller, data):
    controller.velocity_measured = data

def positionHandler(controller, data):
    controller.position_measured = data

try:
    while True:
        print(shooter_motor_0.velocity_measured, shooter_motor_1.velocity_measured, pitch_motor.position_measured, yaw_motor.position_measured)
        
        pitch_motor.getPositionMeasured(positionHandler)
        yaw_motor.getPositionMeasured(positionHandler)
        shooter_motor_0.getVelocityMeasured(torqueHandler)
        shooter_motor_1.getVelocityMeasured(torqueHandler)
        
        pitch_target = 0
        yaw_target = 0
        shooter_vel = 0

        pitch_motor.setPositionTarget(pitch_target)
        yaw_motor.setPositionTarget(yaw_target)
        shooter_motor_0.setTorqueTarget(shooter_vel)
        shooter_motor_1.setTorqueTarget(shooter_vel)
        
        pitch_motor.feed()
        yaw_motor.feed()
        shooter_motor_0.feed()
        shooter_motor_1.feed()

        time.sleep(0.1)
        
except KeyboardInterrupt:
    shooter_motor_0.setMode(recoil.MotorController.MODE_IDLE)
    shooter_motor_1.setMode(recoil.MotorController.MODE_IDLE)
    pitch_motor.setMode(recoil.MotorController.MODE_IDLE)
    yaw_motor.setMode(recoil.MotorController.MODE_IDLE)
    transport.disable()
