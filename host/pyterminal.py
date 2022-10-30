import struct
import time
import math
import threading

import serial
from dotxboxcontroller import XboxController, Hand

# device_id 4 bits
# func_id   7 bits

class Handler:
    def __init__(self, frame, device, callback=None):
        self.frame = frame
        self.device = device
        self.callback = callback
    

debug = 0

class CANFrame:
    CAN_ID_STANDARD = 0
    CAN_ID_EXTENDED = 1
    
    CAN_FRAME_REMOTE = 1
    CAN_FRAME_DATA = 1
    
    def __init__(self, device_id=0, func_id=0, size=0, data=b"",
                 id_type=CAN_ID_STANDARD, frame_type=CAN_FRAME_REMOTE):
        self.device_id = device_id
        self.func_id = func_id
        self.id_type = id_type
        self.frame_type = frame_type
        self.size = size
        self.data = data

class SerialTransport:
    def __init__(self, port="COM1", baudrate=115200):
        self.port = port
        self._ser = serial.Serial(port=self.port, baudrate=baudrate, timeout=None)
        self.handlers = []

    def transmitCANFrame(self, frame):
        can_id = (frame.func_id << 4) | frame.device_id
        # Big endian here to be compatible with CAN ID arbitration order
        header = struct.pack(">HBB", can_id, frame.size, frame.frame_type)
        fill = b'\x00' * (8 - len(frame.data))
        buffer = header + frame.data + fill
        if debug:
            print("[DEBUG] transmit", buffer)
        self._ser.write(buffer)

    def transmitReceiveCANFrame(self, handler):
        self.handlers.append(handler)
        self.transmitCANFrame(handler.frame)

    def receiveCANFrame(self):
        while True:
            frame = CANFrame()
            if debug:
                print("[DEBUG] receiving frame...")
            buffer = self._ser.read(4)
            if not buffer:
                continue
            if debug:
                print("[DEBUG] receive", buffer)
            # Big endian here to be compatible with CAN ID arbitration order
            can_id, frame.size, frame.frame_type = struct.unpack(">HBB", buffer)
            frame.device_id = can_id & 0x0F
            frame.func_id = can_id >> 4
            buffer = self._ser.read(frame.size)
            frame.data = buffer

            for handler in self.handlers:
                if handler.frame.device_id == frame.device_id and handler.frame.func_id == frame.func_id:
                    handler.device.handleRX(frame)
                    if handler.callback:
                        handler.callback(handler.device, frame)
                    self.handlers.remove(handler)


    def start(self):        
        t = threading.Thread(target=self.receiveCANFrame)
        t.start()


class RecoilMotorController:
    CAN_ID_ESTOP              = 0x00
    CAN_ID_ID                 = 0x01
    CAN_ID_VERSION            = 0x02
    CAN_ID_HEARTBEAT          = 0x04

    CAN_ID_MODE               = 0x10
    CAN_ID_FLASH              = 0x11

    CAN_ID_TORQUE_MEASURED    = 0x20
    CAN_ID_TORQUE_TARGET      = 0x21
    CAN_ID_VELOCITY_MEASURED  = 0x22
    CAN_ID_VELOCITY_TARGET    = 0x23
    CAN_ID_POSITION_MEASURED  = 0x24
    CAN_ID_POSITION_TARGET    = 0x25
    CAN_ID_POSITION_KP_KI     = 0x26
    CAN_ID_POSITION_KD        = 0x27
    CAN_ID_IQ_KP_KI           = 0x28
    CAN_ID_ID_KP_KI           = 0x29

    CAN_ID_BUS_VOLTAGE        = 0x30
    CAN_ID_MOTOR_SPEC         = 0x40
    CAN_ID_MOTOR_FLUX_OFFSET  = 0x41
    CAN_ID_ENCODER_N_ROTATION = 0x42

    CAN_ID_CURRENT_DQ         = 0x44
    CAN_ID_CURRENT_AB         = 0x45

    CAN_ID_CURRENTCONTROLLER_IQ = 0x50
    CAN_ID_CURRENTCONTROLLER_ID = 0x51
    CAN_ID_CURRENTCONTROLLER_VQ = 0x52
    CAN_ID_CURRENTCONTROLLER_VD = 0x53

    CAN_ID_PING               = 0x7F
    
    MODE_DISABLED           = 0x00
    MODE_IDLE               = 0x01
    MODE_CALIBRATION        = 0x05
    MODE_TORQUE             = 0x10
    MODE_VELOCITY           = 0x11
    MODE_POSITION           = 0x12
    MODE_OPEN_VDQ           = 0x22
    MODE_OPEN_VALPHABETA    = 0x23
    MODE_OPEN_VABC          = 0x24
    MODE_OPEN_IDQ           = 0x25
    MODE_DEBUG              = 0x80
    
    def __init__(self, transport, device_id=1):
        self.transport = transport
        self.device_id = device_id
        
        self.motor_position_measured = 0
        self.motor_velocity_measured = 0

        self.motor_position_target = 0

        self.mode = self.MODE_DISABLED

    def getMode(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_MODE, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))
    
    def setMode(self, mode):
        frame = CANFrame(self.device_id, self.CAN_ID_MODE, 1, struct.pack("<B", mode))
        self.transport.transmitCANFrame(frame)
    
    def getPosition(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_MEASURED, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))
    
    def getTargetPosition(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_TARGET, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))
    
    def setTargetPosition(self, position):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_TARGET, 4, struct.pack("<f", position))
        self.transport.transmitCANFrame(frame)
    
    def getVelocity(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_VELOCITY_MEASURED, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))
        
    def getIQ(self):
        frame = CANFrame(self.device_id, self.CAN_ID_CURRENTCONTROLLER_IQ, 0)
        self.transport.transmitCANFrame(frame)
        
        frame = self.transport.receiveCANFrame()
        if not frame:
            print("timeout")
            return None
        i_q_target, i_q_measured = struct.unpack("<ff", frame.data)
        return i_q_target, i_q_measured

    def feed(self):
        frame = CANFrame(self.device_id, self.CAN_ID_HEARTBEAT, 1, b"0")
        self.transport.transmitCANFrame(frame)

    def ping(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_PING, 0)
        self.transport.transmitReceiveCANFrame(Handler(frame, self, callback))

    def handleRX(self, frame):
        if frame.func_id == self.CAN_ID_MODE:
            self.mode = frame.data[0]
        
        if frame.func_id == self.CAN_ID_POSITION_MEASURED:
            position_measured, = struct.unpack("<f", frame.data[0:4])
            self.motor_position_measured = position_measured

        if frame.func_id == self.CAN_ID_POSITION_TARGET:
            position_target, = struct.unpack("<f", frame.data[0:4])
            self.motor_position_target = position_target
            

        if frame.func_id == self.CAN_ID_VELOCITY_MEASURED:
            velocity_measured, = struct.unpack("<f", frame.data[0:4])
            self.motor_velocity_measured = velocity_measured    

        if frame.func_id == self.CAN_ID_PING:
            print(device.device_id, frame.data[0])
            

stick = XboxController(0)

ser = SerialTransport("COM23", baudrate=1000000)

motor_0 = RecoilMotorController(ser, 1)
motor_1 = RecoilMotorController(ser, 2)

ser.start()


def ping_interrupt_handler(device, frame):
    print(device.device_id, frame.device_id)

def print_mode_handler(device, frame):
    print(device.device_id, frame.data[0], frame.data[1])


def main():
    
    motor_0.setMode(RecoilMotorController.MODE_IDLE)
    motor_1.setMode(RecoilMotorController.MODE_IDLE)

    time.sleep(0.1)

    motor_0.setMode(RecoilMotorController.MODE_POSITION)
    motor_1.setMode(RecoilMotorController.MODE_POSITION)
    time.sleep(0.1)

    
    while True:
        stick.update()
        
        t = time.time()

        
        motor_0.getMode()
        motor_1.getMode()
        
        
        val0 = 0
        val1 = 0
        val0 = stick.getX(Hand.LEFT) * 5
        val1 = stick.getY(Hand.RIGHT) * 10
        

        
        motor_0.getPosition()
        motor_1.getPosition()
        motor_0.getTargetPosition()
        motor_1.getTargetPosition()
        motor_0.setTargetPosition(val0)
        motor_1.setTargetPosition(val1)
        
        print(motor_0.mode, motor_1.mode, val0, val1,
              motor_0.motor_position_target, motor_1.motor_position_target,
              motor_0.motor_position_measured, motor_1.motor_position_measured)
        #print(motor_0.motor.velocity_measured, motor_1.motor.velocity_measured)

        motor_0.feed()
        motor_1.feed()
        
        time.sleep(0.01)
        
        #print(time.time() - t)

if __name__ == "__main__":
    main()
    #motor_1.setMode(RecoilMotorController.MODE_CALIBRATION)
