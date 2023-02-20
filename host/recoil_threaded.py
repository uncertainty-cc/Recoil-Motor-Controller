import time
import os
import struct
import threading

import can
import serial

class Mode:
    DISABLED             = 0x00
    IDLE                 = 0x01
    # these are special modes
    DAMPING              = 0x02
    CALIBRATION          = 0x05
    # these are closed-loop modes
    CURRENT              = 0x10
    TORQUE               = 0x11
    VELOCITY             = 0x12
    POSITION             = 0x13
    # these are open-loop modes
    VABC_OVERRIDE        = 0x20
    VALPHABETA_OVERRIDE  = 0x21
    VQD_OVERRIDE         = 0x22
    IQD_OVERRIDE         = 0x23
    DEBUG                = 0x80

class CAN_ID:
    ESTOP              = 0x00
    ID                 = 0x01
    VERSION            = 0x02
    SAFETY_WATCHDOG    = 0x04
    MODE_ERROR         = 0x06
    FLASH              = 0x0F

    FAST_FRAME_0       = 0x10
    FAST_FRAME_1       = 0x11

    ENCODER_CPR_OFFSET               = 0x20
    ENCODER_FILTER                   = 0x21
    ENCODER_POSITION_RAW_N_ROTATIONS = 0x22
    POWERSTAGE_VOLTAGE_THRESHOLD     = 0x23
    POWERSTAGE_FILTER                = 0x24
    POWERSTAGE_BUS_VOLTAGE_MEASURED  = 0x25
    MOTOR_POLE_PAIR_KV               = 0x26
    MOTOR_PHASE_ORDER_FLUX_OFFSET    = 0x27

    CURRENT_KP_KI                    = 0x30
    CURRENT_LIMIT                    = 0x31
    CURRENT_IA_IB_MEASURED           = 0x32
    CURRENT_IC_MEASURED              = 0x33
    CURRENT_VA_VB_SETPOINT           = 0x34
    CURRENT_VC_SETPOINT              = 0x35
    CURRENT_IALPHA_IBETA_MEASURED    = 0x36
    CURRENT_VALPHA_VBETA_SETPOINT    = 0x37
    CURRENT_VQ_VD_TARGET             = 0x38
    CURRENT_VQ_VD_SETPOINT           = 0x39
    CURRENT_IQ_ID_TARGET             = 0x3A
    CURRENT_IQ_ID_MEASURED           = 0x3B
    CURRENT_IQ_ID_SETPOINT           = 0x3C
    CURRENT_IQ_ID_INTEGRATOR         = 0x3D

    POSITION_KP_KI                   = 0x40
    VELOCITY_KP_KI                   = 0x41
    TORQUE_VELOCITY_LIMIT            = 0x42
    POSITION_LIMIT                   = 0x43
    TORQUE_TARGET                    = 0x44
    TORQUE_MEASURED_SETPOINT         = 0x45
    VELOCITY_TARGET                  = 0x46
    VELOCITY_MEASURED_SETPOINT       = 0x47
    POSITION_TARGET                  = 0x48
    POSITION_MEASURED_SETPOINT       = 0x49
    POSITION_VELOCITY_INTEGRATOR     = 0x4A

    PING               = 0x7F


class CANFrame:
    ID_STANDARD = 0
    ID_EXTENDED = 1
    
    def __init__(self, 
            device_id=0, 
            func_id=0, 
            size=0, 
            data=b"",
            id_type=ID_STANDARD
        ):
        self.device_id = device_id
        self.func_id = func_id
        self.size = size
        self.data = data
        self.id_type = id_type


class SerialCANTransport:
    def __init__(self, port="COM1", baudrate=1000000):
        self.port = port
        self.baudrate = baudrate
        self._interface = None
        self._handlers = []
        self._killed = threading.Event()
        self._rx_handler_thread = None

    def stop(self):
        self._killed.set()
        self._rx_handler_thread.join()
        self._rx_handler_thread = None

    def start(self):
        self._killed.clear()
        self.connect()

        self._rx_handler_thread = threading.Thread(target=self._handleRX)
        self._rx_handler_thread.start()
        print("starrted")

    def connect(self):
        while not self._interface:
            try:
                self._interface = can.Bus(interface="serial", channel=self.port, baudrate=self.baudrate)
            except serial.serialutil.SerialException as e:
                print(e)
        print("connected")

    def transmit(self, frame, controller=None, callback=None):
        can_id = (frame.func_id << 4) | frame.device_id

        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=False,
            data=frame.data)

        if callback:
            self._handlers.append((frame, controller, callback))
        self._interface.send(msg)
    
    def receive(self):
        try:
            msg = self._interface.recv(timeout=0.1) # blocking
        except can.exceptions.CanOperationError:
            return None
        if not msg:
            return None
        frame = CANFrame(
            device_id = msg.arbitration_id & 0x0F,
            func_id = msg.arbitration_id >> 4,
            size = msg.dlc,
            data = msg.data
        )
        return frame
    
    def _handleRX(self):
        while not self._killed.is_set():
            rx_frame = self.receive()

            if not rx_frame:
                continue
            
            # print(rx_frame)
            for handler in self._handlers:
                tx_frame, controller, callback = handler
                if (rx_frame.device_id == tx_frame.device_id) and (rx_frame.func_id == tx_frame.func_id):
                    callback(controller, rx_frame)
                    self._handlers.remove(handler)


class MotorController:
    def __init__(self, transport, device_id=1):
        self.transport = transport
        self.device_id = device_id
        
        self.mode = Mode.DISABLED
        self.firmware_version = ""

    @staticmethod
    def unpack(format, data, index):
        try:
            return struct.unpack(format, data)[index]
        except struct.error as e:
            print("warning:", e)
            return 0
    
    @staticmethod
    def onPing(self, id):
        pass

    def ping(self, callback=None):
        frame = CANFrame(self.device_id, CAN_ID.PING, 0)
        if not callback:
            callback = MotorController.onPing
        callback_wrap = lambda controller, frame: callback(controller, MotorController.unpack("<B", frame.data, 0))
        self.transport.transmit(frame, self, callback_wrap)
    
    @staticmethod
    def onGetMode(self, mode):
        pass

    def getMode(self, callback=None):
        frame = CANFrame(self.device_id, CAN_ID.MODE_ERROR, 0)
        if not callback:
            callback = MotorController.onGetMode
        callback_wrap = lambda controller, frame: callback(controller, MotorController.unpack("<HH", frame.data, 0))
        self.transport.transmit(frame, self, callback_wrap)

    def setMode(self, mode, callback=None, clear_error=False):
        frame = CANFrame(self.device_id, CAN_ID.MODE_ERROR, 4, struct.pack("<HH", mode, 1 if clear_error else 0))
        if not callback:
            callback = MotorController.onGetMode
        callback_wrap = lambda controller, frame: callback(controller, MotorController.unpack("<HH", frame.data, 0))
        self.transport.transmit(frame, self, callback_wrap)
    
    @staticmethod
    def onGetPositionMeasured(self, data):
        pass

    def getPositionMeasured(self, callback=None):
        frame = CANFrame(self.device_id, CAN_ID.POSITION_MEASURED_SETPOINT, 0)
        if not callback:
            callback = MotorController.onGetPositionMeasured
        callback_wrap = lambda controller, frame: callback(controller, MotorController.unpack("<ff", frame.data, 0))
        self.transport.transmit(frame, self, callback_wrap)
        
    @staticmethod
    def onGetPositionSetpoint(self, data):
        pass

    def getPositionSetpoint(self, callback=None):
        frame = CANFrame(self.device_id, CAN_ID.POSITION_MEASURED_SETPOINT, 0)
        if not callback:
            callback = MotorController.onGetPositionMeasured
        callback_wrap = lambda controller, frame: callback(controller, MotorController.unpack("<ff", frame.data, 1))
        self.transport.transmit(frame, self, callback_wrap)
    
    @staticmethod
    def onGetPositionTarget(self, data):
        pass

    def setPositionTarget(self, data, callback=None):
        frame = CANFrame(self.device_id, CAN_ID.POSITION_TARGET, 4, struct.pack("<f", data))
        if not callback:
            callback = MotorController.onGetPositionTarget
        callback_wrap = lambda controller, frame: callback(controller, MotorController.unpack("<f", frame.data, 0))
        self.transport.transmit(frame, self, callback_wrap)
    
    @staticmethod
    def onFeed(self):
        pass

    def feed(self, callback=None):
        frame = CANFrame(self.device_id, CAN_ID.SAFETY_WATCHDOG, 0)
        if not callback:
            callback = MotorController.onFeed
        callback_wrap = lambda controller, frame: callback(controller)
        self.transport.transmit(frame, self, callback_wrap)
