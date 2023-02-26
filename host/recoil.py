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


class SPICANTransport:
    def __init__(self, port="can0", baudrate=1000000):
        self.port = port
        self.baudrate = baudrate
        self._interface = None
        self._handlers = []
        self._killed = threading.Event()

    def stop(self):
        self._killed.set()
        os.system("sudo ifconfig {port} down".format(port=self.port))

    def start(self):
        os.system("sudo ip link set {port} type can bitrate {baudrate}".format(port=self.port, baudrate=self.baudrate))
        os.system("sudo ifconfig {port} up".format(port=self.port))

        self._killed.clear()
        self.connect()

        print("started")

    def connect(self):
        while not self._interface:
            try:
                self._interface = can.Bus(interface="socketcan", channel=self.port, bustype="socketcan", baudrate=self.baudrate)
            except serial.serialutil.SerialException as e:
                print(e)
        print("connected")

    def transmit(self, controller, frame, callback=None):
        can_id = (frame.func_id << 4) | frame.device_id

        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=False,
            data=frame.data)
        self._interface.send(msg)
            
    def receive(self, controller, timeout=0.1):
        try:
            msg = self._interface.recv(timeout=timeout) # blocking
        except can.exceptions.CanOperationError as e:
            print(e)
            return None
        while msg and msg.is_error_frame:
            try:
                msg = self._interface.recv(timeout=timeout) # blocking
            except can.exceptions.CanOperationError as e:
                print(e)
                return None
        # print(msg)
        if not msg:
            return None
        frame = CANFrame(
            device_id = msg.arbitration_id & 0x0F,
            func_id = msg.arbitration_id >> 4,
            size = msg.dlc,
            data = msg.data
        )
        return frame


class SerialCANTransport:
    def __init__(self, port="COM1", baudrate=1000000):
        self.port = port
        self.baudrate = baudrate
        self._interface = None
        self._handlers = []
        self._killed = threading.Event()

    def stop(self):
        self._killed.set()

    def start(self):
        self._killed.clear()
        self.connect()

        print("started")

    def connect(self):
        while not self._interface:
            try:
                self._interface = can.Bus(interface="serial", channel=self.port, baudrate=self.baudrate)
            except serial.serialutil.SerialException as e:
                print(e)
        print("connected")

    def transmit(self, controller, frame):
        can_id = (frame.func_id << 4) | frame.device_id

        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=False,
            data=frame.data)
        self._interface.send(msg)
        
    def receive(self, controller, timeout=0.1):
        try:
            msg = self._interface.recv(timeout=timeout) # blocking
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
            print("warning:", e, data)
            return 0

    def ping(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.PING, size=0)
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<B", rx_frame.data, 0)
        if callback:
            callback(self, rx_data)
        return rx_data

    def feed(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.SAFETY_WATCHDOG, size=0)
        self.transport.transmit(self, tx_frame)

    def getMode(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.MODE_ERROR, size=0)
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<HH", rx_frame.data, 0)
        if callback:
            callback(self, rx_data)
        return rx_data

    def setMode(self, mode, callback=None, clear_error=False):
        tx_frame = CANFrame(self.device_id, CAN_ID.MODE_ERROR, size=4, 
                            data=struct.pack("<HH", mode, 1 if clear_error else 0))
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<HH", rx_frame.data, 0)
        if callback:
            callback(self, rx_data)
        return rx_data

    def loadSettingFromFlash(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.FLASH, size=1, 
                            data=struct.pack("<B", 0))
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<B", rx_frame.data, 0)
        if callback:
            callback(rx_data)
        return rx_data
        
    def storeSettingToFlash(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.FLASH, size=1, 
                            data=struct.pack("<B", 1))
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<B", rx_frame.data, 0)
        if callback:
            callback(rx_data)
        return rx_data

    def getPositionMeasured(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.POSITION_MEASURED_SETPOINT, size=0)
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<ff", rx_frame.data, 0)
        if callback:
            callback(rx_data)
        return rx_data

    def getPositionSetpoint(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.POSITION_MEASURED_SETPOINT, size=0)
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<ff", rx_frame.data, 1)
        if callback:
            callback(rx_data)
        return rx_data

    def setPositionTarget(self, position_target, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.POSITION_TARGET, size=4, 
                            data=struct.pack("<f", position_target))
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<f", rx_frame.data, 0)
        if callback:
            callback(rx_data)
        return rx_data

    def getPositionLimit(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.POSITION_LIMIT, size=0)
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0, 0
        lower_limit = MotorController.unpack("<ff", rx_frame.data, 0)
        upper_limit = MotorController.unpack("<ff", rx_frame.data, 1)
        if callback:
            callback(lower_limit, upper_limit)
        return lower_limit, upper_limit


    def setPositionOffset(self, offset, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.ENCODER_CPR_OFFSET, size=8, 
                            data=struct.pack("<if", 4096, offset))
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<if", rx_frame.data, 1)
        if callback:
            callback(rx_data)
        return rx_data

    def getCPR(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.ENCODER_CPR_OFFSET, size=0)
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<if", rx_frame.data, 0)
        if callback:
            callback(rx_data)
        return rx_data

    def getPositionOffset(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.ENCODER_CPR_OFFSET, size=0)
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<if", rx_frame.data, 1)
        if callback:
            callback(rx_data)
        return rx_data

    def setPositionLimit(self, lower_limit, upper_limit, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.POSITION_LIMIT, size=8, 
                            data=struct.pack("<ff", lower_limit, upper_limit))
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        lower_limit = MotorController.unpack("<ff", rx_frame.data, 0)
        upper_limit = MotorController.unpack("<ff", rx_frame.data, 1)
        if callback:
            callback(lower_limit, upper_limit)
        return lower_limit, upper_limit

    def getTorqueLimit(self, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.TORQUE_VELOCITY_LIMIT, size=0)
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<ff", rx_frame.data, 0)
        if callback:
            callback(rx_data)
        return rx_data

    def setTorqueVelocityLimit(self, torque_limit, velocity_limit, callback=None):
        tx_frame = CANFrame(self.device_id, CAN_ID.TORQUE_VELOCITY_LIMIT, size=8,
                            data=struct.pack("<ff", torque_limit, velocity_limit))
        self.transport.transmit(self, tx_frame)
        rx_frame = self.transport.receive(self)
        if not rx_frame:
            return 0
        rx_data = MotorController.unpack("<ff", rx_frame.data, 0)
        if callback:
            callback(rx_data)
        return rx_data
        