import os
import struct
import threading

import can

class CANFrame:
    CAN_ID_STANDARD = 0
    CAN_ID_EXTENDED = 1
    
    CAN_FRAME_REMOTE = 0
    CAN_FRAME_DATA = 1
    
    def __init__(self, 
            device_id=0, 
            func_id=0, 
            size=0, 
            data=b"",
            id_type=CAN_ID_STANDARD, 
            frame_type=CAN_FRAME_REMOTE
        ):
        self.device_id = device_id
        self.func_id = func_id
        self.id_type = id_type
        self.frame_type = frame_type
        self.size = size
        self.data = data
    
    def isRemoteFrame(self):
        return self.frame_type == CANFrame.CAN_FRAME_REMOTE

class SocketCANTransport:
    def __init__(self, port="can0", baudrate=1000000):
        self.port = port
        self.baudrate = baudrate
        self.interface = None
        self.handlers = []
        self.killed = False

    def disable(self):
        self.killed = True
        os.system("sudo ifconfig {port} down".format(port=self.port))

    def enable(self):
        os.system("sudo ip link set {port} type can bitrate {baudrate}".format(port=self.port, baudrate=self.baudrate))
        os.system("sudo ifconfig {port} up".format(port=self.port))
        self.interface = can.interface.Bus(channel=self.port, bustype="socketcan")
        self.killed = False
        self.rx_handler_thread = threading.Thread(target=self.handleRX)
        self.rx_handler_thread.start()

    def transmit(self, frame, controller=None, callback=None):
        can_id = (frame.func_id << 4) | frame.device_id

        msg = can.Message(
            arbitration_id=can_id,
            is_extended_id=False,
            is_remote_frame=frame.isRemoteFrame(),
            data=frame.data)

        if callback:
            self.handlers.append((frame, controller, callback))

        self.interface.send(msg)
    
    def receive(self):
        try:
            msg = self.interface.recv(timeout=None) # blocking
        except can.exceptions.CanOperationError:
            return None
        frame = CANFrame(
            device_id = msg.arbitration_id & 0x0F,
            func_id = msg.arbitration_id >> 4,
            size = msg.dlc,
            data = msg.data,
            frame_type = CANFrame.CAN_FRAME_DATA
        )
        return frame
    
    def handleRX(self):
        while not self.killed:
            rx_frame = self.receive()
            if not rx_frame:
                continue
            for handler in self.handlers:
                tx_frame, controller, callback = handler
                if (rx_frame.device_id == tx_frame.device_id) and (rx_frame.func_id == tx_frame.func_id):
                    callback(controller, rx_frame)
                    self.handlers.remove(handler)



class MotorController:
    CAN_ID_ESTOP              = 0x00
    CAN_ID_ID                 = 0x01
    CAN_ID_VERSION            = 0x02
    CAN_ID_SAFETY             = 0x03
    CAN_ID_FLASH              = 0x04

    CAN_ID_MODE                                           = 0x06
    CAN_ID_STATUS                                         = 0x07


    CAN_ID_ENCODER_CPR                                    = 0x10
    CAN_ID_ENCODER_FILTER_BANDWIDTH                       = 0x11
    CAN_ID_ENCODER_POSITION_OFFSET                        = 0x12
    CAN_ID_ENCODER_N_ROTATIONS                            = 0x13
    CAN_ID_ENCODER_POSITION_RELATIVE                      = 0x14
    CAN_ID_ENCODER_POSITION                               = 0x15
    CAN_ID_ENCODER_VELOCITY                               = 0x16

    CAN_ID_POWERSTAGE_VOLTAGE_THREASHOLD                  = 0x20
    CAN_ID_POWERSTAGE_ADC_READING_RAW_A_B_C               = 0x21
    CAN_ID_POWERSTAGE_ADC_READING_OFFSET_A_B_C            = 0x22
    CAN_ID_POWERSTAGE_BUS_VOLTAGE                         = 0x23

    CAN_ID_MOTOR_POLE_PAIRS                               = 0x30
    CAN_ID_MOTOR_KV_RATING                                = 0x31
    CAN_ID_MOTOR_FLUX_ANGLE_OFFSET                        = 0x32

    CAN_ID_CURRENT_CONTROLLER_CURRENT_FILTER_ALPHA        = 0x40
    CAN_ID_CURRENT_CONTROLLER_I_Q_KP_KI                   = 0x41
    CAN_ID_CURRENT_CONTROLLER_I_D_KP_KI                   = 0x42
    CAN_ID_CURRENT_CONTROLLER_I_A_I_B_MEASURED            = 0x43
    CAN_ID_CURRENT_CONTROLLER_I_C_MEASURED                = 0x44
    CAN_ID_CURRENT_CONTROLLER_V_A_V_B_SETPOINT            = 0x45
    CAN_ID_CURRENT_CONTROLLER_V_C_SETPOINT                = 0x46
    CAN_ID_CURRENT_CONTROLLER_I_ALPHA_I_BETA_MEASURED     = 0x47
    CAN_ID_CURRENT_CONTROLLER_V_ALPHA_V_BETA_SETPOINT     = 0x48
    CAN_ID_CURRENT_CONTROLLER_V_Q_V_D_TARGET              = 0x49
    CAN_ID_CURRENT_CONTROLLER_V_Q_V_D_SETPOINT            = 0x4A
    CAN_ID_CURRENT_CONTROLLER_I_Q_I_D_MEASURED            = 0x4B
    CAN_ID_CURRENT_CONTROLLER_I_Q_I_D_TARGET              = 0x4C
    CAN_ID_CURRENT_CONTROLLER_I_Q_I_D_SETPOINT            = 0x4D
    CAN_ID_CURRENT_CONTROLLER_I_Q_I_D_INTEGRATOR          = 0x4E

    CAN_ID_POSITION_CONTROLLER_POSITION_KP_KI             = 0x50
    CAN_ID_POSITION_CONTROLLER_VELOCITY_KP_KI             = 0x51
    CAN_ID_POSITION_CONTROLLER_TORQUE_VELOCITY_LIMIT      = 0x52
    CAN_ID_POSITION_CONTROLLER_POSITION_LIMIT             = 0x54
    CAN_ID_POSITION_CONTROLLER_TORQUE_TARGET_MEASURED     = 0x55
    CAN_ID_POSITION_CONTROLLER_TORQUE_SETPOINT            = 0x56
    CAN_ID_POSITION_CONTROLLER_VELOCITY_TARGET_MEASURED   = 0x57
    CAN_ID_POSITION_CONTROLLER_VELOCITY_SETPOINT          = 0x58
    CAN_ID_POSITION_CONTROLLER_POSITION_TARGET_MEASURED   = 0x59
    CAN_ID_POSITION_CONTROLLER_POSITION_SETPOINT          = 0x5A
    

    
    MODE_DISABLED           = 0x00
    MODE_IDLE               = 0x01

    MODE_CALIBRATION        = 0x05

    MODE_TORQUE             = 0x10
    MODE_VELOCITY           = 0x11
    MODE_POSITION           = 0x12

    MODE_OPEN_IDQ           = 0x21
    MODE_OPEN_VDQ           = 0x22
    MODE_OPEN_VALPHABETA    = 0x23
    MODE_OPEN_VABC          = 0x24

    MODE_DEBUG              = 0x80


    def __init__(self, transport, device_id=1):
        self.transport = transport
        self.device_id = device_id
        
        self.mode = self.MODE_DISABLED
    
    def getMode(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_MODE, 0)
        callback_wrap = lambda controller, frame: callback(controller, struct.unpack("<B", frame.data)[0])
        self.transport.transmit(frame, self, callback_wrap)
    
    def setMode(self, mode):
        frame = CANFrame(self.device_id, self.CAN_ID_MODE, 1, struct.pack("<B", mode), frame_type=CANFrame.CAN_FRAME_DATA)
        self.transport.transmit(frame)
    
    def getPositionTarget(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_CONTROLLER_POSITION_TARGET_MEASURED, 0)
        callback_wrap = lambda controller, frame: callback(controller, struct.unpack("<ff", frame.data)[0])
        self.transport.transmit(frame, self, callback_wrap)
    
    def getPositionMeasured(self, callback=None):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_CONTROLLER_POSITION_TARGET_MEASURED, 0)
        callback_wrap = lambda controller, frame: callback(controller, struct.unpack("<ff", frame.data)[1])
        self.transport.transmit(frame, self, callback_wrap)
    
    def setPositionTarget(self, position_target):
        frame = CANFrame(self.device_id, self.CAN_ID_POSITION_CONTROLLER_POSITION_TARGET_MEASURED, 4, struct.pack("<f", position_target), frame_type=CANFrame.CAN_FRAME_DATA)
        self.transport.transmit(frame)
    
    def feed(self):
        frame = CANFrame(self.device_id, self.CAN_ID_HEARTBEAT, 1, b"0", frame_type=CANFrame.CAN_FRAME_DATA)
        self.transport.transmit(frame)
