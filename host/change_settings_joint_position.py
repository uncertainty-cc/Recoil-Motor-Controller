import time

import recoil



DEVICE_ID = 8

transport = recoil.SPICANTransport(port="can0", baudrate=500000)

controller = recoil.MotorController(transport, device_id=DEVICE_ID)

transport.start()

print("================ Device {id} ================".format(id=controller.device_id))
print("torque limit: {0}".format(controller.getTorqueLimit()))
pos_limit_lo, pos_limit_hi = controller.getPositionLimit()
print("position limit: [{0}, {1}]".format(pos_limit_lo, pos_limit_hi))
print("CPR: {0}".format(controller.getCPR()))
print("position offset: {0}".format(controller.getPositionOffset()))

print("=============================================")

quit()

# controller.setPositionOffset(0)

try:
    while True:
        print(controller.getPositionMeasured())
        time.sleep(0.1)
except KeyboardInterrupt:
    pass

# controller.setTorqueVelocityLimit(0.1, 20)
controller.setPositionLimit(-30, 2)
# controller.setPositionOffset(-controller.getPositionMeasured())



# try:
#     while True:
#         print(controller.getPositionMeasured())
#         time.sleep(0.1)
# except KeyboardInterrupt:
#     pass

# print("saving settings to Flash")
# controller.storeSettingToFlash()

transport.stop()
