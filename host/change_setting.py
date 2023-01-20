import time

import rath_recoil as recoil


DEVICE_ID = 11
transport = recoil.SocketCANTransport()
controller = recoil.MotorController(transport=transport, device_id=DEVICE_ID)

transport.enable()

def responseHandler(controller, data):
    print(data)

print("torque limit:")
controller.getTorqueLimit(responseHandler)
time.sleep(1)

# print("velocity limit:")
# controller.getVelocityLimit(responseHandler)
# time.sleep(1)

# print("setting new limit")
# controller.setTorqueVelocityLimit(0.06, 20)
# time.sleep(1)

# controller.setTorqueVelocityLimit(0.06, 20)

transport.disable()
time.sleep(10)
