import time

import recoil



DEVICE_ID = 2

transport = recoil.SPICANTransport(port="can0", baudrate=1000000)

controller = recoil.MotorController(transport, device_id=DEVICE_ID)

transport.start()

try:
    print("Change setting on device #{id}".format(id=DEVICE_ID))

    print("torque limit:", controller.getTorqueLimit())
    time.sleep(1)

    torque_limit = 0.8

    print("setting torque limit to {torque_limit}".format(torque_limit=torque_limit))

    controller.setTorqueVelocityLimit(torque_limit, 20)

    time.sleep(1)

    print("saving config...")
    controller.storeSettingToFlash()

    time.sleep(1)

    print("new torque limit:", controller.getTorqueLimit())
    print("new torque limit:", controller.getTorqueLimit())

except KeyboardInterrupt:
    controller.setMode(recoil.Mode.IDLE)
    pass

transport.stop()
