import time

import rath_recoil as recoil

DEVICE_ID = 11

transport = recoil.SocketCANTransport()
controller = recoil.MotorController(transport=transport, device_id=DEVICE_ID)

transport.enable()

received = False

def pingHandler(controller, data):
    global received

    print(controller.device_id, ":", data)
    received = True

try:
    while True:
        print("ping!")
        controller.ping(pingHandler)
        while not received:
            time.sleep(0.1)
        time.sleep(0.1)

except KeyboardInterrupt:
    transport.disable()
