import can
import can.interfaces.serial
import serial
import time

def send_one():
    """Sends a single message."""

    counter = 0
    
    #bus = can.Bus(interface="serial", channel="COM9", baudrate=1000000)
    bus = can.Bus(interface="serial", channel="COM34", baudrate=1000000)

    

    while 1:
        #        try:
        #            msg = bus.recv(timeout=0.01)
        #        except:
        #            pass
        #
        #        if msg:
        #            print(msg)
        #print(counter)
        msg = can.Message(
            arbitration_id=0x02 | (0x7F << 4),
            data=[],
            is_extended_id=False
        )
        counter += 1

        try:
            bus.send(msg)
            print(f"Message sent on {bus.channel_info}")
        except can.CanError:
            print("Message NOT sent")
        except serial.serialutil.SerialException:
            print("Message NOT sent, reconnect")
            while 1:
                try:
                    bus = can.Bus(interface="serial", channel="COM9", baudrate=115200)
                except serial.serialutil.SerialException:
                    continue
                break
        time.sleep(0.1)

if __name__ == "__main__":
    send_one()
