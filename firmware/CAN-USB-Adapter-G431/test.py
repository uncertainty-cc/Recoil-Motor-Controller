import can
import can.interfaces.serial
import serial


def send_one():
    """Sends a single message."""

    bus = can.Bus(interface="serial", channel="COM9", baudrate=115200)


    counter = 0
    while 1:
        msg = bus.recv(timeout=0)
        print(msg)
        print(counter)
        msg = can.Message(
            arbitration_id=0x213,
            data=[counter%255],
            is_extended_id=False
        )
        counter += 1
        try:
            bus.send(msg)
            #print(f"Message sent on {bus.channel_info}")
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


if __name__ == "__main__":
    send_one()
