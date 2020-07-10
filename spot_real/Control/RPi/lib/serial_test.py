import time
from Teensy_Interface import TeensyInterface


ti = TeensyInterface()
while True:
    s = input("Send? [y/n]")
    if s == "y":
        ti.add_to_buffer(4, 0, 135, 60)
        ti.send_buffer()
        time.sleep(1.0)

    r = input("Read? [y/n]")
    if r == "y":
        print(ti.read_buffer())
        time.sleep(1.0)
