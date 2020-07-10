import time
from Teensy_Interface import TeensyInterface


ti = TeensyInterface()
while True:
    ti.add_to_buffer(4, 0, 135, 60)
    ti.send_buffer()
    time.sleep(1.0)
    print(ti.read_buffer())
    time.sleep(1.0)
