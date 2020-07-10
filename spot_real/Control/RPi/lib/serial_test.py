from Teensy_Interface import TeensyInterface


ti = TeensyInterface()
while True:
    ti.add_to_buffer(4, 0, 135, 60)
    ti.send_buffer()
    print(ti.read_buffer())
