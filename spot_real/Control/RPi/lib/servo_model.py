#!/usr/bin/env python

import numpy as np
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

import time
from adafruit_servokit import ServoKit


class ServoJoint:
    def __init__(self,
                 name,
                 effort=0.15,
                 speed=8.76,
                 gpio=22,
                 fb_chan=0,
                 pwm_chan=0,
                 pwm_min=600,
                 pwm_max=2800,
                 servo_horn_bias=0,
                 actuation_range=270):
        self.name = name
        self.effort = effort  # Nm
        self.speed = speed  # rad/s

        # offset in mechanical design
        self.servo_horn_bias = servo_horn_bias

        # create the spi bus
        self.spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

        # create the cs (chip select)
        if gpio == 22:
            self.cs = digitalio.DigitalInOut(board.D22)
        elif gpio == 27:
            self.cs = digitalio.DigitalInOut(board.D27)

        # create the mcp object
        self.mcp = MCP.MCP3008(self.spi, self.cs)

        # fb_chan from 0 to 7 for each MCP ADC
        if fb_chan == 0:
            self.chan = AnalogIn(self.mcp, MCP.P0)
        elif fb_chan == 1:
            self.chan = AnalogIn(self.mcp, MCP.P1)
        elif fb_chan == 2:
            self.chan = AnalogIn(self.mcp, MCP.P2)
        elif fb_chan == 3:
            self.chan = AnalogIn(self.mcp, MCP.P3)
        elif fb_chan == 4:
            self.chan = AnalogIn(self.mcp, MCP.P4)
        elif fb_chan == 5:
            self.chan = AnalogIn(self.mcp, MCP.P5)
        elif fb_chan == 6:
            self.chan = AnalogIn(self.mcp, MCP.P6)
        elif fb_chan == 7:
            self.chan = AnalogIn(self.mcp, MCP.P7)

        self.kit = ServoKit(channels=16)
        self.pwm_chan = pwm_chan
        self.kit.servo[self.pwm_chan].set_pulse_width_range(pwm_min, pwm_max)
        self.kit.servo[self.pwm_chan].actuation_range = actuation_range

        self.bias = self.rad2deg(self.servo_horn_bias)  # degrees

    def forward_propagate(self, current_pos, desired_pos, dt):
        """ Predict the new position of the actuated servo
            motor joint
        """
        pos_change = desired_pos - current_pos
        percent_of_pos_reached = (self.speed * dt) / np.abs(pos_change)
        # Cap at 100%
        if percent_of_pos_reached > 100.0:
            percent_of_pos_reached = 100.0
        return current_pos + (pos_change * percent_of_pos_reached)

    def calibrate(self, min_val, max_val, num_iters=22):
        # Send to min value and record digital sig
        # Send to max value and record digital sig

        # OR INCREMENT TO GET MORE DATA
        commands = np.array([])
        measurements = np.array([])

        # Number of data points to collect
        num_iters = num_iters

        for i in range(num_iters):
            # commanded_value = (-np.pi / 2.0) + (i *
            # (np.pi) / float(num_iters - 1))
            range_val = max_val - min_val
            commanded_value = (min_val) + (i *
                                           (range_val) / float(num_iters - 1))
            commands = np.append(commands, commanded_value)
            self.actuate(commanded_value)
            time.sleep(.5)  # according to rated speed 0.1sec/60deg
            measurements = np.append(measurements, self.chan.value)

        time.sleep(1.0)  # according to rated speed 0.1sec/60deg
        # Perform fit
        print("COMMANDS: {}".format(commands))
        print("MEASUREMENTS: {}".format(measurements))
        polynomial = 4
        # We want to input measurements and receive equivalent commanded angles in radians
        self.fit = np.poly1d(np.polyfit(measurements, commands, polynomial))
        # Test Fit
        print("TESTING FIT: 90 DEG; RESULT IS {}".format(
            self.fit(self.chan.value)))

        print("RETURNING TO -90")

        self.actuate(-np.pi / 2)

        # Save fit
        np.save(self.name + "_fit", self.fit)

    def load_calibration(self):
        # Load fit
        self.fit = np.load(self.name + "_fit.npy")

    def remap(self, value):
        # Use calibraton value to remap from Digital Sig to Angle
        p = np.poly1d(self.fit)

        return p(value)

    def measure(self):
        return self.remap(self.chan.value)

    def rad2deg(self, rad):
        deg = rad * 180.0 / np.pi
        if deg > 90:
            deg = 90
        elif deg < -90:
            deg = -90
        return deg

    def deg2rad(self, deg):
        return deg * np.pi / 180.0

    def actuate(self, desired_pos):
        self.kit.servo[
            self.pwm_chan].angle = self.bias + self.rad2deg(desired_pos)

    def actuate_deg(self, desired_pos):
        self.kit.servo[
            self.pwm_chan].angle = desired_pos
