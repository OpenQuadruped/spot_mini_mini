#!/usr/bin/env python3
"""
DESCRIPTION:

SUBSCRIBERS:
"""

from __future__ import division
import rospy
from mini_ros.msg import IMUdata, ContactData
from std_msgs.msg import String
# Used for str(Boolean) --> Boolean
from distutils.util import strtobool

import sys

import rospkg
rospack = rospkg.RosPack()

sys.path.append(rospack.get_path('mini_ros') + '/../')

sys.path.append('../../')

from spot_real.Control.RPi.lib.Teensy_Interface import TeensyInterface
from spot_real.Control.RPi.lib.imu import IMU


class SensorInterface():
    def __init__(self):

        rospy.init_node('SensorInterface', anonymous=True)
        self.TI = TeensyInterface()
        self.imu_pub = rospy.Publisher('spot/imu', IMUdata, queue_size=1)
        self.cnt_pub = rospy.Publisher('spot/contact',
                                       ContactData,
                                       queue_size=1)
        self.str_pub = rospy.Publisher('spot/teensydebug',
                                       String,
                                       queue_size=1)
        print("PUT SPOT ON THE GROUND, CALIBRATING IMU")
        self.imu = IMU()

    def read_sensors(self):
        """ Reads IMU and Contact Sensor data from Teensy 4.0
            and publishes to respective topics
        """
        while not rospy.is_shutdown():
            data = self.TI.read_buffer()

            rospy.logdebug(data)

            imu_dat = IMUdata()
            imu_read = False
            cnt_dat = ContactData()
            cnt_read = False

            msg = data.decode().split(",")
            # REMOVE NEWLINE
            msg[-1] = msg[-1].rstrip('\r\n')

            # IMU
            if msg[0] == "IMUDATA":
                try:
                    imu_msg = [float(x) for x in msg[1:]]
                    # rospy.loginfo(imu_msg)
                    # IMU
                    imu_dat.roll = imu_msg[0]
                    imu_dat.pitch = imu_msg[1]
                    imu_dat.acc_x = imu_msg[2]
                    imu_dat.acc_y = imu_msg[3]
                    imu_dat.acc_z = imu_msg[4]
                    imu_dat.gyro_x = imu_msg[5]
                    imu_dat.gyro_y = imu_msg[6]
                    imu_dat.gyro_z = imu_msg[7]
                    imu_read = True
                except:
                    rospy.logdebug("bad imu read")

            # CONTACT
            elif msg[0] == "CONTACT":
                try:
                    cnt_msg = [strtobool(x) for x in msg[1:]]
                    # rospy.loginfo(cnt_msg)
                    # Contact Sensor
                    cnt_dat.FL = cnt_msg[0]
                    cnt_dat.FR = cnt_msg[1]
                    cnt_dat.BL = cnt_msg[2]
                    cnt_dat.BR = cnt_msg[3]
                    cnt_read = True
                except:
                    rospy.logdebug("bad contact read")

            # READ IMU
            self.imu.filter_rpy()
            imu_dat.roll = self.imu.true_roll
            imu_dat.pitch = self.imu.true_pitch
            imu_dat.acc_x = self.imu.imu_data[3]
            imu_dat.acc_y = self.imu.imu_data[4]
            imu_dat.acc_z = self.imu.imu_data[5]
            imu_dat.gyro_x = self.imu.imu_data[0]
            imu_dat.gyro_y = self.imu.imu_data[1]
            imu_dat.gyro_z = self.imu.imu_data[2]
            imu_read = True

            if imu_read:
                self.imu_pub.publish(imu_dat)
                # rospy.logdebug("IMU")
            if cnt_read:
                self.cnt_pub.publish(cnt_dat)
                # rospy.logdebug("CONTACT")

            string_msg = String()
            string_msg.data = data
            self.str_pub.publish(string_msg)


def main():
    """ The main() function. """
    si = SensorInterface()
    rospy.loginfo("Ready To Log Data!")
    # rate = rospy.Rate(1.0 / 60.0)
    while not rospy.is_shutdown():
        si.read_sensors()
        # rate.sleep()
        rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass