#!/usr/bin/env python
"""
DESCRIPTION:

SUBSCRIBERS:
"""

from __future__ import division
import rospy
from mini_ros.srv import CalibServo, CalibServoResponse
from mini_ros.msg import JointPulse
import numpy as np

import sys

import rospkg
rospack = rospkg.RosPack()

sys.path.append(rospack.get_path('mini_ros') + '/../')

sys.path.append('../../')


class ServoCalibrator():
    def __init__(self):

        rospy.init_node('ServoCalibrator', anonymous=True)

        self.serv = rospy.Service('servo_calibrator', CalibServo,
                                  self.calib_service_cb)
        self.jp_pub = rospy.Publisher('spot/pulse', JointPulse, queue_size=1)

    def calib_service_cb(self, req):
        """ Requests a servo to be moved to a certain position

            Args: req
            Returns: response
        """
        try:
            jp_msg = JointPulse()

            jp_msg.servo_num = req.servo_num
            jp_msg.servo_pulse = req.servo_pulse

            self.jp_pub.publish(jp_msg)
            response = "Servo Command Sent."
        except rospy.ROSInterruptException:
            response = "FAILED to send Servo Command"
        return CalibServoResponse(response)


def main():
    """ The main() function. """
    srv_calib = ServoCalibrator()
    rospy.loginfo(
        "Use The servo_calibrator service (Pulse Width Unit is us (nominal ~500-2500))."
    )
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass