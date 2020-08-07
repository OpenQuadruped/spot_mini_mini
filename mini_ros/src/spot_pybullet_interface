#!/usr/bin/env python
"""
DESCRIPTION:

SUBSCRIBERS:
"""

from __future__ import division
import os
import rospy
import numpy as np
from mini_ros.msg import MiniCmd, JoyButtons
import copy

import sys

import rospkg 
rospack = rospkg.RosPack()


sys.path.append(rospack.get_path('mini_ros') + '/../')

sys.path.append('../../')

from spotmicro.GymEnvs.spot_bezier_env import spotBezierEnv
from spotmicro.Kinematics.SpotKinematics import SpotModel
from spotmicro.GaitGenerator.Bezier import BezierGait

# Controller Params
STEPLENGTH_SCALE = 0.06
Z_SCALE_CTRL = 0.12
RPY_SCALE = 0.6
SV_SCALE = 0.1
CHPD_SCALE = 0.0005
YAW_SCALE = 1.5

# AGENT PARAMS
CD_SCALE = 0.05
SLV_SCALE = 0.05
RESIDUALS_SCALE = 0.03
Z_SCALE = 0.05
# Filter actions
alpha = 0.7
# Added this to avoid filtering residuals
# -1 for all


class SpotCommander():
    def __init__(self):

        rospy.init_node('Policies', anonymous=True)
        self.agents = {}
        # self.movetypes = [
        #     "Forward", "Backward", "Left", "Right", "CW", "CCW", "Stop"
        # ]
        self.movetypes = ["Stop"]
        self.mini_cmd = MiniCmd()
        self.jb = JoyButtons()
        self.mini_cmd.x_velocity = 0.0
        self.mini_cmd.y_velocity = 0.0
        self.mini_cmd.rate = 0.0
        self.mini_cmd.roll = 0.0
        self.mini_cmd.pitch = 0.0
        self.mini_cmd.yaw = 0.0
        self.mini_cmd.z = 0.0
        self.mini_cmd.motion = "Stop"
        self.mini_cmd.movement = "Stepping"
        # FIXED
        self.BaseStepVelocity = 0.1
        self.StepVelocity = self.BaseStepVelocity
        # Stock, use Bumpers to change
        self.BaseSwingPeriod = 0.2
        self.SwingPeriod = self.BaseSwingPeriod
        # Stock, use arrow pads to change
        self.BaseClearanceHeight = 0.04
        self.BasePenetrationDepth = 0.005
        self.ClearanceHeight = self.BaseClearanceHeight
        self.PenetrationDepth = self.BasePenetrationDepth

        self.load_spot()
        # mini_cmd_cb from mini_cmd topic
        self.sub_cmd = rospy.Subscriber('mini_cmd', MiniCmd, self.mini_cmd_cb)
        self.sub_jb = rospy.Subscriber('joybuttons', JoyButtons, self.jb_cb)
        self.time = rospy.get_time()
        print("READY TO GO!")

    def load_spot(self):

        self.env = spotBezierEnv(render=True,
                                 on_rack=False,
                                 height_field=False,
                                 draw_foot_path=False)

        self.env.reset()

        seed = 0
        # Set seeds
        self.env.seed(seed)
        np.random.seed(seed)

        state_dim = self.env.observation_space.shape[0]
        print("STATE DIM: {}".format(state_dim))
        action_dim = self.env.action_space.shape[0]
        print("ACTION DIM: {}".format(action_dim))
        max_action = float(self.env.action_space.high[0])
        print("RECORDED MAX ACTION: {}".format(max_action))

        self.state = self.env.reset()

        # Load Spot Model
        self.spot = SpotModel()

        self.dt = self.env._time_step

        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)

        self.bzg = BezierGait(dt=self.env._time_step)

    def mini_cmd_cb(self, mini_cmd):
        """ Reads the desired Minitaur command and passes it for execution

            Args: mini_cmd
        """
        try:
            # Update mini_cmd
            self.mini_cmd = mini_cmd
            # log input data as debug-level message
            rospy.logdebug(mini_cmd)
        except rospy.ROSInterruptException:
            pass

    def jb_cb(self, jb):
        """ Reads the desired additional joystick buttons

            Args: jb
        """
        try:
            # Update jb
            self.jb = jb
            # log input data as debug-level message
            rospy.logdebug(jb)
        except rospy.ROSInterruptException:
            pass

    def move(self):
        """ Turn joystick inputs into commands
        """

        if self.mini_cmd.motion != "Stop":
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = np.clip(
                self.BaseSwingPeriod +
                (-self.mini_cmd.faster + -self.mini_cmd.slower) * SV_SCALE,
                0.1, 0.3)
            if self.mini_cmd.movement == "Stepping":
                StepLength = self.mini_cmd.x_velocity + abs(
                    self.mini_cmd.y_velocity * 0.66)
                StepLength = np.clip(StepLength, -1.0, 1.0)
                StepLength *= STEPLENGTH_SCALE
                LateralFraction = self.mini_cmd.y_velocity * np.pi / 2
                YawRate = self.mini_cmd.rate * YAW_SCALE
                # x offset
                pos = np.array(
                    [0.0, 0.0, self.mini_cmd.z * Z_SCALE_CTRL])
                orn = np.array([0.0, 0.0, 0.0])
            else:
                StepLength = 0.0
                LateralFraction = 0.0
                YawRate = 0.0
                # RESET
                self.ClearanceHeight = self.BaseClearanceHeight
                self.PenetrationDepth = self.BasePenetrationDepth
                self.StepVelocity = self.BaseStepVelocity
                # x offset
                pos = np.array(
                    [0.0, 0.0, self.mini_cmd.z * Z_SCALE_CTRL])
                orn = np.array([
                    self.mini_cmd.roll * RPY_SCALE,
                    self.mini_cmd.pitch * RPY_SCALE,
                    self.mini_cmd.yaw * RPY_SCALE
                ])
        else:
            StepLength = 0.0
            LateralFraction = 0.0
            YawRate = 0.0
            # RESET
            self.ClearanceHeight = self.BaseClearanceHeight
            self.PenetrationDepth = self.BasePenetrationDepth
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = self.BaseSwingPeriod
            pos = np.array([0.0, 0.0, 0.0])
            orn = np.array([0.0, 0.0, 0.0])

        # TODO: integrate into controller
        self.ClearanceHeight += self.jb.updown * CHPD_SCALE
        self.PenetrationDepth += self.jb.leftright * CHPD_SCALE

        # Manual Reset
        if self.jb.left_bump or self.jb.right_bump:
            self.ClearanceHeight = self.BaseClearanceHeight
            self.PenetrationDepth = self.BasePenetrationDepth
            self.StepVelocity = self.BaseStepVelocity
            self.SwingPeriod = self.BaseSwingPeriod
            self.env.reset()


        # print("SL: {} \tSV: {} \nLAT: {} \tYAW: {}".format(
        #     StepLength, self.StepVelocity, LateralFraction, YawRate))
        # print("BASE VEL: {}".format(self.BaseStepVelocity))
        # print("---------------------------------------")

        contacts = self.state[-4:]

        # Time
        dt = rospy.get_time() - self.time
        # print("dt: {}".format(dt))
        self.time = rospy.get_time()

        # Update Step Period
        self.bzg.Tswing = self.SwingPeriod

        self.T_bf = self.bzg.GenerateTrajectory(StepLength, LateralFraction,
                                                YawRate, self.StepVelocity,
                                                self.T_bf0, self.T_bf,
                                                self.ClearanceHeight,
                                                self.PenetrationDepth,
                                                contacts, dt)

        joint_angles = self.spot.IK(orn, pos, self.T_bf)
        self.env.pass_joint_angles(joint_angles.reshape(-1))
        # Get External Observations
        # TODO
        # self.env.spot.GetExternalObservations(bzg, bz_step)
        # Step
        action = self.env.action_space.sample()
        action[:] = 0.0
        self.state, reward, done, _ = self.env.step(action)


def main():
    """ The main() function. """
    mini_commander = SpotCommander()
    rate = rospy.Rate(600.0)
    while not rospy.is_shutdown():
        # This is called continuously. Has timeout functionality too
        mini_commander.move()
        rate.sleep()
        # rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass