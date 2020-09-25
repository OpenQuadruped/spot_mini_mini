#!/usr/bin/env python
"""
DESCRIPTION:

SUBSCRIBERS:
"""

from __future__ import division
import rospy
import numpy as np
from mini_ros.msg import MiniCmd, JoyButtons, IMUdata, ContactData, AgentData, JointAngles
import copy
import sys
import os
import rospkg
rospack = rospkg.RosPack()

sys.path.append(rospack.get_path('mini_ros') + '/../')

sys.path.append('../../')

from spotmicro.Kinematics.SpotKinematics import SpotModel
from spotmicro.GaitGenerator.Bezier import BezierGait
from spot_bullet.src.ars_lib.ars import ARSAgent, Normalizer, Policy
from spotmicro.GymEnvs.spot_bezier_env import spotBezierEnv

# Initialize Node
rospy.init_node('Policies', anonymous=True)

# Controller Params
STEPLENGTH_SCALE = rospy.get_param("STEPLENGTH_SCALE")
Z_SCALE_CTRL = rospy.get_param("Z_SCALE_CTRL")
RPY_SCALE = rospy.get_param("RPY_SCALE")
SV_SCALE = rospy.get_param("SV_SCALE")
CHPD_SCALE = rospy.get_param("CHPD_SCALE")
YAW_SCALE = rospy.get_param("YAW_SCALE")

# AGENT PARAMS
CD_SCALE = rospy.get_param("CD_SCALE")
SLV_SCALE = rospy.get_param("SLV_SCALE")
RESIDUALS_SCALE = rospy.get_param("RESIDUALS_SCALE")
Z_SCALE = rospy.get_param("Z_SCALE")
# Filter actions
alpha = rospy.get_param("alpha")
# Added this to avoid filtering residuals
# -1 for all
actions_to_filter = rospy.get_param("actions_to_filter")


class SpotCommander():
    def __init__(self, Agent=True, contacts=False):

        self.Agent = Agent
        self.agent_num = rospy.get_param("agent_num")
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
        self.BaseStepVelocity = rospy.get_param("BaseStepVelocity")
        self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
        # Stock, use Bumpers to change
        self.BaseSwingPeriod = rospy.get_param("Tswing")
        self.SwingPeriod = copy.deepcopy(self.BaseSwingPeriod)
        self.SwingPeriod_LIMITS = rospy.get_param("SwingPeriod_LIMITS")
        # Stock, use arrow pads to change
        self.BaseClearanceHeight = rospy.get_param("BaseClearanceHeight")
        self.BasePenetrationDepth = rospy.get_param("BasePenetrationDepth")
        self.ClearanceHeight = copy.deepcopy(self.BaseClearanceHeight)
        self.PenetrationDepth = copy.deepcopy(self.BasePenetrationDepth)
        self.ClearanceHeight_LIMITS = rospy.get_param("ClearanceHeight_LIMITS")
        self.PenetrationDepth_LIMITS = rospy.get_param(
            "PenetrationDepth_LIMITS")

        # Time
        self.time = rospy.get_time()

        self.enable_contact = contacts

        # Contacts: FL, FR, BL, BR
        self.contacts = [0, 0, 0, 0]

        # IMU: R, P, Ax, Ay, Az, Gx, Gy, Gz
        self.imu = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # See spot_params.yaml in config
        self.spot = SpotModel(
            shoulder_length=rospy.get_param("shoulder_length"),
            elbow_length=rospy.get_param("elbow_length"),
            wrist_length=rospy.get_param("wrist_length"),
            hip_x=rospy.get_param("hip_x"),
            hip_y=rospy.get_param("hip_y"),
            foot_x=rospy.get_param("foot_x"),
            foot_y=rospy.get_param("foot_y"),
            height=rospy.get_param("height"),
            com_offset=rospy.get_param("com_offset"))

        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)

        # See spot_params.yaml in config
        self.bzg = BezierGait(dt=rospy.get_param("dt"),
                              Tswing=rospy.get_param("Tswing"))

        if self.Agent:
            self.load_spot(contacts, agent_num=self.agent_num)
        # cmd_cb from mini_cmd topic
        self.sub_cmd = rospy.Subscriber('mini_cmd',
                                        MiniCmd,
                                        self.cmd_cb,
                                        queue_size=1)
        self.sub_jb = rospy.Subscriber('joybuttons',
                                       JoyButtons,
                                       self.jb_cb,
                                       queue_size=1)
        self.sub_imu = rospy.Subscriber('spot/imu',
                                        IMUdata,
                                        self.imu_cb,
                                        queue_size=1)
        self.sub_cnt = rospy.Subscriber('spot/contact',
                                        ContactData,
                                        self.cnt_cb,
                                        queue_size=1)
        self.ag_pub = rospy.Publisher('spot/agent', AgentData, queue_size=1)
        self.ja_pub = rospy.Publisher('spot/joints', JointAngles, queue_size=1)
        print("READY TO GO!")

    def load_spot(self, contacts, state_dim=12, action_dim=14, agent_num=0):
        self.policy = Policy(state_dim=state_dim, action_dim=action_dim)
        self.normalizer = Normalizer(state_dim=state_dim)
        env = spotBezierEnv(render=False,
                            on_rack=False,
                            height_field=False,
                            draw_foot_path=False)
        agent = ARSAgent(self.normalizer, self.policy, env)
        my_path = os.path.abspath(os.path.dirname(__file__))
        if contacts:
            models_path = os.path.join(my_path,
                                       "../../spot_bullet/models/contact")
        else:
            models_path = os.path.join(my_path,
                                       "../../spot_bullet/models/no_contact")

        print("MODEL PATH: {}".format(my_path))
        file_name = "spot_ars_"
        if os.path.exists(models_path + "/" + file_name + str(agent_num) +
                          "_policy"):
            print("Loading Existing agent: {}".format(agent_num))
            agent.load(models_path + "/" + file_name + str(agent_num))
            agent.policy.episode_steps = np.inf
            self.policy = agent.policy

        self.action = np.zeros(action_dim)
        self.old_act = self.action[:actions_to_filter]

    def imu_cb(self, imu):
        """ Reads the IMU

            Args: imu
        """
        try:
            # Update imu
            self.imu = [
                imu.roll, imu.pitch,
                np.radians(imu.gyro_x),
                np.radians(imu.gyro_y),
                np.radians(imu.gyro_z), imu.acc_x, imu.acc_y, imu.acc_z - 9.81
            ]
            # log input data as debug-level message
            rospy.logdebug(imu)
        except rospy.ROSInterruptException:
            pass

    def cnt_cb(self, cnt):
        """ Reads the Contact Sensors

            Args: cnt
        """
        try:
            # Update contacts
            self.contacts = [cnt.FL, cnt.FR, cnt.BL, cnt.BR]
            # log input data as debug-level message
            rospy.logdebug(cnt)
        except rospy.ROSInterruptException:
            pass

    def cmd_cb(self, mini_cmd):
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

        # Move Type
        if self.mini_cmd.movement == "Stepping":
            step_or_view = False
        else:
            step_or_view = True

        if self.mini_cmd.motion != "Stop":
            self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
            self.SwingPeriod = np.clip(
                copy.deepcopy(self.BaseSwingPeriod) +
                (-self.mini_cmd.faster + -self.mini_cmd.slower) * SV_SCALE,
                self.SwingPeriod_LIMITS[0], self.SwingPeriod_LIMITS[1])
            if self.mini_cmd.movement == "Stepping":
                # Only take 2/3 of lateral step length or it will be too great.
                StepLength = self.mini_cmd.x_velocity + abs(
                    self.mini_cmd.y_velocity * 0.66)
                StepLength = np.clip(StepLength, -1.0, 1.0)
                StepLength *= STEPLENGTH_SCALE
                # Convert lateral joystick action into poolar coordinates
                # for lateral movement
                LateralFraction = self.mini_cmd.y_velocity * np.pi / 2
                YawRate = self.mini_cmd.rate * YAW_SCALE
                # NOTE: NO HEIGHT MOD DURING WALK
                pos = np.array([0.0, 0.0, 0.0])
                orn = np.array([0.0, 0.0, 0.0])
            else:
                StepLength = 0.0
                LateralFraction = 0.0
                YawRate = 0.0
                # RESET
                self.ClearanceHeight = copy.deepcopy(self.BaseClearanceHeight)
                self.PenetrationDepth = copy.deepcopy(
                    self.BasePenetrationDepth)
                self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
                # x offset
                pos = np.array([0.0, 0.0, self.mini_cmd.z * Z_SCALE_CTRL])
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
            self.ClearanceHeight = copy.deepcopy(self.BaseClearanceHeight)
            self.PenetrationDepth = copy.deepcopy(self.BasePenetrationDepth)
            self.StepVelocity = copy.deepcopy(self.BaseStepVelocity)
            self.SwingPeriod = copy.deepcopy(self.BaseSwingPeriod)

        # OPTIONAL: Agent
        if self.Agent and self.mini_cmd.motion != "Stop":
            phases = copy.deepcopy(self.bzg.Phases)
            # Total 12
            state = []
            # r, p, gz, gy, gz, ax, ay, az (8)
            state.extend(self.imu)
            # FL, FR, BL, BR (4)
            state.extend(phases)
            # FL, FR, BL, BR (4)
            if self.enable_contact:
                state.extend(self.contacts)
            self.normalizer.observe(state)
            # Don't normalize contacts
            state[:-4] = self.normalizer.normalize(state)[:-4]
            self.action = self.policy.evaluate(state, None, None)
            self.action = np.tanh(self.action)
            # EXP FILTER
            self.action[:actions_to_filter] = alpha * self.old_act + (
                1.0 - alpha) * self.action[:actions_to_filter]
            self.old_act = self.action[:actions_to_filter]

            self.ClearanceHeight += self.action[0] * CD_SCALE

        # Time
        dt = rospy.get_time() - self.time
        # print("dt: {}".format(dt))
        self.time = rospy.get_time()

        # Update Step Period
        self.bzg.Tswing = self.SwingPeriod

        # CLIP
        self.ClearanceHeight = np.clip(self.ClearanceHeight,
                                       self.ClearanceHeight_LIMITS[0],
                                       self.ClearanceHeight_LIMITS[1])
        self.PenetrationDepth = np.clip(self.PenetrationDepth,
                                        self.PenetrationDepth_LIMITS[0],
                                        self.PenetrationDepth_LIMITS[1])

        self.T_bf = self.bzg.GenerateTrajectory(StepLength, LateralFraction,
                                                YawRate, self.StepVelocity,
                                                self.T_bf0, self.T_bf,
                                                self.ClearanceHeight,
                                                self.PenetrationDepth,
                                                self.contacts, dt)

        T_bf_copy = copy.deepcopy(self.T_bf)
        # OPTIONAL: Agent
        if self.Agent and self.mini_cmd.motion != "Stop":
            self.action[2:] *= RESIDUALS_SCALE
            # Add DELTA to XYZ Foot Poses
            T_bf_copy["FL"][:3, 3] += self.action[2:5]
            T_bf_copy["FR"][:3, 3] += self.action[5:8]
            T_bf_copy["BL"][:3, 3] += self.action[8:11]
            T_bf_copy["BR"][:3, 3] += self.action[11:14]
            pos[2] += abs(self.action[1]) * Z_SCALE

        joint_angles = self.spot.IK(orn, pos, T_bf_copy)

        ja_msg = JointAngles()

        ja_msg.fls = np.degrees(joint_angles[0][0])
        ja_msg.fle = np.degrees(joint_angles[0][1])
        ja_msg.flw = np.degrees(joint_angles[0][2])

        ja_msg.frs = np.degrees(joint_angles[1][0])
        ja_msg.fre = np.degrees(joint_angles[1][1])
        ja_msg.frw = np.degrees(joint_angles[1][2])

        ja_msg.bls = np.degrees(joint_angles[2][0])
        ja_msg.ble = np.degrees(joint_angles[2][1])
        ja_msg.blw = np.degrees(joint_angles[2][2])

        ja_msg.brs = np.degrees(joint_angles[3][0])
        ja_msg.bre = np.degrees(joint_angles[3][1])
        ja_msg.brw = np.degrees(joint_angles[3][2])

        # Move Type
        ja_msg.step_or_view = step_or_view

        self.ja_pub.publish(ja_msg)


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