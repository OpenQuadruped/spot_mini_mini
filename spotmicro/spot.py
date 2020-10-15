"""
CODE BASED ON EXAMPLE FROM:
@misc{coumans2017pybullet,
  title={Pybullet, a python module for physics simulation in robotics, games and machine learning},
  author={Coumans, Erwin and Bai, Yunfei},
  url={www.pybullet.org},
  year={2017},
}

Example: minitaur.py
https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/bullet/minitaur.py
"""

import collections
import copy
import math
import re
import numpy as np
from . import motor
from spotmicro.util import pybullet_data
print(pybullet_data.getDataPath())
from spotmicro.Kinematics.SpotKinematics import SpotModel
import spotmicro.Kinematics.LieAlgebra as LA

INIT_POSITION = [0, 0, 0.25]
INIT_RACK_POSITION = [0, 0, 1]
# NOTE: URDF IS FACING THE WRONG WAY
# TEMP FIX
INIT_ORIENTATION = [0, 0, 0, 1]
OVERHEAT_SHUTDOWN_TORQUE = 2.45
OVERHEAT_SHUTDOWN_TIME = 1.0
# -math.pi / 5
INIT_LEG_POS = -0.658319
# math.pi / 3
INIT_FOOT_POS = 1.0472

OLD_LEG_POSITION = ["front_left", "front_right", "rear_left", "rear_right"]
OLD_MOTOR_NAMES = [
    "motor_front_left_shoulder", "motor_front_left_leg",
    "foot_motor_front_left", "motor_front_right_shoulder",
    "motor_front_right_leg", "foot_motor_front_right",
    "motor_rear_left_shoulder", "motor_rear_left_leg", "foot_motor_rear_left",
    "motor_rear_right_shoulder", "motor_rear_right_leg",
    "foot_motor_rear_right"
]

OLD_MOTOR_LIMITS_BY_NAME = {}
for name in OLD_MOTOR_NAMES:
    if "shoulder" in name:
        OLD_MOTOR_LIMITS_BY_NAME[name] = [-1.04, 1.04]
    elif "leg" in name:
        OLD_MOTOR_LIMITS_BY_NAME[name] = [-2.59, 1.571]
    elif "foot" in name:
        OLD_MOTOR_LIMITS_BY_NAME[name] = [-1.571, 2.9]

OLD_FOOT_NAMES = [
    "front_left_toe", "front_right_toe", "rear_left_toe", "rear_right_toe"
]

LEG_POSITION = ["front_left", "front_right", "back_left", "back_right"]
MOTOR_NAMES = [
    "motor_front_left_hip", "motor_front_left_upper_leg",
    "motor_front_left_lower_leg", "motor_front_right_hip",
    "motor_front_right_upper_leg", "motor_front_right_lower_leg",
    "motor_back_left_hip", "motor_back_left_upper_leg",
    "motor_back_left_lower_leg", "motor_back_right_hip",
    "motor_back_right_upper_leg", "motor_back_right_lower_leg"
]

MOTOR_LIMITS_BY_NAME = {}
for name in MOTOR_NAMES:
    if "hip" in name:
        MOTOR_LIMITS_BY_NAME[name] = [-1.04, 1.04]
    elif "upper_leg" in name:
        MOTOR_LIMITS_BY_NAME[name] = [-1.571, 2.59]
    elif "lower_leg" in name:
        MOTOR_LIMITS_BY_NAME[name] = [-2.9, 1.671]

FOOT_NAMES = [
    "front_left_leg_foot", "front_right_leg_foot", "back_left_leg_foot",
    "back_right_leg_foot"
]

_CHASSIS_NAME_PATTERN = re.compile(r"chassis\D*")
_MOTOR_NAME_PATTERN = re.compile(r"motor\D*")
_FOOT_NAME_PATTERN = re.compile(r"foot\D*")
SENSOR_NOISE_STDDEV = (0.0, 0.0, 0.0, 0.0, 0.0)
TWO_PI = 2 * math.pi


def MapToMinusPiToPi(angles):
    """Maps a list of angles to [-pi, pi].

  Args:
    angles: A list of angles in rad.
  Returns:
    A list of angle mapped to [-pi, pi].
  """
    mapped_angles = copy.deepcopy(angles)
    for i in range(len(angles)):
        mapped_angles[i] = math.fmod(angles[i], TWO_PI)
        if mapped_angles[i] >= math.pi:
            mapped_angles[i] -= TWO_PI
        elif mapped_angles[i] < -math.pi:
            mapped_angles[i] += TWO_PI
    return mapped_angles


class Spot(object):
    """The spot class that simulates a quadruped robot.

  """
    INIT_POSES = {
        'stand':
        np.array([
            0.15192765, 0.7552236, -1.5104472, -0.15192765, 0.7552236,
            -1.5104472, 0.15192765, 0.7552236, -1.5104472, -0.15192765,
            0.7552236, -1.5104472
        ]),
        'liedown':
        np.array([-0.4, -1.5, 6, 0.4, -1.5, 6, -0.4, -1.5, 6, 0.4, -1.5, 6]),
        'zero':
        np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    }

    def __init__(self,
                 pybullet_client,
                 urdf_root=pybullet_data.getDataPath(),
                 time_step=0.01,
                 action_repeat=1,
                 self_collision_enabled=False,
                 motor_velocity_limit=9.7,
                 pd_control_enabled=False,
                 accurate_motor_model_enabled=False,
                 remove_default_joint_damping=False,
                 max_force=100.0,
                 motor_kp=1.0,
                 motor_kd=0.02,
                 pd_latency=0.0,
                 control_latency=0.0,
                 observation_noise_stdev=SENSOR_NOISE_STDDEV,
                 torque_control_enabled=False,
                 motor_overheat_protection=False,
                 on_rack=False,
                 kd_for_pd_controllers=0.3,
                 pose_id='stand',
                 np_random=np.random,
                 contacts=True):
        """Constructs a spot and reset it to the initial states.

    Args:
      pybullet_client: The instance of BulletClient to manage different
        simulations.
      urdf_root: The path to the urdf folder.
      time_step: The time step of the simulation.
      action_repeat: The number of ApplyAction() for each control step.
      self_collision_enabled: Whether to enable self collision.
      motor_velocity_limit: The upper limit of the motor velocity.
      pd_control_enabled: Whether to use PD control for the motors.
      accurate_motor_model_enabled: Whether to use the accurate DC motor model.
      remove_default_joint_damping: Whether to remove the default joint damping.
      motor_kp: proportional gain for the accurate motor model.
      motor_kd: derivative gain for the accurate motor model.
      pd_latency: The latency of the observations (in seconds) used to calculate
        PD control. On the real hardware, it is the latency between the
        microcontroller and the motor controller.
      control_latency: The latency of the observations (in second) used to
        calculate action. On the real hardware, it is the latency from the motor
        controller, the microcontroller to the host (Nvidia TX2).
      observation_noise_stdev: The standard deviation of a Gaussian noise model
        for the sensor. It should be an array for separate sensors in the
        following order [motor_angle, motor_velocity, motor_torque,
        base_roll_pitch_yaw, base_angular_velocity]
      torque_control_enabled: Whether to use the torque control, if set to
        False, pose control will be used.
      motor_overheat_protection: Whether to shutdown the motor that has exerted
        large torque (OVERHEAT_SHUTDOWN_TORQUE) for an extended amount of time
        (OVERHEAT_SHUTDOWN_TIME). See ApplyAction() in spot.py for more
        details.
      on_rack: Whether to place the spot on rack. This is only used to debug
        the walking gait. In this mode, the spot's base is hanged midair so
        that its walking gait is clearer to visualize.
    """
        # SPOT MODEL
        self.spot = SpotModel()
        # Whether to include contact sensing
        self.contacts = contacts
        # Control Inputs
        self.StepLength = 0.0
        self.StepVelocity = 0.0
        self.LateralFraction = 0.0
        self.YawRate = 0.0
        # Leg Phases
        self.LegPhases = [0.0, 0.0, 0.0, 0.0]
        # used to calculate minitaur acceleration
        self.init_leg = INIT_LEG_POS
        self.init_foot = INIT_FOOT_POS
        self.prev_ang_twist = np.array([0, 0, 0])
        self.prev_lin_twist = np.array([0, 0, 0])
        self.prev_lin_acc = np.array([0, 0, 0])
        self.num_motors = 12
        self.num_legs = int(self.num_motors / 3)
        self._pybullet_client = pybullet_client
        self._action_repeat = action_repeat
        self._urdf_root = urdf_root
        self._self_collision_enabled = self_collision_enabled
        self._motor_velocity_limit = motor_velocity_limit
        self._pd_control_enabled = pd_control_enabled
        self._motor_direction = np.ones(self.num_motors)
        self._observed_motor_torques = np.zeros(self.num_motors)
        self._applied_motor_torques = np.zeros(self.num_motors)
        self._max_force = max_force
        self._pd_latency = pd_latency
        self._control_latency = control_latency
        self._observation_noise_stdev = observation_noise_stdev
        self._accurate_motor_model_enabled = accurate_motor_model_enabled
        self._remove_default_joint_damping = remove_default_joint_damping
        self._observation_history = collections.deque(maxlen=100)
        self._control_observation = []
        self._chassis_link_ids = [-1]
        self._leg_link_ids = []
        self._motor_link_ids = []
        self._foot_link_ids = []
        self._torque_control_enabled = torque_control_enabled
        self._motor_overheat_protection = motor_overheat_protection
        self._on_rack = on_rack
        self._pose_id = pose_id
        self.np_random = np_random
        if self._accurate_motor_model_enabled:
            self._kp = motor_kp
            self._kd = motor_kd
            self._motor_model = motor.MotorModel(
                torque_control_enabled=self._torque_control_enabled,
                kp=self._kp,
                kd=self._kd)
        elif self._pd_control_enabled:
            self._kp = 8
            self._kd = kd_for_pd_controllers
        else:
            self._kp = 1
            self._kd = 1
        self.time_step = time_step
        self._step_counter = 0
        # reset_time=-1.0 means skipping the reset motion.
        # See Reset for more details.
        self.Reset(reset_time=-1)
        self.init_on_rack_position = INIT_RACK_POSITION
        self.init_position = INIT_POSITION
        self.initial_pose = self.INIT_POSES[pose_id]

    def _RecordMassInfoFromURDF(self):
        self._base_mass_urdf = []
        for chassis_id in self._chassis_link_ids:
            self._base_mass_urdf.append(
                self._pybullet_client.getDynamicsInfo(self.quadruped,
                                                      chassis_id)[0])
        self._leg_masses_urdf = []
        for leg_id in self._leg_link_ids:
            self._leg_masses_urdf.append(
                self._pybullet_client.getDynamicsInfo(self.quadruped,
                                                      leg_id)[0])
        for motor_id in self._motor_link_ids:
            self._leg_masses_urdf.append(
                self._pybullet_client.getDynamicsInfo(self.quadruped,
                                                      motor_id)[0])

    def GetBaseMassFromURDF(self):
        """Get the mass of the base from the URDF file."""
        return self._base_mass_urdf

    def SetBaseMass(self, base_mass):
        for i in range(len(self._chassis_link_ids)):
            self._pybullet_client.changeDynamics(self.quadruped,
                                                 self._chassis_link_ids[i],
                                                 mass=base_mass[i])

    def _RecordInertiaInfoFromURDF(self):
        """Record the inertia of each body from URDF file."""
        self._link_urdf = []
        num_bodies = self._pybullet_client.getNumJoints(self.quadruped)
        for body_id in range(-1, num_bodies):  # -1 is for the base link.
            inertia = self._pybullet_client.getDynamicsInfo(
                self.quadruped, body_id)[2]
            self._link_urdf.append(inertia)
        # We need to use id+1 to index self._link_urdf because it has the base
        # (index = -1) at the first element.
        self._base_inertia_urdf = [
            self._link_urdf[chassis_id + 1]
            for chassis_id in self._chassis_link_ids
        ]
        self._leg_inertia_urdf = [
            self._link_urdf[leg_id + 1] for leg_id in self._leg_link_ids
        ]
        self._leg_inertia_urdf.extend([
            self._link_urdf[motor_id + 1] for motor_id in self._motor_link_ids
        ])

    def _BuildJointNameToIdDict(self):
        num_joints = self._pybullet_client.getNumJoints(self.quadruped)
        self._joint_name_to_id = {}
        for i in range(num_joints):
            joint_info = self._pybullet_client.getJointInfo(self.quadruped, i)
            self._joint_name_to_id[joint_info[1].decode(
                "UTF-8")] = joint_info[0]

    def _BuildMotorIdList(self):
        self._motor_id_list = [
            self._joint_name_to_id[motor_name] for motor_name in MOTOR_NAMES
        ]

    def _BuildFootIdList(self):
        self._foot_id_list = [
            self._joint_name_to_id[foot_name] for foot_name in FOOT_NAMES
        ]

        print(self._foot_id_list)

    def _BuildUrdfIds(self):
        """Build the link Ids from its name in the URDF file."""
        c = []
        m = []
        f = []
        lg = []
        num_joints = self._pybullet_client.getNumJoints(self.quadruped)
        self._chassis_link_ids = [-1]
        # the self._leg_link_ids include both the upper and lower links of the leg.
        self._leg_link_ids = []
        self._motor_link_ids = []
        self._foot_link_ids = []
        for i in range(num_joints):
            joint_info = self._pybullet_client.getJointInfo(self.quadruped, i)
            joint_name = joint_info[1].decode("UTF-8")
            joint_id = self._joint_name_to_id[joint_name]
            if _CHASSIS_NAME_PATTERN.match(joint_name):
                c.append(joint_name)
                self._chassis_link_ids.append(joint_id)
            elif _MOTOR_NAME_PATTERN.match(joint_name):
                m.append(joint_name)
                self._motor_link_ids.append(joint_id)
            elif _FOOT_NAME_PATTERN.match(joint_name):
                f.append(joint_name)
                self._foot_link_ids.append(joint_id)
            else:
                lg.append(joint_name)
                self._leg_link_ids.append(joint_id)
        self._leg_link_ids.extend(self._foot_link_ids)
        self._chassis_link_ids.sort()
        self._motor_link_ids.sort()
        self._foot_link_ids.sort()
        self._leg_link_ids.sort()

    def Reset(self,
              reload_urdf=True,
              default_motor_angles=None,
              reset_time=3.0):
        """Reset the spot to its initial states.

    Args:
      reload_urdf: Whether to reload the urdf file. If not, Reset() just place
        the spot back to its starting position.
      default_motor_angles: The default motor angles. If it is None, spot
        will hold a default pose for 100 steps. In
        torque control mode, the phase of holding the default pose is skipped.
      reset_time: The duration (in seconds) to hold the default motor angles. If
        reset_time <= 0 or in torque control mode, the phase of holding the
        default pose is skipped.
    """
        if self._on_rack:
            init_position = INIT_RACK_POSITION
        else:
            init_position = INIT_POSITION

        if reload_urdf:
            if self._self_collision_enabled:
                self.quadruped = self._pybullet_client.loadURDF(
                    pybullet_data.getDataPath() + "/assets/urdf/spot.urdf",
                    init_position,
                    useFixedBase=self._on_rack,
                    flags=self._pybullet_client.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
            else:
                self.quadruped = self._pybullet_client.loadURDF(
                    pybullet_data.getDataPath() + "/assets/urdf/spot.urdf",
                    init_position,
                    INIT_ORIENTATION,
                    useFixedBase=self._on_rack)
            self._BuildJointNameToIdDict()
            self._BuildUrdfIds()
            if self._remove_default_joint_damping:
                self._RemoveDefaultJointDamping()
            self._BuildMotorIdList()
            self._BuildFootIdList()
            self._RecordMassInfoFromURDF()
            self._RecordInertiaInfoFromURDF()
            self.ResetPose(add_constraint=True)
        else:
            self._pybullet_client.resetBasePositionAndOrientation(
                self.quadruped, init_position, INIT_ORIENTATION)
            self._pybullet_client.resetBaseVelocity(self.quadruped, [0, 0, 0],
                                                    [0, 0, 0])
            # self._pybullet_client.changeDynamics(self.quadruped, -1, lateralFriction=0.8)
            self.ResetPose(add_constraint=False)
        self._overheat_counter = np.zeros(self.num_motors)
        self._motor_enabled_list = [True] * self.num_motors
        self._step_counter = 0

        # Perform reset motion within reset_duration if in position control mode.
        # Nothing is performed if in torque control mode for now.
        self._observation_history.clear()
        if reset_time > 0.0 and default_motor_angles is not None:
            self.RealisticObservation()
            for _ in range(100):
                self.ApplyAction(self.initial_pose)
                self._pybullet_client.stepSimulation()
                self.RealisticObservation()
            num_steps_to_reset = int(reset_time / self.time_step)
            for _ in range(num_steps_to_reset):
                self.ApplyAction(default_motor_angles)
                self._pybullet_client.stepSimulation()
                self.RealisticObservation()
        self.RealisticObservation()

        # Set Foot Friction
        self.SetFootFriction()

    def _RemoveDefaultJointDamping(self):
        num_joints = self._pybullet_client.getNumJoints(self.quadruped)
        for i in range(num_joints):
            joint_info = self._pybullet_client.getJointInfo(self.quadruped, i)
            self._pybullet_client.changeDynamics(joint_info[0],
                                                 -1,
                                                 linearDamping=0,
                                                 angularDamping=0)

    def _SetMotorTorqueById(self, motor_id, torque):
        self._pybullet_client.setJointMotorControl2(
            bodyIndex=self.quadruped,
            jointIndex=motor_id,
            controlMode=self._pybullet_client.TORQUE_CONTROL,
            force=torque)

    def _SetDesiredMotorAngleById(self, motor_id, desired_angle):
        if self._pd_control_enabled or self._accurate_motor_model_enabled:
            self._pybullet_client.setJointMotorControl2(
                bodyIndex=self.quadruped,
                jointIndex=motor_id,
                controlMode=self._pybullet_client.POSITION_CONTROL,
                targetPosition=desired_angle,
                positionGain=self._kp,
                velocityGain=self._kd,
                force=self._max_force)
        # Pybullet has a 'perfect' joint controller with its default p,d
        else:
            self._pybullet_client.setJointMotorControl2(
                bodyIndex=self.quadruped,
                jointIndex=motor_id,
                controlMode=self._pybullet_client.POSITION_CONTROL,
                targetPosition=desired_angle)

    def _SetDesiredMotorAngleByName(self, motor_name, desired_angle):
        self._SetDesiredMotorAngleById(self._joint_name_to_id[motor_name],
                                       desired_angle)

    def ResetPose(self, add_constraint):
        """Reset the pose of the spot.

    Args:
      add_constraint: Whether to add a constraint at the joints of two feet.
    """
        for i in range(self.num_legs):
            self._ResetPoseForLeg(i, add_constraint)

    def _ResetPoseForLeg(self, leg_id, add_constraint):
        """Reset the initial pose for the leg.

    Args:
      leg_id: It should be 0, 1, 2, or 3, which represents the leg at
        front_left, back_left, front_right and back_right.
      add_constraint: Whether to add a constraint at the joints of two feet.
    """
        knee_friction_force = 0
        pi = math.pi
        leg_position = LEG_POSITION[leg_id]
        self._pybullet_client.resetJointState(
            self.quadruped,
            self._joint_name_to_id["motor_" + leg_position + "_hip"],
            self.INIT_POSES[self._pose_id][3 * leg_id],
            targetVelocity=0)

        self._pybullet_client.resetJointState(
            self.quadruped,
            self._joint_name_to_id["motor_" + leg_position + "_upper_leg"],
            self.INIT_POSES[self._pose_id][3 * leg_id + 1],
            targetVelocity=0)
        self._pybullet_client.resetJointState(
            self.quadruped,
            self._joint_name_to_id["motor_" + leg_position + "_lower_leg"],
            self.INIT_POSES[self._pose_id][3 * leg_id + 2],
            targetVelocity=0)

        if self._accurate_motor_model_enabled or self._pd_control_enabled:
            # Disable the default motor in pybullet.
            self._pybullet_client.setJointMotorControl2(
                bodyIndex=self.quadruped,
                jointIndex=(self._joint_name_to_id["motor_" + leg_position +
                                                   "_hip"]),
                controlMode=self._pybullet_client.VELOCITY_CONTROL,
                targetVelocity=0,
                force=knee_friction_force)
            self._pybullet_client.setJointMotorControl2(
                bodyIndex=self.quadruped,
                jointIndex=(self._joint_name_to_id["motor_" + leg_position +
                                                   "_upper_leg"]),
                controlMode=self._pybullet_client.VELOCITY_CONTROL,
                targetVelocity=0,
                force=knee_friction_force)
            self._pybullet_client.setJointMotorControl2(
                bodyIndex=self.quadruped,
                jointIndex=(self._joint_name_to_id["motor_" + leg_position +
                                                   "_lower_leg"]),
                controlMode=self._pybullet_client.VELOCITY_CONTROL,
                targetVelocity=0,
                force=knee_friction_force)

    def GetBasePosition(self):
        """Get the position of spot's base.

        Returns:
          The position of spot's base.
        """
        position, _ = (self._pybullet_client.getBasePositionAndOrientation(
            self.quadruped))
        return position

    def GetBaseOrientation(self):
        """Get the orientation of spot's base, represented as quaternion.

        Returns:
          The orientation of spot's base.
        """
        _, orientation = (self._pybullet_client.getBasePositionAndOrientation(
            self.quadruped))
        return orientation

    def GetBaseRollPitchYaw(self):
        """Get the rate of orientation change of the spot's base in euler angle.

        Returns:
          rate of (roll, pitch, yaw) change of the spot's base.
        """
        vel = self._pybullet_client.getBaseVelocity(self.quadruped)
        return np.asarray([vel[1][0], vel[1][1], vel[1][2]])

    def GetBaseRollPitchYawRate(self):
        """Get the rate of orientation change of the spot's base in euler angle.

        This function mimicks the noisy sensor reading and adds latency.
        Returns:
          rate of (roll, pitch, yaw) change of the spot's base polluted by noise
          and latency.
        """
        return self._AddSensorNoise(
            np.array(self._control_observation[3 * self.num_motors +
                                               4:3 * self.num_motors + 7]),
            self._observation_noise_stdev[4])

    def GetBaseTwist(self):
        """Get the Twist of minitaur's base.
        Returns:
          The Twist of the minitaur's base.
        """
        return self._pybullet_client.getBaseVelocity(self.quadruped)

    def GetActionDimension(self):
        """Get the length of the action list.

        Returns:
          The length of the action list.
        """
        return self.num_motors

    def GetObservationUpperBound(self):
        """Get the upper bound of the observation.
        Returns:
          The upper bound of an observation. See GetObservation() for the details
            of each element of an observation.
          NOTE: Changed just like GetObservation()
        """
        upper_bound = np.array([0.0] * self.GetObservationDimension())
        # roll, pitch
        upper_bound[0:2] = 2.0 * np.pi
        # acc, rate in x,y,z
        upper_bound[2:8] = np.inf
        # Leg Phases
        upper_bound[8:12] = 2.0
        # Contacts
        if self.contacts:
            upper_bound[12:] = 1.0
        return upper_bound

    def GetObservationLowerBound(self):
        """Get the lower bound of the observation."""
        return -self.GetObservationUpperBound()

    def GetObservationDimension(self):
        """Get the length of the observation list.
    Returns:
      The length of the observation list.
    """
        return len(self.GetObservation())

    def GetObservation(self):
        """Get the observations of minitaur.
        It includes the angles, velocities, torques and the orientation of the base.
        Returns:
          The observation list. observation[0:8] are motor angles. observation[8:16]
          are motor velocities, observation[16:24] are motor torques.
          observation[24:28] is the orientation of the base, in quaternion form.
          NOTE: DIVERGES FROM STOCK MINITAUR ENV. WILL LEAVE ORIGINAL COMMENTED
          For my purpose, the observation space includes Roll and Pitch, as well as
          acceleration and gyroscopic rate along the x,y,z axes. All of this
          information can be collected from an onboard IMU. The reward function
          will contain a hidden velocity reward (fwd, bwd) which cannot be measured
          and so is not included. For spinning, the gyroscopic z rate will be used
          as the (explicit) velocity reward.
          This version operates without motor torques, angles and velocities. Erwin
          Coumans' paper suggests a sparse observation space leads to higher reward

          # NOTE: use True version for perfect data, or other for realistic data
        """
        observation = []
        # GETTING TWIST IN BODY FRAME
        pos = self.GetBasePosition()
        orn = self.GetBaseOrientation()
        roll, pitch, yaw = self._pybullet_client.getEulerFromQuaternion(
            [orn[0], orn[1], orn[2], orn[3]])
        # rpy = LA.RPY(roll, pitch, yaw)
        # R, _ = LA.TransToRp(rpy)
        # T_wb = LA.RpToTrans(R, np.array([pos[0], pos[1], pos[2]]))
        # T_bw = LA.TransInv(T_wb)
        # Adj_Tbw = LA.Adjoint(T_bw)

        # Get Linear and Angular Twist in WORLD FRAME
        lin_twist, ang_twist = self.GetBaseTwist()

        lin_twist = np.array([lin_twist[0], lin_twist[1], lin_twist[2]])
        ang_twist = np.array([ang_twist[0], ang_twist[1], ang_twist[2]])

        # Vw = np.concatenate((ang_twist, lin_twist))
        # Vb = np.dot(Adj_Tbw, Vw)

        # roll, pitch, _ = self._pybullet_client.getEulerFromQuaternion(
        #     [orn[0], orn[1], orn[2], orn[3]])

        # # Get linear accelerations
        # lin_twist = -Vb[3:]
        # ang_twist = Vb[:3]
        lin_acc = lin_twist - self.prev_lin_twist
        if lin_acc.all() == 0.0:
            lin_acc = self.prev_lin_acc
        self.prev_lin_acc = lin_acc
        # print("LIN TWIST: ", lin_twist)
        self.prev_lin_twist = lin_twist
        self.prev_ang_twist = ang_twist

        # Get Contacts
        CONTACT = list(self._pybullet_client.getContactPoints(self.quadruped))

        FLC = 0
        FRC = 0
        BLC = 0
        BRC = 0

        if len(CONTACT) > 0:
            for i in range(len(CONTACT)):
                Contact_Link_Index = CONTACT[i][3]
                if Contact_Link_Index == self._foot_id_list[0]:
                    FLC = 1
                    # print("FL CONTACT")
                if Contact_Link_Index == self._foot_id_list[1]:
                    FRC = 1
                    # print("FR CONTACT")
                if Contact_Link_Index == self._foot_id_list[2]:
                    BLC = 1
                    # print("BL CONTACT")
                if Contact_Link_Index == self._foot_id_list[3]:
                    BRC = 1
                    # print("BR CONTACT")
        # order: roll, pitch, gyro(x,y,z), acc(x, y, z)
        observation.append(roll)
        observation.append(pitch)
        observation.extend(list(ang_twist))
        observation.extend(list(lin_acc))
        # Control Input
        # observation.append(self.StepLength)
        # observation.append(self.StepVelocity)
        # observation.append(self.LateralFraction)
        # observation.append(self.YawRate)
        observation.extend(self.LegPhases)
        if self.contacts:
            observation.append(FLC)
            observation.append(FRC)
            observation.append(BLC)
            observation.append(BRC)
        # print("CONTACTS: {}  {}  {}  {}".format(FLC, FRC, BLC, BRC))
        return observation

    def GetControlInput(self, controller):
        """ Store Control Input as Observation
        """
        _, _, StepLength, LateralFraction, YawRate, StepVelocity, _, _ = controller.return_bezier_params(
        )

        self.StepLength = StepLength
        self.StepVelocity = StepVelocity
        self.LateralFraction = LateralFraction
        self.YawRate = YawRate

    def GetLegPhases(self, TrajectoryGenerator):
        """ Leg phases according to TG from 0->2
            0->1: Stance
            1->2 Swing
        """
        self.LegPhases = TrajectoryGenerator.Phases

    def GetExternalObservations(self, TrajectoryGenerator, controller):
        """ Augment State Space
        """
        self.GetControlInput(controller)
        self.GetLegPhases(TrajectoryGenerator)

    def ConvertFromLegModel(self, action):
        # TODO
        joint_angles = action

        return joint_angles

    def ApplyMotorLimits(self, joint_angles):
        eps = 0.001
        for i in range(len(joint_angles)):
            LIM = MOTOR_LIMITS_BY_NAME[MOTOR_NAMES[i]]
            joint_angles[i] = np.clip(joint_angles[i], LIM[0] + eps,
                                      LIM[1] - eps)
        return joint_angles

    def ApplyAction(self, motor_commands):
        """Set the desired motor angles to the motors of the minitaur.
        The desired motor angles are clipped based on the maximum allowed velocity.
        If the pd_control_enabled is True, a torque is calculated according to
        the difference between current and desired joint angle, as well as the joint
        velocity. This torque is exerted to the motor. For more information about
        PD control, please refer to: https://en.wikipedia.org/wiki/PID_controller.
        Args:
          motor_commands: The eight desired motor angles.
        """
        # FIRST, APPLY MOTOR LIMITS:
        motor_commands = self.ApplyMotorLimits(motor_commands)

        if self._motor_velocity_limit < np.inf:
            current_motor_angle = self.GetMotorAngles()
            motor_commands_max = (current_motor_angle +
                                  self.time_step * self._motor_velocity_limit)
            motor_commands_min = (current_motor_angle -
                                  self.time_step * self._motor_velocity_limit)
            motor_commands = np.clip(motor_commands, motor_commands_min,
                                     motor_commands_max)

        if self._accurate_motor_model_enabled or self._pd_control_enabled:
            q = self.GetMotorAngles()
            qdot = self.GetMotorVelocities()
            if self._accurate_motor_model_enabled:
                actual_torque, observed_torque = self._motor_model.convert_to_torque(
                    motor_commands, q, qdot)
                if self._motor_overheat_protection:
                    for i in range(self.num_motors):
                        if abs(actual_torque[i]) > OVERHEAT_SHUTDOWN_TORQUE:
                            self._overheat_counter[i] += 1
                        else:
                            self._overheat_counter[i] = 0
                        if (self._overheat_counter[i] >
                                OVERHEAT_SHUTDOWN_TIME / self.time_step):
                            self._motor_enabled_list[i] = False

                # The torque is already in the observation space because we use
                # GetMotorAngles and GetMotorVelocities.
                self._observed_motor_torques = observed_torque

                # Transform into the motor space when applying the torque.
                self._applied_motor_torque = np.multiply(
                    actual_torque, self._motor_direction)

                for motor_id, motor_torque, motor_enabled in zip(
                        self._motor_id_list, self._applied_motor_torque,
                        self._motor_enabled_list):
                    if motor_enabled:
                        self._SetMotorTorqueById(motor_id, motor_torque)
                    else:
                        self._SetMotorTorqueById(motor_id, 0)
            else:
                torque_commands = -self._kp * (
                    q - motor_commands) - self._kd * qdot

                # The torque is already in the observation space because we use
                # GetMotorAngles and GetMotorVelocities.
                self._observed_motor_torques = torque_commands

                # Transform into the motor space when applying the torque.
                self._applied_motor_torques = np.multiply(
                    self._observed_motor_torques, self._motor_direction)

                for motor_id, motor_torque in zip(self._motor_id_list,
                                                  self._applied_motor_torques):
                    self._SetMotorTorqueById(motor_id, motor_torque)
        else:
            motor_commands_with_direction = np.multiply(
                motor_commands, self._motor_direction)
            for motor_id, motor_command_with_direction in zip(
                    self._motor_id_list, motor_commands_with_direction):
                self._SetDesiredMotorAngleById(motor_id,
                                               motor_command_with_direction)

    def Step(self, action):
        for _ in range(self._action_repeat):
            self.ApplyAction(action)
            self._pybullet_client.stepSimulation()
            self.RealisticObservation()
            self._step_counter += 1

    def GetTimeSinceReset(self):
        return self._step_counter * self.time_step

    def GetMotorAngles(self):
        """Gets the eight motor angles at the current moment, mapped to [-pi, pi].

        Returns:
          Motor angles, mapped to [-pi, pi].
        """
        motor_angles = [
            self._pybullet_client.getJointState(self.quadruped, motor_id)[0]
            for motor_id in self._motor_id_list
        ]
        motor_angles = np.multiply(motor_angles, self._motor_direction)
        return MapToMinusPiToPi(motor_angles)

    def GetMotorVelocities(self):
        """Get the velocity of all eight motors.

        Returns:
          Velocities of all eight motors.
        """
        motor_velocities = [
            self._pybullet_client.getJointState(self.quadruped, motor_id)[1]
            for motor_id in self._motor_id_list
        ]
        motor_velocities = np.multiply(motor_velocities, self._motor_direction)
        return motor_velocities

    def GetMotorTorques(self):
        """Get the amount of torque the motors are exerting.

    Returns:
      Motor torques of all eight motors.
    """
        if self._accurate_motor_model_enabled or self._pd_control_enabled:
            return self._observed_motor_torques
        else:
            motor_torques = [
                self._pybullet_client.getJointState(self.quadruped,
                                                    motor_id)[3]
                for motor_id in self._motor_id_list
            ]
            motor_torques = np.multiply(motor_torques, self._motor_direction)
        return motor_torques

    def GetBaseMassesFromURDF(self):
        """Get the mass of the base from the URDF file."""
        return self._base_mass_urdf

    def GetBaseInertiasFromURDF(self):
        """Get the inertia of the base from the URDF file."""
        return self._base_inertia_urdf

    def GetLegMassesFromURDF(self):
        """Get the mass of the legs from the URDF file."""
        return self._leg_masses_urdf

    def GetLegInertiasFromURDF(self):
        """Get the inertia of the legs from the URDF file."""
        return self._leg_inertia_urdf

    def SetBaseMasses(self, base_mass):
        """Set the mass of spot's base.

        Args:
          base_mass: A list of masses of each body link in CHASIS_LINK_IDS. The
            length of this list should be the same as the length of CHASIS_LINK_IDS.
        Raises:
          ValueError: It is raised when the length of base_mass is not the same as
            the length of self._chassis_link_ids.
        """
        if len(base_mass) != len(self._chassis_link_ids):
            raise ValueError(
                "The length of base_mass {} and self._chassis_link_ids {} are not "
                "the same.".format(len(base_mass),
                                   len(self._chassis_link_ids)))
        for chassis_id, chassis_mass in zip(self._chassis_link_ids, base_mass):
            self._pybullet_client.changeDynamics(self.quadruped,
                                                 chassis_id,
                                                 mass=chassis_mass)

    def SetLegMasses(self, leg_masses):
        """Set the mass of the legs.
        Args:
          leg_masses: The leg and motor masses for all the leg links and motors.

        Raises:
          ValueError: It is raised when the length of masses is not equal to number
            of links + motors.
        """
        if len(leg_masses) != len(self._leg_link_ids) + len(
                self._motor_link_ids):
            raise ValueError("The number of values passed to SetLegMasses are "
                             "different than number of leg links and motors.")
        for leg_id, leg_mass in zip(self._leg_link_ids, leg_masses):
            self._pybullet_client.changeDynamics(self.quadruped,
                                                 leg_id,
                                                 mass=leg_mass)
        motor_masses = leg_masses[len(self._leg_link_ids):]
        for link_id, motor_mass in zip(self._motor_link_ids, motor_masses):
            self._pybullet_client.changeDynamics(self.quadruped,
                                                 link_id,
                                                 mass=motor_mass)

    def SetBaseInertias(self, base_inertias):
        """Set the inertias of spot's base.
        Args:
          base_inertias: A list of inertias of each body link in CHASIS_LINK_IDS.
            The length of this list should be the same as the length of
            CHASIS_LINK_IDS.
        Raises:
          ValueError: It is raised when the length of base_inertias is not the same
            as the length of self._chassis_link_ids and base_inertias contains
            negative values.
        """
        if len(base_inertias) != len(self._chassis_link_ids):
            raise ValueError(
                "The length of base_inertias {} and self._chassis_link_ids {} are "
                "not the same.".format(len(base_inertias),
                                       len(self._chassis_link_ids)))
        for chassis_id, chassis_inertia in zip(self._chassis_link_ids,
                                               base_inertias):
            for inertia_value in chassis_inertia:
                if (np.asarray(inertia_value) < 0).any():
                    raise ValueError(
                        "Values in inertia matrix should be non-negative.")
            self._pybullet_client.changeDynamics(
                self.quadruped,
                chassis_id,
                localInertiaDiagonal=chassis_inertia)

    def SetLegInertias(self, leg_inertias):
        """Set the inertias of the legs.

        Args:
          leg_inertias: The leg and motor inertias for all the leg links and motors.

        Raises:
          ValueError: It is raised when the length of inertias is not equal to
          the number of links + motors or leg_inertias contains negative values.
        """

        if len(leg_inertias) != len(self._leg_link_ids) + len(
                self._motor_link_ids):
            raise ValueError("The number of values passed to SetLegMasses are "
                             "different than number of leg links and motors.")
        for leg_id, leg_inertia in zip(self._leg_link_ids, leg_inertias):
            for inertia_value in leg_inertias:
                if (np.asarray(inertia_value) < 0).any():
                    raise ValueError(
                        "Values in inertia matrix should be non-negative.")
            self._pybullet_client.changeDynamics(
                self.quadruped, leg_id, localInertiaDiagonal=leg_inertia)

        motor_inertias = leg_inertias[len(self._leg_link_ids):]
        for link_id, motor_inertia in zip(self._motor_link_ids,
                                          motor_inertias):
            for inertia_value in motor_inertias:
                if (np.asarray(inertia_value) < 0).any():
                    raise ValueError(
                        "Values in inertia matrix should be non-negative.")
            self._pybullet_client.changeDynamics(
                self.quadruped, link_id, localInertiaDiagonal=motor_inertia)

    def SetFootFriction(self, foot_friction=100.0):
        """Set the lateral friction of the feet.

        Args:
          foot_friction: The lateral friction coefficient of the foot. This value is
            shared by all four feet.
        """
        for link_id in self._foot_link_ids:
            self._pybullet_client.changeDynamics(self.quadruped,
                                                 link_id,
                                                 lateralFriction=foot_friction)

    # TODO(b/73748980): Add more API's to set other contact parameters.
    def SetFootRestitution(self, link_id, foot_restitution=1.0):
        """Set the coefficient of restitution at the feet.

        Args:
          foot_restitution: The coefficient of restitution (bounciness) of the feet.
            This value is shared by all four feet.
        """
        self._pybullet_client.changeDynamics(self.quadruped,
                                             link_id,
                                             restitution=foot_restitution)

    def SetJointFriction(self, joint_frictions):
        for knee_joint_id, friction in zip(self._foot_link_ids,
                                           joint_frictions):
            self._pybullet_client.setJointMotorControl2(
                bodyIndex=self.quadruped,
                jointIndex=knee_joint_id,
                controlMode=self._pybullet_client.VELOCITY_CONTROL,
                targetVelocity=0,
                force=friction)

    def GetNumKneeJoints(self):
        return len(self._foot_link_ids)

    def SetBatteryVoltage(self, voltage):
        if self._accurate_motor_model_enabled:
            self._motor_model.set_voltage(voltage)

    def SetMotorViscousDamping(self, viscous_damping):
        if self._accurate_motor_model_enabled:
            self._motor_model.set_viscous_damping(viscous_damping)

    def RealisticObservation(self):
        """Receive the observation from sensors.

        This function is called once per step. The observations are only updated
        when this function is called.
        """
        self._observation_history.appendleft(self.GetObservation())
        self._control_observation = self._GetDelayedObservation(
            self._control_latency)
        self._control_observation = self._AddSensorNoise(
            self._control_observation, self._observation_noise_stdev)
        return self._control_observation

    def _GetDelayedObservation(self, latency):
        """Get observation that is delayed by the amount specified in latency.

        Args:
          latency: The latency (in seconds) of the delayed observation.
        Returns:
          observation: The observation which was actually latency seconds ago.
        """
        if latency <= 0 or len(self._observation_history) == 1:
            observation = self._observation_history[0]
        else:
            n_steps_ago = int(latency / self.time_step)
            if n_steps_ago + 1 >= len(self._observation_history):
                return self._observation_history[-1]
            remaining_latency = latency - n_steps_ago * self.time_step
            blend_alpha = remaining_latency / self.time_step
            observation = (
                (1.0 - blend_alpha) *
                np.array(self._observation_history[n_steps_ago]) +
                blend_alpha *
                np.array(self._observation_history[n_steps_ago + 1]))
        return observation

    def _GetPDObservation(self):
        pd_delayed_observation = self._GetDelayedObservation(self._pd_latency)
        q = pd_delayed_observation[0:self.num_motors]
        qdot = pd_delayed_observation[self.num_motors:2 * self.num_motors]
        return (np.array(q), np.array(qdot))

    def _AddSensorNoise(self, observation, noise_stdev):
        # if self._observation_noise_stdev > 0:
        #     observation += (self.np_random.normal(scale=noise_stdev,
        #                                           size=observation.shape) *
        #                     self.GetObservationUpperBound())
        return observation

    def SetControlLatency(self, latency):
        """Set the latency of the control loop.

        It measures the duration between sending an action from Nvidia TX2 and
        receiving the observation from microcontroller.

        Args:
          latency: The latency (in seconds) of the control loop.
        """
        self._control_latency = latency

    def GetControlLatency(self):
        """Get the control latency.

        Returns:
          The latency (in seconds) between when the motor command is sent and when
            the sensor measurements are reported back to the controller.
        """
        return self._control_latency

    def SetMotorGains(self, kp, kd):
        """Set the gains of all motors.

        These gains are PD gains for motor positional control. kp is the
        proportional gain and kd is the derivative gain.

        Args:
          kp: proportional gain of the motors.
          kd: derivative gain of the motors.
        """
        self._kp = kp
        self._kd = kd
        if self._accurate_motor_model_enabled:
            self._motor_model.set_motor_gains(kp, kd)

    def GetMotorGains(self):
        """Get the gains of the motor.

        Returns:
          The proportional gain.
          The derivative gain.
        """
        return self._kp, self._kd

    def SetMotorStrengthRatio(self, ratio):
        """Set the strength of all motors relative to the default value.

        Args:
          ratio: The relative strength. A scalar range from 0.0 to 1.0.
        """
        if self._accurate_motor_model_enabled:
            self._motor_model.set_strength_ratios([ratio] * self.num_motors)

    def SetMotorStrengthRatios(self, ratios):
        """Set the strength of each motor relative to the default value.

        Args:
          ratios: The relative strength. A numpy array ranging from 0.0 to 1.0.
        """
        if self._accurate_motor_model_enabled:
            self._motor_model.set_strength_ratios(ratios)

    def SetTimeSteps(self, action_repeat, simulation_step):
        """Set the time steps of the control and simulation.

        Args:
          action_repeat: The number of simulation steps that the same action is
            repeated.
          simulation_step: The simulation time step.
        """
        self.time_step = simulation_step
        self._action_repeat = action_repeat

    @property
    def chassis_link_ids(self):
        return self._chassis_link_ids
