"""This file implements the gym environment of spot alternating legs.

"""
import math

from gym import spaces
import numpy as np
from .. import spot_gym_env

DESIRED_PITCH = 0
NUM_LEGS = 4
NUM_MOTORS = 3 * NUM_LEGS
STEP_PERIOD = 1.0 / 4.5


class spotWalkEnv(spot_gym_env.spotGymEnv):
    """The gym environment for the spot.

  It simulates the locomotion of a spot, a quadruped robot. The state space
  include the angles, velocities and torques for all the motors and the action
  space is the desired motor angle for each motor. The reward function is based
  on how far the spot walks in 1000 steps and penalizes the energy
  expenditure.

  """
    metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 66}

    def __init__(self,
                 urdf_version=None,
                 control_time_step=0.006,
                 action_repeat=6,
                 control_latency=0,
                 pd_latency=0,
                 on_rack=False,
                 motor_kp=1.0,
                 motor_kd=0.02,
                 remove_default_joint_damping=False,
                 render=False,
                 num_steps_to_log=1000,
                 env_randomizer=None,
                 log_path=None):
        """Initialize the spot alternating legs gym environment.

    Args:
      urdf_version: [DEFAULT_URDF_VERSION, DERPY_V0_URDF_VERSION] are allowable
        versions. If None, DEFAULT_URDF_VERSION is used. Refer to
        spot_gym_env for more details.
      control_time_step: The time step between two successive control signals.
      action_repeat: The number of simulation steps that an action is repeated.
      control_latency: The latency between get_observation() and the actual
        observation. See minituar.py for more details.
      pd_latency: The latency used to get motor angles/velocities used to
        compute PD controllers. See spot.py for more details.
      on_rack: Whether to place the spot on rack. This is only used to debug
        the walking gait. In this mode, the spot's base is hung midair so
        that its walking gait is clearer to visualize.
      motor_kp: The P gain of the motor.
      motor_kd: The D gain of the motor.
      remove_default_joint_damping: Whether to remove the default joint damping.
      render: Whether to render the simulation.
      num_steps_to_log: The max number of control steps in one episode. If the
        number of steps is over num_steps_to_log, the environment will still
        be running, but only first num_steps_to_log will be recorded in logging.
      env_randomizer: An instance (or a list) of EnvRanzomier(s) that can
        randomize the environment during when env.reset() is called and add
        perturbations when env.step() is called.
      log_path: The path to write out logs. For the details of logging, refer to
        spot_logging.proto.
    """
        super(spotWalkEnv,
              self).__init__(urdf_version=urdf_version,
                             accurate_motor_model_enabled=True,
                             motor_overheat_protection=True,
                             hard_reset=False,
                             motor_kp=motor_kp,
                             motor_kd=motor_kd,
                             remove_default_joint_damping=remove_default_joint_damping,
                             control_latency=control_latency,
                             pd_latency=pd_latency,
                             on_rack=on_rack,
                             render=render,
                             num_steps_to_log=num_steps_to_log,
                             env_randomizer=env_randomizer,
                             log_path=log_path,
                             control_time_step=control_time_step,
                             action_repeat=action_repeat)

        action_dim = 12
        action_high = np.array([0.1] * action_dim)
        self.action_space = spaces.Box(-action_high, action_high)
        self._cam_dist = 1.0
        self._cam_yaw = 30
        self._cam_pitch = -30

    def reset(self):
        self.desired_pitch = DESIRED_PITCH
        super(spotWalkEnv, self).reset()
        return self._get_observation()

    def _convert_from_leg_model(self, leg_pose):
        motor_pose = np.zeros(NUM_MOTORS)
        for i in range(NUM_LEGS):
            if i % 2 == 0:
                motor_pose[3 * i] = 0.1
            else:
                motor_pose[3 * i] = -0.1
            motor_pose[3 * i + 1] = leg_pose[3 * i + 1]
            motor_pose[3 * i + 2] = leg_pose[3 * i + 2]
        return motor_pose

    def _signal(self, t):
        initial_pose = self.spot.initial_pose
        period = STEP_PERIOD
        l_extension = 0.2 * math.cos(2 * math.pi / period * t)
        l_swing = -l_extension
        extension = 0.3 * math.cos(2 * math.pi / period * t)
        swing = -extension
        pose = np.array([0, l_extension, extension,
                         0, l_swing, swing,
                         0, l_swing, swing,
                         0, l_extension, extension])

        signal = initial_pose + pose
        return signal

    def _transform_action_to_motor_command(self, action):
        action += self._signal(self.spot.GetTimeSinceReset())
        action = self._convert_from_leg_model(action)
        return action

    def set_swing_offset(self, value):
        """Set the swing offset of each leg.

    It is to mimic the bent leg.

    Args:
      value: A list of four values.
    """
        self._swing_offset = value

    def set_extension_offset(self, value):
        """Set the extension offset of each leg.

    It is to mimic the bent leg.

    Args:
      value: A list of four values.
    """
        self._extension_offset = value

    def set_desired_pitch(self, value):
        """Set the desired pitch of the base, which is a user input.

    Args:
      value: A scalar.
    """
        self.desired_pitch = value
