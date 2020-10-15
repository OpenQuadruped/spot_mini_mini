"""
CODE BASED ON EXAMPLE FROM:
@misc{coumans2017pybullet,
  title={Pybullet, a python module for physics simulation in robotics, games and machine learning},
  author={Coumans, Erwin and Bai, Yunfei},
  url={www.pybullet.org},
  year={2017},
}

Example: minitaur_gym_env.py
https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/minitaur/envs/minitaur_gym_env.py
"""
import math
import time
import gym
import numpy as np
import pybullet
import pybullet_data
from gym import spaces
from gym.utils import seeding
from pkg_resources import parse_version
from spotmicro import spot
import pybullet_utils.bullet_client as bullet_client
from gym.envs.registration import register
from spotmicro.heightfield import HeightField
from spotmicro.OpenLoopSM.SpotOL import BezierStepper
import spotmicro.Kinematics.LieAlgebra as LA
from spotmicro.spot_env_randomizer import SpotEnvRandomizer

NUM_SUBSTEPS = 5
NUM_MOTORS = 12
MOTOR_ANGLE_OBSERVATION_INDEX = 0
MOTOR_VELOCITY_OBSERVATION_INDEX = MOTOR_ANGLE_OBSERVATION_INDEX + NUM_MOTORS
MOTOR_TORQUE_OBSERVATION_INDEX = MOTOR_VELOCITY_OBSERVATION_INDEX + NUM_MOTORS
BASE_ORIENTATION_OBSERVATION_INDEX = MOTOR_TORQUE_OBSERVATION_INDEX + NUM_MOTORS
ACTION_EPS = 0.01
OBSERVATION_EPS = 0.01
RENDER_HEIGHT = 720
RENDER_WIDTH = 960
SENSOR_NOISE_STDDEV = spot.SENSOR_NOISE_STDDEV
DEFAULT_URDF_VERSION = "default"
NUM_SIMULATION_ITERATION_STEPS = 1000

spot_URDF_VERSION_MAP = {DEFAULT_URDF_VERSION: spot.Spot}

# Register as OpenAI Gym Environment
register(
    id="SpotMicroEnv-v0",
    entry_point='spotmicro.spot_gym_env:spotGymEnv',
    max_episode_steps=1000,
)


def convert_to_list(obj):
    try:
        iter(obj)
        return obj
    except TypeError:
        return [obj]


class spotGymEnv(gym.Env):
    """The gym environment for spot.

  It simulates the locomotion of spot, a quadruped robot. The state space
  include the angles, velocities and torques for all the motors and the action
  space is the desired motor angle for each motor. The reward function is based
  on how far spot walks in 1000 steps and penalizes the energy
  expenditure.

  """
    metadata = {
        "render.modes": ["human", "rgb_array"],
        "video.frames_per_second": 50
    }

    def __init__(self,
                 distance_weight=1.0,
                 rotation_weight=1.0,
                 energy_weight=0.0005,
                 shake_weight=0.005,
                 drift_weight=2.0,
                 rp_weight=0.1,
                 rate_weight=0.1,
                 urdf_root=pybullet_data.getDataPath(),
                 urdf_version=None,
                 distance_limit=float("inf"),
                 observation_noise_stdev=SENSOR_NOISE_STDDEV,
                 self_collision_enabled=True,
                 motor_velocity_limit=np.inf,
                 pd_control_enabled=False,
                 leg_model_enabled=False,
                 accurate_motor_model_enabled=False,
                 remove_default_joint_damping=False,
                 motor_kp=2.0,
                 motor_kd=0.03,
                 control_latency=0.0,
                 pd_latency=0.0,
                 torque_control_enabled=False,
                 motor_overheat_protection=False,
                 hard_reset=False,
                 on_rack=False,
                 render=True,
                 num_steps_to_log=1000,
                 action_repeat=1,
                 control_time_step=None,
                 env_randomizer=SpotEnvRandomizer(),
                 forward_reward_cap=float("inf"),
                 reflection=True,
                 log_path=None,
                 desired_velocity=0.5,
                 desired_rate=0.0,
                 lateral=False,
                 draw_foot_path=False,
                 height_field=False,
                 height_field_iters=2,
                 AutoStepper=False,
                 contacts=True):
        """Initialize the spot gym environment.

    Args:
      urdf_root: The path to the urdf data folder.
      urdf_version: [DEFAULT_URDF_VERSION] are allowable
        versions. If None, DEFAULT_URDF_VERSION is used.
      distance_weight: The weight of the distance term in the reward.
      energy_weight: The weight of the energy term in the reward.
      shake_weight: The weight of the vertical shakiness term in the reward.
      drift_weight: The weight of the sideways drift term in the reward.
      distance_limit: The maximum distance to terminate the episode.
      observation_noise_stdev: The standard deviation of observation noise.
      self_collision_enabled: Whether to enable self collision in the sim.
      motor_velocity_limit: The velocity limit of each motor.
      pd_control_enabled: Whether to use PD controller for each motor.
      leg_model_enabled: Whether to use a leg motor to reparameterize the action
        space.
      accurate_motor_model_enabled: Whether to use the accurate DC motor model.
      remove_default_joint_damping: Whether to remove the default joint damping.
      motor_kp: proportional gain for the accurate motor model.
      motor_kd: derivative gain for the accurate motor model.
      control_latency: It is the delay in the controller between when an
        observation is made at some point, and when that reading is reported
        back to the Neural Network.
      pd_latency: latency of the PD controller loop. PD calculates PWM based on
        the motor angle and velocity. The latency measures the time between when
        the motor angle and velocity are observed on the microcontroller and
        when the true state happens on the motor. It is typically (0.001-
        0.002s).
      torque_control_enabled: Whether to use the torque control, if set to
        False, pose control will be used.
      motor_overheat_protection: Whether to shutdown the motor that has exerted
        large torque (OVERHEAT_SHUTDOWN_TORQUE) for an extended amount of time
        (OVERHEAT_SHUTDOWN_TIME). See ApplyAction() in spot.py for more
        details.
      hard_reset: Whether to wipe the simulation and load everything when reset
        is called. If set to false, reset just place spot back to start
        position and set its pose to initial configuration.
      on_rack: Whether to place spot on rack. This is only used to debug
        the walking gait. In this mode, spot's base is hanged midair so
        that its walking gait is clearer to visualize.
      render: Whether to render the simulation.
      num_steps_to_log: The max number of control steps in one episode that will
        be logged. If the number of steps is more than num_steps_to_log, the
        environment will still be running, but only first num_steps_to_log will
        be recorded in logging.
      action_repeat: The number of simulation steps before actions are applied.
      control_time_step: The time step between two successive control signals.
      env_randomizer: An instance (or a list) of EnvRandomizer(s). An
        EnvRandomizer may randomize the physical property of spot, change
          the terrrain during reset(), or add perturbation forces during step().
      forward_reward_cap: The maximum value that forward reward is capped at.
        Disabled (Inf) by default.
      log_path: The path to write out logs. For the details of logging, refer to
        spot_logging.proto.
    Raises:
      ValueError: If the urdf_version is not supported.
    """
        # Sense Contacts
        self.contacts = contacts
        # Enable Auto Stepper State Machine
        self.AutoStepper = AutoStepper
        # Enable Rough Terrain or Not
        self.height_field = height_field
        self.draw_foot_path = draw_foot_path
        # DRAWING FEET PATH
        self.prev_feet_path = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        # CONTROL METRICS
        self.desired_velocity = desired_velocity
        self.desired_rate = desired_rate
        self.lateral = lateral

        # Set up logging.
        self._log_path = log_path
        # @TODO fix logging

        # NUM ITERS
        self._time_step = 0.01
        self._action_repeat = action_repeat
        self._num_bullet_solver_iterations = 300
        self.logging = None
        if pd_control_enabled or accurate_motor_model_enabled:
            self._time_step /= NUM_SUBSTEPS
            self._num_bullet_solver_iterations /= NUM_SUBSTEPS
            self._action_repeat *= NUM_SUBSTEPS
        # PD control needs smaller time step for stability.
        if control_time_step is not None:
            self.control_time_step = control_time_step
        else:
            # Get Control Timestep
            self.control_time_step = self._time_step * self._action_repeat
        # TODO: Fix the value of self._num_bullet_solver_iterations.
        self._num_bullet_solver_iterations = int(
            NUM_SIMULATION_ITERATION_STEPS / self._action_repeat)

        # URDF
        self._urdf_root = urdf_root
        self._self_collision_enabled = self_collision_enabled
        self._motor_velocity_limit = motor_velocity_limit
        self._observation = []
        self._true_observation = []
        self._objectives = []
        self._objective_weights = [
            distance_weight, energy_weight, drift_weight, shake_weight
        ]
        self._env_step_counter = 0
        self._num_steps_to_log = num_steps_to_log
        self._is_render = render
        self._last_base_position = [0, 0, 0]
        self._last_base_orientation = [0, 0, 0, 1]
        self._distance_weight = distance_weight
        self._rotation_weight = rotation_weight
        self._energy_weight = energy_weight
        self._drift_weight = drift_weight
        self._shake_weight = shake_weight
        self._rp_weight = rp_weight
        self._rate_weight = rate_weight
        self._distance_limit = distance_limit
        self._observation_noise_stdev = observation_noise_stdev
        self._action_bound = 1
        self._pd_control_enabled = pd_control_enabled
        self._leg_model_enabled = leg_model_enabled
        self._accurate_motor_model_enabled = accurate_motor_model_enabled
        self._remove_default_joint_damping = remove_default_joint_damping
        self._motor_kp = motor_kp
        self._motor_kd = motor_kd
        self._torque_control_enabled = torque_control_enabled
        self._motor_overheat_protection = motor_overheat_protection
        self._on_rack = on_rack
        self._cam_dist = 1.0
        self._cam_yaw = 0
        self._cam_pitch = -30
        self._forward_reward_cap = forward_reward_cap
        self._hard_reset = True
        self._last_frame_time = 0.0
        self._control_latency = control_latency
        self._pd_latency = pd_latency
        self._urdf_version = urdf_version
        self._ground_id = None
        self._reflection = reflection
        self._env_randomizer = env_randomizer
        # @TODO fix logging
        self._episode_proto = None
        if self._is_render:
            self._pybullet_client = bullet_client.BulletClient(
                connection_mode=pybullet.GUI)
        else:
            self._pybullet_client = bullet_client.BulletClient()
        if self._urdf_version is None:
            self._urdf_version = DEFAULT_URDF_VERSION
        self._pybullet_client.setPhysicsEngineParameter(enableConeFriction=0)
        self.seed()
        # Only update after HF has been generated
        self.height_field = False
        self.reset()
        observation_high = (self.spot.GetObservationUpperBound() +
                            OBSERVATION_EPS)
        observation_low = (self.spot.GetObservationLowerBound() -
                           OBSERVATION_EPS)
        action_dim = NUM_MOTORS
        action_high = np.array([self._action_bound] * action_dim)
        self.action_space = spaces.Box(-action_high, action_high)
        self.observation_space = spaces.Box(observation_low, observation_high)
        self.viewer = None
        self._hard_reset = hard_reset  # This assignment need to be after reset()
        self.goal_reached = False
        # Generate HeightField or not
        self.height_field = height_field
        self.hf = HeightField()
        if self.height_field:
            # Do 3x for extra roughness
            for i in range(height_field_iters):
                self.hf._generate_field(self)

    def set_env_randomizer(self, env_randomizer):
        self._env_randomizer = env_randomizer

    def configure(self, args):
        self._args = args

    def reset(self,
              initial_motor_angles=None,
              reset_duration=1.0,
              desired_velocity=None,
              desired_rate=None):
        # Use Autostepper
        if self.AutoStepper:
            self.StateMachine = BezierStepper(dt=self._time_step)
            # Shuffle order of states
            self.StateMachine.reshuffle()

        self._pybullet_client.configureDebugVisualizer(
            self._pybullet_client.COV_ENABLE_RENDERING, 0)
        if self._hard_reset:
            self._pybullet_client.resetSimulation()
            self._pybullet_client.setPhysicsEngineParameter(
                numSolverIterations=int(self._num_bullet_solver_iterations))
            self._pybullet_client.setTimeStep(self._time_step)
            self._ground_id = self._pybullet_client.loadURDF("%s/plane.urdf" %
                                                             self._urdf_root)
            if self._reflection:
                self._pybullet_client.changeVisualShape(
                    self._ground_id, -1, rgbaColor=[1, 1, 1, 0.8])
                self._pybullet_client.configureDebugVisualizer(
                    self._pybullet_client.COV_ENABLE_PLANAR_REFLECTION,
                    self._ground_id)
            self._pybullet_client.setGravity(0, 0, -9.81)
            acc_motor = self._accurate_motor_model_enabled
            motor_protect = self._motor_overheat_protection
            if self._urdf_version not in spot_URDF_VERSION_MAP:
                raise ValueError("%s is not a supported urdf_version." %
                                 self._urdf_version)
            else:
                self.spot = (spot_URDF_VERSION_MAP[self._urdf_version](
                    pybullet_client=self._pybullet_client,
                    action_repeat=self._action_repeat,
                    urdf_root=self._urdf_root,
                    time_step=self._time_step,
                    self_collision_enabled=self._self_collision_enabled,
                    motor_velocity_limit=self._motor_velocity_limit,
                    pd_control_enabled=self._pd_control_enabled,
                    accurate_motor_model_enabled=acc_motor,
                    remove_default_joint_damping=self.
                    _remove_default_joint_damping,
                    motor_kp=self._motor_kp,
                    motor_kd=self._motor_kd,
                    control_latency=self._control_latency,
                    pd_latency=self._pd_latency,
                    observation_noise_stdev=self._observation_noise_stdev,
                    torque_control_enabled=self._torque_control_enabled,
                    motor_overheat_protection=motor_protect,
                    on_rack=self._on_rack,
                    np_random=self.np_random,
                    contacts=self.contacts))
        self.spot.Reset(reload_urdf=False,
                        default_motor_angles=initial_motor_angles,
                        reset_time=reset_duration)

        if self._env_randomizer is not None:
            self._env_randomizer.randomize_env(self)

            # Also update heightfield if wr are wholly randomizing
            if self.height_field:
                self.hf.UpdateHeightField()

        if desired_velocity is not None:
            self.desired_velocity = desired_velocity
        if desired_rate is not None:
            self.desired_rate = desired_rate

        self._pybullet_client.setPhysicsEngineParameter(enableConeFriction=0)
        self._env_step_counter = 0
        self._last_base_position = [0, 0, 0]
        self._last_base_orientation = [0, 0, 0, 1]
        self._objectives = []
        self._pybullet_client.resetDebugVisualizerCamera(
            self._cam_dist, self._cam_yaw, self._cam_pitch, [0, 0, 0])
        self._pybullet_client.configureDebugVisualizer(
            self._pybullet_client.COV_ENABLE_RENDERING, 1)
        return self._get_observation()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _transform_action_to_motor_command(self, action):
        if self._leg_model_enabled:
            for i, action_component in enumerate(action):
                if not (-self._action_bound - ACTION_EPS <= action_component <=
                        self._action_bound + ACTION_EPS):
                    raise ValueError("{}th action {} out of bounds.".format(
                        i, action_component))
            action = self.spot.ConvertFromLegModel(action)
        return action

    def step(self, action):
        """Step forward the simulation, given the action.

    Args:
      action: A list of desired motor angles for eight motors.

    Returns:
      observations: The angles, velocities and torques of all motors.
      reward: The reward for the current state-action pair.
      done: Whether the episode has ended.
      info: A dictionary that stores diagnostic information.

    Raises:
      ValueError: The action dimension is not the same as the number of motors.
      ValueError: The magnitude of actions is out of bounds.
    """
        self._last_base_position = self.spot.GetBasePosition()
        self._last_base_orientation = self.spot.GetBaseOrientation()
        # print("ACTION:")
        # print(action)
        if self._is_render:
            # Sleep, otherwise the computation takes less time than real time,
            # which will make the visualization like a fast-forward video.
            time_spent = time.time() - self._last_frame_time
            self._last_frame_time = time.time()
            time_to_sleep = self.control_time_step - time_spent
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)
            base_pos = self.spot.GetBasePosition()
            # Keep the previous orientation of the camera set by the user.
            [yaw, pitch,
             dist] = self._pybullet_client.getDebugVisualizerCamera()[8:11]
            self._pybullet_client.resetDebugVisualizerCamera(
                dist, yaw, pitch, base_pos)

        action = self._transform_action_to_motor_command(action)
        self.spot.Step(action)
        reward = self._reward()
        done = self._termination()
        self._env_step_counter += 1

        # DRAW FOOT PATH
        if self.draw_foot_path:
            self.DrawFootPath()
        return np.array(self._get_observation()), reward, done, {}

    def render(self, mode="rgb_array", close=False):
        if mode != "rgb_array":
            return np.array([])
        base_pos = self.spot.GetBasePosition()
        view_matrix = self._pybullet_client.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=base_pos,
            distance=self._cam_dist,
            yaw=self._cam_yaw,
            pitch=self._cam_pitch,
            roll=0,
            upAxisIndex=2)
        proj_matrix = self._pybullet_client.computeProjectionMatrixFOV(
            fov=60,
            aspect=float(RENDER_WIDTH) / RENDER_HEIGHT,
            nearVal=0.1,
            farVal=100.0)
        (_, _, px, _, _) = self._pybullet_client.getCameraImage(
            width=RENDER_WIDTH,
            height=RENDER_HEIGHT,
            renderer=self._pybullet_client.ER_BULLET_HARDWARE_OPENGL,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix)
        rgb_array = np.array(px)
        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def DrawFootPath(self):
        # Get Foot Positions
        FL = self._pybullet_client.getLinkState(self.spot.quadruped,
                                                self.spot._foot_id_list[0])[0]
        FR = self._pybullet_client.getLinkState(self.spot.quadruped,
                                                self.spot._foot_id_list[1])[0]
        BL = self._pybullet_client.getLinkState(self.spot.quadruped,
                                                self.spot._foot_id_list[2])[0]
        BR = self._pybullet_client.getLinkState(self.spot.quadruped,
                                                self.spot._foot_id_list[3])[0]

        lifetime = 3.0  # sec
        self._pybullet_client.addUserDebugLine(self.prev_feet_path[0],
                                               FL, [1, 0, 0],
                                               lifeTime=lifetime)
        self._pybullet_client.addUserDebugLine(self.prev_feet_path[1],
                                               FR, [0, 1, 0],
                                               lifeTime=lifetime)
        self._pybullet_client.addUserDebugLine(self.prev_feet_path[2],
                                               BL, [0, 0, 1],
                                               lifeTime=lifetime)
        self._pybullet_client.addUserDebugLine(self.prev_feet_path[3],
                                               BR, [1, 1, 0],
                                               lifeTime=lifetime)

        self.prev_feet_path[0] = FL
        self.prev_feet_path[1] = FR
        self.prev_feet_path[2] = BL
        self.prev_feet_path[3] = BR

    def get_spot_motor_angles(self):
        """Get the spot's motor angles.

    Returns:
      A numpy array of motor angles.
    """
        return np.array(
            self._observation[MOTOR_ANGLE_OBSERVATION_INDEX:
                              MOTOR_ANGLE_OBSERVATION_INDEX + NUM_MOTORS])

    def get_spot_motor_velocities(self):
        """Get the spot's motor velocities.

    Returns:
      A numpy array of motor velocities.
    """
        return np.array(
            self._observation[MOTOR_VELOCITY_OBSERVATION_INDEX:
                              MOTOR_VELOCITY_OBSERVATION_INDEX + NUM_MOTORS])

    def get_spot_motor_torques(self):
        """Get the spot's motor torques.

    Returns:
      A numpy array of motor torques.
    """
        return np.array(
            self._observation[MOTOR_TORQUE_OBSERVATION_INDEX:
                              MOTOR_TORQUE_OBSERVATION_INDEX + NUM_MOTORS])

    def get_spot_base_orientation(self):
        """Get the spot's base orientation, represented by a quaternion.

    Returns:
      A numpy array of spot's orientation.
    """
        return np.array(self._observation[BASE_ORIENTATION_OBSERVATION_INDEX:])

    def is_fallen(self):
        """Decide whether spot has fallen.

    If the up directions between the base and the world is larger (the dot
    product is smaller than 0.85) or the base is very low on the ground
    (the height is smaller than 0.13 meter), spot is considered fallen.

    Returns:
      Boolean value that indicates whether spot has fallen.
    """
        orientation = self.spot.GetBaseOrientation()
        rot_mat = self._pybullet_client.getMatrixFromQuaternion(orientation)
        local_up = rot_mat[6:]
        pos = self.spot.GetBasePosition()
        #  or pos[2] < 0.13
        return (np.dot(np.asarray([0, 0, 1]), np.asarray(local_up)) < 0.55)

    def _termination(self):
        position = self.spot.GetBasePosition()
        distance = math.sqrt(position[0]**2 + position[1]**2)
        return self.is_fallen() or distance > self._distance_limit

    def _reward(self):
        """ NOTE: reward now consists of:
        roll, pitch at desired 0
        acc (y,z) = 0
        FORWARD-BACKWARD: rate(x,y,z) = 0
        --> HIDDEN REWARD: x(+-) velocity reference, not incl. in obs
        SPIN: acc(x) = 0, rate(x,y) = 0, rate (z) = rate reference
        Also include drift, energy vanilla rewards
        """
        current_base_position = self.spot.GetBasePosition()

        # get observation
        obs = self._get_observation()
        # forward_reward = current_base_position[0] - self._last_base_position[0]

        # # POSITIVE FOR FORWARD, NEGATIVE FOR BACKWARD | NOTE: HIDDEN
        # GETTING TWIST IN BODY FRAME
        pos = self.spot.GetBasePosition()
        orn = self.spot.GetBaseOrientation()
        roll, pitch, yaw = self._pybullet_client.getEulerFromQuaternion(
            [orn[0], orn[1], orn[2], orn[3]])
        rpy = LA.RPY(roll, pitch, yaw)
        R, _ = LA.TransToRp(rpy)
        T_wb = LA.RpToTrans(R, np.array([pos[0], pos[1], pos[2]]))
        T_bw = LA.TransInv(T_wb)
        Adj_Tbw = LA.Adjoint(T_bw)

        Vw = np.concatenate(
            (self.spot.prev_ang_twist, self.spot.prev_lin_twist))
        Vb = np.dot(Adj_Tbw, Vw)

        # New Twist in Body Frame
        # POSITIVE FOR FORWARD, NEGATIVE FOR BACKWARD | NOTE: HIDDEN
        fwd_speed = -Vb[3]  # vx
        lat_speed = -Vb[4]  # vy
        # fwd_speed = self.spot.prev_lin_twist[0]
        # lat_speed = self.spot.prev_lin_twist[1]
        # print("FORWARD SPEED: {} \t STATE SPEED: {}".format(
        #     fwd_speed, self.desired_velocity))
        # self.desired_velocity = 0.4

        # Modification for lateral/fwd rewards
        reward_max = 1.0
        # FORWARD
        if not self.lateral:
            # f(x)=-(x-desired))^(2)*((1/desired)^2)+1
            # to make sure that at 0vel there is 0 reawrd.
            # also squishes allowable tolerance
            forward_reward = reward_max * np.exp(
                -(fwd_speed - self.desired_velocity)**2 / (0.1))
        # LATERAL
        else:
            forward_reward = reward_max * np.exp(
                -(lat_speed - self.desired_velocity)**2 / (0.1))

        yaw_rate = obs[4]

        rot_reward = reward_max * np.exp(-(yaw_rate - self.desired_rate)**2 /
                                         (0.1))

        # Make sure that for forward-policy there is the appropriate rotation penalty
        if self.desired_velocity != 0:
            self._rotation_weight = self._rate_weight
            rot_reward = -abs(obs[4])
        elif self.desired_rate != 0:
            forward_reward = 0.0

        # penalty for nonzero roll, pitch
        rp_reward = -(abs(obs[0]) + abs(obs[1]))
        # print("ROLL: {} \t PITCH: {}".format(obs[0], obs[1]))

        # penalty for nonzero acc(z)
        shake_reward = -abs(obs[4])

        # penalty for nonzero rate (x,y,z)
        rate_reward = -(abs(obs[2]) + abs(obs[3]))

        # drift_reward = -abs(current_base_position[1] -
        #                     self._last_base_position[1])

        # this penalizes absolute error, and does not penalize correction
        # NOTE: for side-side, drift reward becomes in x instead
        drift_reward = -abs(current_base_position[1])

        # If Lateral, change drift reward
        if self.lateral:
            drift_reward = -abs(current_base_position[0])

        # shake_reward = -abs(current_base_position[2] -
        #                     self._last_base_position[2])
        self._last_base_position = current_base_position
        energy_reward = -np.abs(
            np.dot(self.spot.GetMotorTorques(),
                   self.spot.GetMotorVelocities())) * self._time_step
        reward = (self._distance_weight * forward_reward +
                  self._rotation_weight * rot_reward +
                  self._energy_weight * energy_reward +
                  self._drift_weight * drift_reward +
                  self._shake_weight * shake_reward +
                  self._rp_weight * rp_reward +
                  self._rate_weight * rate_reward)
        self._objectives.append(
            [forward_reward, energy_reward, drift_reward, shake_reward])
        # print("REWARD: ", reward)
        return reward

    def get_objectives(self):
        return self._objectives

    @property
    def objective_weights(self):
        """Accessor for the weights for all the objectives.

    Returns:
      List of floating points that corresponds to weights for the objectives in
      the order that objectives are stored.
    """
        return self._objective_weights

    def _get_observation(self):
        """Get observation of this environment, including noise and latency.

    spot class maintains a history of true observations. Based on the
    latency, this function will find the observation at the right time,
    interpolate if necessary. Then Gaussian noise is added to this observation
    based on self.observation_noise_stdev.

    Returns:
      The noisy observation with latency.
    """

        self._observation = self.spot.GetObservation()
        return self._observation

    def _get_realistic_observation(self):
        """Get the observations of this environment.

    It includes the angles, velocities, torques and the orientation of the base.

    Returns:
      The observation list. observation[0:8] are motor angles. observation[8:16]
      are motor velocities, observation[16:24] are motor torques.
      observation[24:28] is the orientation of the base, in quaternion form.
    """
        self._observation = self.spot.RealisticObservation()
        return self._observation

    if parse_version(gym.__version__) < parse_version('0.9.6'):
        _render = render
        _reset = reset
        _seed = seed
        _step = step

    def set_time_step(self, control_step, simulation_step=0.001):
        """Sets the time step of the environment.

    Args:
      control_step: The time period (in seconds) between two adjacent control
        actions are applied.
      simulation_step: The simulation time step in PyBullet. By default, the
        simulation step is 0.001s, which is a good trade-off between simulation
        speed and accuracy.
    Raises:
      ValueError: If the control step is smaller than the simulation step.
    """
        if control_step < simulation_step:
            raise ValueError(
                "Control step should be larger than or equal to simulation step."
            )
        self.control_time_step = control_step
        self._time_step = simulation_step
        self._action_repeat = int(round(control_step / simulation_step))
        self._num_bullet_solver_iterations = (NUM_SIMULATION_ITERATION_STEPS /
                                              self._action_repeat)
        self._pybullet_client.setPhysicsEngineParameter(
            numSolverIterations=self._num_bullet_solver_iterations)
        self._pybullet_client.setTimeStep(self._time_step)
        self.spot.SetTimeSteps(action_repeat=self._action_repeat,
                               simulation_step=self._time_step)

    @property
    def pybullet_client(self):
        return self._pybullet_client

    @property
    def ground_id(self):
        return self._ground_id

    @ground_id.setter
    def ground_id(self, new_ground_id):
        self._ground_id = new_ground_id

    @property
    def env_step_counter(self):
        return self._env_step_counter
