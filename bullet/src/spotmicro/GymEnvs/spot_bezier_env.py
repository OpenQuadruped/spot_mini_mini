""" This file implements the gym environment of SpotMicro with Bezier Curve.
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
from spotmicro.OpenLoopSM.SpotOL import BezierStepper
from spotmicro.spot_gym_env import spotGymEnv
import spotmicro.Kinematics.LieAlgebra as LA

SENSOR_NOISE_STDDEV = spot.SENSOR_NOISE_STDDEV

# Register as OpenAI Gym Environment
register(
    id="SpotMicroEnv-v1",
    entry_point='spotmicro.GymEnvs.spot_bezier_env:spotBezierEnv',
    max_episode_steps=1000,
)


class spotBezierEnv(spotGymEnv):
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
                 rotation_weight=0.5,
                 energy_weight=0.0005,
                 shake_weight=0.00,
                 drift_weight=2.0,
                 rp_weight=5.0,
                 rate_weight=0.0005,
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
                 env_randomizer=None,
                 forward_reward_cap=float("inf"),
                 reflection=True,
                 log_path=None,
                 desired_velocity=0.5,
                 desired_rate=0.0,
                 lateral=False,
                 draw_foot_path=False,
                 height_field=False,
                 AutoStepper=True,
                 action_dim=14):

        super(spotBezierEnv, self).__init__(
            distance_weight=distance_weight,
            rotation_weight=rotation_weight,
            energy_weight=energy_weight,
            shake_weight=shake_weight,
            drift_weight=drift_weight,
            rp_weight=rp_weight,
            rate_weight=rate_weight,
            urdf_root=urdf_root,
            urdf_version=urdf_version,
            distance_limit=distance_limit,
            observation_noise_stdev=observation_noise_stdev,
            self_collision_enabled=self_collision_enabled,
            motor_velocity_limit=motor_velocity_limit,
            pd_control_enabled=pd_control_enabled,
            leg_model_enabled=leg_model_enabled,
            accurate_motor_model_enabled=accurate_motor_model_enabled,
            remove_default_joint_damping=remove_default_joint_damping,
            motor_kp=motor_kp,
            motor_kd=motor_kd,
            control_latency=control_latency,
            pd_latency=pd_latency,
            torque_control_enabled=torque_control_enabled,
            motor_overheat_protection=motor_overheat_protection,
            hard_reset=hard_reset,
            on_rack=on_rack,
            render=render,
            num_steps_to_log=num_steps_to_log,
            action_repeat=action_repeat,
            control_time_step=control_time_step,
            env_randomizer=env_randomizer,
            forward_reward_cap=forward_reward_cap,
            reflection=reflection,
            log_path=log_path,
            desired_velocity=desired_velocity,
            desired_rate=desired_rate,
            lateral=lateral,
            draw_foot_path=draw_foot_path,
            height_field=height_field,
            AutoStepper=AutoStepper)

        # Residuals + Clearance Height + Penetration Depth
        action_high = np.array([self._action_bound] * action_dim)
        self.action_space = spaces.Box(-action_high, action_high)
        print("Action SPACE: {}".format(self.action_space))

        self.prev_pos = np.array([0.0, 0.0, 0.0])

    def pass_joint_angles(self, ja):
        """ For executing joint angles
        """
        self.ja = ja

    def step(self, action):
        """Step forward the simulation, given the action.

    Args:
      action: A list of desired motor angles for eight motors.
      smach: the bezier state machine containing simulated
             random controll inputs

    Returns:
      observations: The angles, velocities and torques of all motors.
      reward: The reward for the current state-action pair.
      done: Whether the episode has ended.
      info: A dictionary that stores diagnostic information.

    Raises:
      ValueError: The action dimension is not the same as the number of motors.
      ValueError: The magnitude of actions is out of bounds.
    """
        # Discard all but joint angles
        action = self.ja

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
        # NOTE: SMACH is passed to the reward method
        reward = self._reward()
        done = self._termination()
        self._env_step_counter += 1

        # DRAW FOOT PATH
        if self.draw_foot_path:
            self.DrawFootPath()
        return np.array(self._get_observation()), reward, done, {}

    def return_state(self):
        return np.array(self._get_observation())

    def _reward(self):
        # get observation
        obs = self._get_observation()

        # Return StepVelocity with the sign of StepLength
        DesiredVelicty = math.copysign(self.spot.StepVelocity / 4.0,
                                       self.spot.StepLength)

        fwd_speed = self.spot.prev_lin_twist[0]  # vx
        lat_speed = self.spot.prev_lin_twist[1]  # vy

        # DEBUG
        lt, at = self.spot.GetBaseTwist()

        # Modification for lateral/fwd rewards
        reward_max = 1.0
        # FORWARD/LATERAL
        # forward_reward = reward_max * np.exp(
        #     -(fwd_speed - DesiredVelicty * np.cos(self.spot.LateralFraction))**
        #     2 / (0.1))
        # lateral_reward = reward_max * np.exp(
        #     -(lat_speed - DesiredVelicty * np.sin(self.spot.LateralFraction))**
        #     2 / (0.1))
        # NOTE: Replaceing speed-based reward with movement-based reward
        # ONLY WORKS FOR MOVING PURELY FORWARD
        pos = self.spot.GetBasePosition()
        if DesiredVelicty > 0:
            forward_reward = pos[0] - self.prev_pos[0]
        else:
            forward_reward = -(pos[0] - self.prev_pos[0])

        lateral_reward = pos[1] - self.prev_pos[1]

        self.prev_pos = pos

        lateral_reward = np.sin(
            self.spot.LateralFraction) * lateral_reward * 300
        forward_reward = np.cos(
            self.spot.LateralFraction) * forward_reward * 300

        # print("FWD RWD: {:.2f}".format(forward_reward))
        # print("LAT RWD: {:.2f}".format(lateral_reward))

        # print("FWD SPEED: {:.2f} \t UNMOD: {:.2f} ".format(fwd_speed, lt[0]))

        # print("LAT SPEED: {:.2f} \t UNMOD: {:.2f} ".format(lat_speed, lt[1]))

        # print("ROLL: {:.2f}".format(obs[0]))
        # print("PITCH: {:.2f}".format(obs[1]))

        # print("GYRO X: {:.2f}".format(obs[2]))
        # print("GYRO Y: {:.2f}".format(obs[3]))
        # print("GYRO Z: {:.2f}".format(obs[4]))

        # print("ACC X: {:.2f}".format(obs[5]))
        # print("ACC Y: {:.2f}".format(obs[6]))
        # print("ACC Z: {:.2f}".format(obs[7]))

        # print("STEP LEN: {:.2f}".format(obs[8]))
        # print("STEP VEL: {:.2f}".format(obs[9]))
        # print("LAT FRAC: {:.2f}".format(obs[10]))
        # print("YAW RATE: {:.2f}".format(obs[11]))
        # print(
        #     "LEG PHASES: \nFL: {:.2f} \tFR: {:.2f} \nBL: {:.2f} \tBR: {:.2f}".
        #     format(obs[12], obs[13], obs[14], obs[15]))

        # print("-----------------------------------------------")

        forward_reward += lateral_reward

        yaw_rate = obs[4]

        # rot_reward = reward_max * np.exp(-(yaw_rate - self.spot.YawRate)**2 /
        #                                  (0.1))

        rot_reward = -abs(yaw_rate)

        # penalty for nonzero PITCH ONLY
        rp_reward = -(abs(obs[1]))
        # print("RP RWD: {:.2f}".format(rp_reward))
        # print("ROLL: {} \t PITCH: {}".format(obs[0], obs[1]))

        # penalty for nonzero acc(z) - UNRELIABLE ON IMU
        shake_reward = 0

        # penalty for nonzero rate (x,y,z)
        rate_reward = -(abs(obs[2]) + abs(obs[3]))

        drift_reward = -abs(pos[1])
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