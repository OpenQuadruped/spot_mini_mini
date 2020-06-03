#!/usr/bin/env python

import numpy as np

from sac_lib import SoftActorCritic, NormalizedActions, ReplayBuffer, PolicyNetwork
import copy
from gym import spaces

from spotmicro.GymEnvs.spot_bezier_env import spotBezierEnv
from spotmicro.Kinematics.SpotKinematics import SpotModel
from spotmicro.GaitGenerator.Bezier import BezierGait

# TESTING
from spotmicro.OpenLoopSM.SpotOL import BezierStepper

import time

import torch
import os


def main():
    """ The main() function. """

    print("STARTING SPOT SAC")

    # TRAINING PARAMETERS
    seed = 0
    max_timesteps = 4e6
    batch_size = 256
    eval_freq = 1e4
    save_model = True
    file_name = "spot_sac_"

    # Find abs path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    results_path = os.path.join(my_path, "../results")
    models_path = os.path.join(my_path, "../models")

    if not os.path.exists(results_path):
        os.makedirs(results_path)

    if not os.path.exists(models_path):
        os.makedirs(models_path)

    env = spotBezierEnv(render=True,
                        on_rack=False,
                        height_field=True,
                        draw_foot_path=False,
                        action_dim=12)
    env = NormalizedActions(env)

    # Set seeds
    env.seed(seed)
    torch.manual_seed(seed)
    np.random.seed(seed)

    state_dim = env.observation_space.shape[0]
    print("STATE DIM: {}".format(state_dim))
    action_dim = env.action_space.shape[0]
    print("ACTION DIM: {}".format(action_dim))
    max_action = float(env.action_space.high[0])

    print("RECORDED MAX ACTION: {}".format(max_action))

    hidden_dim = 256
    policy = PolicyNetwork(state_dim, action_dim, hidden_dim)

    replay_buffer_size = 1000000
    replay_buffer = ReplayBuffer(replay_buffer_size)

    sac = SoftActorCritic(policy=policy,
                          state_dim=state_dim,
                          action_dim=action_dim,
                          replay_buffer=replay_buffer)

    policy_num = raw_input("Policy Number: ")
    if os.path.exists(models_path + "/" + file_name + str(policy_num) +
                      "_policy_net"):
        print("Loading Existing Policy")
        sac.load(models_path + "/" + file_name + str(policy_num))
        policy = sac.policy_net

    # Evaluate untrained policy and init list for storage
    evaluations = []

    state = env.reset()
    done = False
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0
    max_t_per_ep = 5000

    # State Machine for Random Controller Commands
    bz_step = BezierStepper(dt=0.01, mode=0)

    # Bezier Gait Generator
    bzg = BezierGait(dt=0.01)

    # Spot Model
    spot = SpotModel()
    T_bf0 = spot.WorldToFoot
    T_bf = copy.deepcopy(T_bf0)

    BaseClearanceHeight = bz_step.ClearanceHeight
    BasePenetrationDepth = bz_step.PenetrationDepth

    print("STARTED SPOT SAC")

    for t in range(int(max_timesteps)):

        contacts = state[-4:]

        t += 1

        episode_timesteps += 1
        pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth = bz_step.StateMachine(
        )

        env.spot.GetExternalObservations(bzg, bz_step)

        # Read UPDATED state based on controls and phase
        state = env.return_state()

        action = sac.policy_net.get_action(state)

        # Add DELTA to Bezier Params
        # LIMS
        # SL_SCALE = 0.007
        # SV_SCALE = 0.2
        # LF_SCALE = 0.1
        # Y_SCALE = 0.1
        # CH_SCALE = 0.007
        # PD_SCALE = 0.0025
        # StepLength += np.tanh(action[0]) * SL_SCALE
        # StepVelocity += np.tanh(action[1]) * SV_SCALE
        # LateralFraction += np.tanh(action[2]) * LF_SCALE
        # YawRate += np.tanh(action[3]) * Y_SCALE
        # ClearanceHeight += np.tanh(action[4]) * CH_SCALE
        # PenetrationDepth += np.tanh(action[5]) * PD_SCALE

        # # CLIP EVERYTHING
        # StepLength = np.clip(StepLength, bz_step.StepLength_LIMITS[0],
        #                      bz_step.StepLength_LIMITS[1])
        # StepVelocity = np.clip(StepVelocity, bz_step.StepVelocity_LIMITS[0],
        #                        bz_step.StepVelocity_LIMITS[1])
        # LateralFraction = np.clip(LateralFraction,
        #                           bz_step.LateralFraction_LIMITS[0],
        #                           bz_step.LateralFraction_LIMITS[1])
        # YawRate = np.clip(YawRate, bz_step.YawRate_LIMITS[0],
        #                   bz_step.YawRate_LIMITS[1])
        # ClearanceHeight = np.clip(ClearanceHeight,
        #                           bz_step.ClearanceHeight_LIMITS[0],
        #                           bz_step.ClearanceHeight_LIMITS[1])
        # PenetrationDepth = np.clip(PenetrationDepth,
        #                            bz_step.PenetrationDepth_LIMITS[0],
        #                            bz_step.PenetrationDepth_LIMITS[1])

        # Get Desired Foot Poses
        T_bf = bzg.GenerateTrajectory(StepLength, LateralFraction, YawRate,
                                      StepVelocity, T_bf0, T_bf,
                                      ClearanceHeight, PenetrationDepth,
                                      contacts)
        # Add DELTA to XYZ Foot Poses
        RESIDUALS_SCALE = 0.03
        # T_bf["FL"][3, :3] += action[0:3] * RESIDUALS_SCALE
        # T_bf["FR"][3, :3] += action[3:6] * RESIDUALS_SCALE
        # T_bf["BL"][3, :3] += action[6:9] * RESIDUALS_SCALE
        # T_bf["BR"][3, :3] += action[9:12] * RESIDUALS_SCALE
        # T_bf["FL"][3, 2] += action[0] * RESIDUALS_SCALE
        # T_bf["FR"][3, 2] += action[1] * RESIDUALS_SCALE
        # T_bf["BL"][3, 2] += action[2] * RESIDUALS_SCALE
        # T_bf["BR"][3, 2] += action[3] * RESIDUALS_SCALE

        joint_angles = spot.IK(orn, pos, T_bf)
        # Pass Joint Angles
        env.pass_joint_angles(joint_angles.reshape(-1))

        # Perform action
        state, reward, done, _ = env.step(action)
        episode_reward += reward
        # print("DT REWARD: {}".format(reward))

        if done:
            # +1 to account for 0 indexing.
            # +0 on ep_timesteps since it will increment +1 even if done=True
            print(
                "Total T: {} Episode Num: {} Episode T: {} Reward: {}".format(
                    t + 1, episode_num, episode_timesteps, episode_reward))
            # Reset environment
            state, done = env.reset(), False
            evaluations.append(episode_reward)
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1

    env.close()


if __name__ == '__main__':
    main()