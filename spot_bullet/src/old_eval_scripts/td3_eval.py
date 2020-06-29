#!/usr/bin/env python

import numpy as np

from td3_lib.td3 import ReplayBuffer, TD3Agent, evaluate_policy

from mini_bullet.minitaur_gym_env import MinitaurBulletEnv

import gym
import torch
import os

import time


def main():
    """ The main() function. """

    print("STARTING MINITAUR TD3")

    # TRAINING PARAMETERS
    # env_name = "MinitaurBulletEnv-v0"
    seed = 0
    max_timesteps = 4e6
    file_name = "mini_td3_"

    # Find abs path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    results_path = os.path.join(my_path, "../results")
    models_path = os.path.join(my_path, "../models")

    if not os.path.exists(results_path):
        os.makedirs(results_path)

    if not os.path.exists(models_path):
        os.makedirs(models_path)

    env = MinitaurBulletEnv(render=True)

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

    policy = TD3Agent(state_dim, action_dim, max_action)
    policy_num = 2239999
    if os.path.exists(models_path + "/" + file_name +
                      str(policy_num) + "_critic"):
        print("Loading Existing Policy")
        policy.load(models_path + "/" + file_name + str(policy_num))

    replay_buffer = ReplayBuffer()
    # Optionally load existing policy, replace 9999 with num
    buffer_number = 0  # BY DEFAULT WILL LOAD NOTHING, CHANGE THIS
    if os.path.exists(replay_buffer.buffer_path + "/" + "replay_buffer_" +
                      str(buffer_number) + '.data'):
        print("Loading Replay Buffer " + str(buffer_number))
        replay_buffer.load(buffer_number)
        print(replay_buffer.storage)

    # Evaluate untrained policy and init list for storage
    evaluations = []

    state = env.reset()
    done = False
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0

    print("STARTED MINITAUR TEST SCRIPT")

    for t in range(int(max_timesteps)):

        episode_timesteps += 1
        # Deterministic Policy Action
        action = np.clip(policy.select_action(np.array(state)),
                         -max_action, max_action)
        # rospy.logdebug("Selected Acton: {}".format(action))

        # Perform action
        next_state, reward, done, _ = env.step(action)

        state = next_state
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


if __name__ == '__main__':
    main()
