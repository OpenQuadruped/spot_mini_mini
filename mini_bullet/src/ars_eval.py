#!/usr/bin/env python

import numpy as np

from ars_lib.ars import ARSAgent, Normalizer, Policy, ParallelWorker
from mini_bullet.minitaur_gym_env import MinitaurBulletEnv

import torch
import os


def main():
    """ The main() function. """

    print("STARTING MINITAUR ARS")

    # TRAINING PARAMETERS
    # env_name = "MinitaurBulletEnv-v0"
    seed = 0
    max_timesteps = 4e6
    file_name = "mini_ars_"

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

    # Initialize Normalizer
    normalizer = Normalizer(state_dim)

    # Initialize Policy
    policy = Policy(state_dim, action_dim)

    # Initialize Agent with normalizer, policy and gym env
    agent = ARSAgent(normalizer, policy, env)
    agent_num = raw_input("Policy Number: ")
    if os.path.exists(models_path + "/" + file_name + str(agent_num) +
                      "_policy"):
        print("Loading Existing agent")
        agent.load(models_path + "/" + file_name + str(agent_num))
        agent.policy.episode_steps = 3000
        policy = agent.policy

    # Evaluate untrained agent and init list for storage
    evaluations = []

    env.reset()
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0

    print("STARTED MINITAUR TEST SCRIPT")

    t = 0
    while t < (int(max_timesteps)):

        # Maximum timesteps per rollout
        t += policy.episode_steps

        episode_timesteps += 1

        episode_reward = agent.deploy()
        # episode_reward = agent.train()
        # +1 to account for 0 indexing.
        # +0 on ep_timesteps since it will increment +1 even if done=True
        print("Total T: {} Episode Num: {} Episode T: {} Reward: {}".format(
            t, episode_num, policy.episode_steps, episode_reward))
        # Reset environment
        evaluations.append(episode_reward)
        episode_reward = 0
        episode_timesteps = 0
        episode_num += 1

    env.close()


if __name__ == '__main__':
    main()
