#!/usr/bin/env python

import numpy as np

from ppo_lib.ppo import PPO, ActorCritic, NUM_ENVS, HIDDEN_DIM, device
from mini_bullet.minitaur_gym_env import MinitaurBulletEnv
from multi_proc.multiproc import SubprocVecEnv
import gym

import torch
import os


def make_env():
    # returns a function which creates a single environment
    def _thunk():
        env = gym.make("MinitaurBulletEnv-v999")
        return env
    return _thunk


def main():
    """ The main() function. """

    print("STARTING MINITAUR PPO")

    # TRAINING PARAMETERS
    # env_name = "MinitaurBulletEnv-v0"
    seed = 0
    max_timesteps = 4e6
    eval_freq = 1e2
    save_model = True
    file_name = "mini_ppo_"

    # Find abs path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    results_path = os.path.join(my_path, "../results")
    models_path = os.path.join(my_path, "../models")

    if not os.path.exists(results_path):
        os.makedirs(results_path)

    if not os.path.exists(models_path):
        os.makedirs(models_path)

    env = MinitaurBulletEnv(render=False)

    # Set seeds
    env.seed(seed)
    torch.manual_seed(seed)
    np.random.seed(seed)

    state_dim = env.observation_space.shape[0]
    print("STATE DIM: {}".format(state_dim))
    action_dim = env.action_space.shape[0]
    print("ACTION DIM: {}".format(action_dim))
    max_action = float(env.action_space.high[0])

    # Prepare environments
    # returns one state per environment
    envs = [make_env() for i in range(NUM_ENVS)]
    envs = SubprocVecEnv(envs)

    ac = ActorCritic(state_dim, action_dim, HIDDEN_DIM).to(device)
    agent = PPO(ac=ac)

    print("RECORDED MAX ACTION: {}".format(max_action))

    agent_num = 0

    if os.path.exists(models_path + "/" + file_name + str(agent_num) +
                      "_policy"):
        print("Loading Existing agent")
        agent.load(models_path + "/" + file_name + str(agent_num))

    # Evaluate untrained agent and init list for storage
    evaluations = []

    env.reset()
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0

    # Reset all environments
    state = envs.reset()

    print("STARTED MINITAUR PPO")

    t = 0
    while t < (int(max_timesteps)):

        # Maximum timesteps per rollout
        t += 500

        episode_timesteps += 1

        state = agent.train(state, envs)
        episode_reward = agent.deploy(env)
        # episode_reward = agent.train()
        # +1 to account for 0 indexing.
        # +0 on ep_timesteps since it will increment +1 even if done=True
        print("Episode Num: {} Reward: {}".format(
            episode_num, episode_reward))
        # Reset environment
        evaluations.append(episode_reward)
        episode_reward = 0
        episode_timesteps = 0

        # Evaluate episode
        if (episode_num + 1) % eval_freq == 0:
            # evaluate_agent(agent, env_name, seed,
            np.save(results_path + "/" + str(file_name), evaluations)
            if save_model:
                agent.save(models_path + "/" + str(file_name) +
                           str(episode_num))
                # replay_buffer.save(t)

        episode_num += 1


if __name__ == '__main__':
    main()
