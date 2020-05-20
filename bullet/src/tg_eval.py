#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

from ars_lib.ars import ARSAgent, Normalizer, Policy, ParallelWorker
from mini_bullet.minitaur_gym_env import MinitaurBulletEnv

from tg_lib.tg_policy import TGPolicy
import time

import torch
import os


def main():
    """ The main() function. """

    print("STARTING MINITAUR ARS")

    # TRAINING PARAMETERS
    # env_name = "MinitaurBulletEnv-v0"
    seed = 0
    max_timesteps = 1e6
    file_name = "mini_tg_ars_"

    # Find abs path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    results_path = os.path.join(my_path, "../results")
    models_path = os.path.join(my_path, "../models")

    if not os.path.exists(results_path):
        os.makedirs(results_path)

    if not os.path.exists(models_path):
        os.makedirs(models_path)

    env = MinitaurBulletEnv(render=True, on_rack=False)

    dt = env._time_step

    # TRAJECTORY GENERATOR
    movetype = "walk"
    # movetype = "trot"
    # movetype = "bound"
    # movetype = "pace"
    # movetype = "pronk"
    TG = TGPolicy(movetype=movetype,
                  center_swing=0.0,
                  amplitude_extension=0.2,
                  amplitude_lift=0.4)
    TG_state_dim = len(TG.get_TG_state())
    TG_action_dim = 5  # f_tg, Beta, alpha_tg, h_tg, intensity
    state_dim = env.observation_space.shape[0] + TG_state_dim
    print("STATE DIM: {}".format(state_dim))
    action_dim = env.action_space.shape[0] + TG_action_dim
    print("ACTION DIM: {}".format(action_dim))
    max_action = float(env.action_space.high[0])

    print("RECORDED MAX ACTION: {}".format(max_action))

    # Initialize Normalizer
    normalizer = Normalizer(state_dim)

    # Initialize Policy
    policy = Policy(state_dim, action_dim)

    # Initialize Agent with normalizer, policy and gym env
    agent = ARSAgent(normalizer, policy, env, TGP=TG)
    agent_num = raw_input("Policy Number: ")
    if os.path.exists(models_path + "/" + file_name + str(agent_num) +
                      "_policy"):
        print("Loading Existing agent")
        agent.load(models_path + "/" + file_name + str(agent_num))
        agent.policy.episode_steps = 1000
        policy = agent.policy

    # Set seeds
    env.seed(seed)
    torch.manual_seed(seed)
    np.random.seed(seed)

    env.reset()
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0

    print("STARTED MINITAUR TEST SCRIPT")

    # Just to store correct action space
    action = env.action_space.sample()

    # Record extends for plot
    # LF_ext = []
    # LB_ext = []
    # RF_ext = []
    # RB_ext = []

    LF_tp = []
    LB_tp = []
    RF_tp = []
    RB_tp = []

    t = 0
    while t < (int(max_timesteps)):
        action[:] = 0.0

        # # Get Action from TG [no policies here]
        # action = TG.get_utg(action, alpha_tg, h_tg, intensity,
        #                     env.minitaur.num_motors)

        # LF_ext.append(action[env.minitaur.num_motors / 2])
        # LB_ext.append(action[1 + env.minitaur.num_motors / 2])
        # RF_ext.append(action[2 + env.minitaur.num_motors / 2])
        # RB_ext.append(action[3 + env.minitaur.num_motors / 2])
        # # Perform action
        # next_state, reward, done, _ = env.step(action)

        obs = agent.TGP.get_TG_state()
        # LF_tp.append(obs[0])
        # LB_tp.append(obs[1])
        # RF_tp.append(obs[2])
        # RB_tp.append(obs[3])

        # # Increment phase
        # TG.increment(dt, f_tg, Beta)

        # # time.sleep(1.0)

        # t += 1

        # Maximum timesteps per rollout
        t += policy.episode_steps

        episode_timesteps += 1

        episode_reward = agent.deployTG()
        # episode_reward = agent.train()
        # +1 to account for 0 indexing.
        # +0 on ep_timesteps since it will increment +1 even if done=True
        print("Total T: {} Episode Num: {} Episode T: {} Reward: {}".format(
            t, episode_num, policy.episode_steps, episode_reward))
        # Reset environment
        episode_reward = 0
        episode_timesteps = 0
        episode_num += 1

    plt.plot(0)
    plt.plot(LF_tp, label="LF")
    plt.plot(LB_tp, label="LB")
    plt.plot(RF_tp, label="RF")
    plt.plot(RB_tp, label="RB")
    plt.xlabel("t")
    plt.ylabel("EXT")
    plt.title("Leg Extensions")
    plt.legend()
    plt.show()

    env.close()


if __name__ == '__main__':
    main()
