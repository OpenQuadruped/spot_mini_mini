#!/usr/bin/env python

import numpy as np

from ars_lib.ars import ARSAgent, Normalizer, Policy, ParallelWorker
from mini_bullet.minitaur_gym_env import MinitaurBulletEnv

from tg_lib.traj_gen import TrajectoryGenerator, CyclicIntegrator

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

    env = MinitaurBulletEnv(render=True, on_rack=True)

    dt = env._time_step

    TG_dict = {}

    TG_LF = TrajectoryGenerator(dphi_leg=0.0)
    TG_LB = TrajectoryGenerator(dphi_leg=0.0)
    TG_RF = TrajectoryGenerator(dphi_leg=0.0)
    TG_RB = TrajectoryGenerator(dphi_leg=0.0)

    TG_dict["LF"] = TG_LF
    TG_dict["LB"] = TG_LB
    TG_dict["RF"] = TG_RF
    TG_dict["RB"] = TG_RB

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

    # # Initialize Normalizer
    # normalizer = Normalizer(state_dim)

    # # Initialize Policy
    # policy = Policy(state_dim, action_dim)

    # # Initialize Agent with normalizer, policy and gym env
    # agent = ARSAgent(normalizer, policy, env)
    # agent_num = raw_input("Policy Number: ")
    # if os.path.exists(models_path + "/" + file_name + str(agent_num) +
    #                   "_policy"):
    #     print("Loading Existing agent")
    #     agent.load(models_path + "/" + file_name + str(agent_num))
    #     agent.policy.episode_steps = 3000
    #     policy = agent.policy

    env.reset()

    print("STARTED MINITAUR TEST SCRIPT")

    # Just to store correct action space
    action = env.action_space.sample()

    t = 0
    while t < (int(max_timesteps)):

        # Get Action from TG [no policies here]
        half_num_motors = int(env.minitaur.num_motors / 2)
        for i, (key, tg) in enumerate(TG_dict.items()):
            action_idx = i
            swing, extend = tg.get_swing_extend_based_on_phase()
            print("LEG: {} \t SWING: {:.3f} \t EXTEND: {:.3f}".format(
                action_idx, swing, extend))
            action[action_idx] = swing
            action[action_idx + half_num_motors] = extend

        # Perform action
        next_state, reward, done, _ = env.step(action)

        # Increment phase
        for (key, tg) in TG_dict.items():
            tg.CI.progress_tprime(dt, 10.0, 5.0)

    env.close()


if __name__ == '__main__':
    main()
