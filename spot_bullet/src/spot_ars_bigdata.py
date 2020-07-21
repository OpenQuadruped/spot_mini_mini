#!/usr/bin/env python

import numpy as np

import sys

sys.path.append('../../')

from ars_lib.ars import ARSAgent, Normalizer, Policy
from spotmicro.util.gui import GUI
from spotmicro.Kinematics.SpotKinematics import SpotModel
from spotmicro.GaitGenerator.Bezier import BezierGait
from spotmicro.OpenLoopSM.SpotOL import BezierStepper
from spotmicro.GymEnvs.spot_bezier_env import spotBezierEnv

import os

import argparse

import pickle
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
sns.set()

# ARGUMENTS
descr = "Spot Mini Mini ARS Agent Evaluator."
parser = argparse.ArgumentParser(description=descr)
parser.add_argument("-hf",
                    "--HeightField",
                    help="Use HeightField",
                    action='store_true')
parser.add_argument("-a", "--AgentNum", help="Agent Number To Load")

parser.add_argument("-nep",
                    "--NumberOfEpisodes",
                    help="Number of Episodes to Collect Data For")
ARGS = parser.parse_args()


def main():
    """ The main() function. """

    print("STARTING MINITAUR ARS")

    # TRAINING PARAMETERS
    # env_name = "MinitaurBulletEnv-v0"
    seed = 0
    max_episodes = 10000
    if ARGS.NumberOfEpisodes:
        max_episodes = ARGS.NumberOfEpisodes
    file_name = "spot_ars_"

    # Find abs path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    results_path = os.path.join(my_path, "../results")
    models_path = os.path.join(my_path, "../models")

    if not os.path.exists(results_path):
        os.makedirs(results_path)

    if not os.path.exists(models_path):
        os.makedirs(models_path)

    if ARGS.HeightField:
        height_field = True
    else:
        height_field = False

    env = spotBezierEnv(render=False,
                        on_rack=False,
                        height_field=height_field,
                        draw_foot_path=False)

    # Set seeds
    env.seed(seed)
    np.random.seed(seed)

    state_dim = env.observation_space.shape[0]
    print("STATE DIM: {}".format(state_dim))
    action_dim = env.action_space.shape[0]
    print("ACTION DIM: {}".format(action_dim))
    max_action = float(env.action_space.high[0])

    env.reset()

    spot = SpotModel()

    bz_step = BezierStepper(dt=env._time_step)
    bzg = BezierGait(dt=env._time_step)

    # Initialize Normalizer
    normalizer = Normalizer(state_dim)

    # Initialize Policy
    policy = Policy(state_dim, action_dim, episode_steps=np.inf)

    # Initialize Agent with normalizer, policy and gym env
    agent = ARSAgent(normalizer, policy, env, bz_step, bzg, spot, False)
    use_agent = False
    agent_num = 0
    if ARGS.AgentNum:
        agent_num = ARGS.AgentNum
        use_agent = True
    if os.path.exists(models_path + "/" + file_name + str(agent_num) +
                      "_policy"):
        print("Loading Existing agent")
        agent.load(models_path + "/" + file_name + str(agent_num))
        agent.policy.episode_steps = np.inf
        policy = agent.policy

    env.reset()
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0

    print("STARTED MINITAUR TEST SCRIPT")

    # Used to create gaussian distribution of survival
    survived_timesteps = dict()

    while episode_num < (int(max_episodes)):

        episode_reward, episode_timesteps = agent.deployTG()
        episode_num += 1

        # If ep_dt contains duplicate values, increase frequency
        # Store dt and frequency for prob distribution
        survived_timesteps[episode_timesteps] = survived_timesteps.get(
            episode_timesteps, 0) + 1

        print("Episode Num: {} Episode T: {} Reward: {}".format(
            episode_num, episode_timesteps, episode_reward))

    env.close()
    print("---------------------------------------")

    # Store results
    if use_agent:
        # Store _agent
        agt = "_agent"
    else:
        # Store _vanilla
        agt = "_vanilla"

    with open(
            results_path + "/" + str(file_name) + agt + '_survival_' +
            str(max_episodes), 'wb') as filehandle:
        pickle.dump(survived_timesteps, filehandle)

    # Plot
    df = pd.DataFrame(survived_timesteps, index=[0])
    print(df)
    ax = sns.distplot(df,
                      bins=100,
                      kde=True,
                      color='skyblue',
                      hist_kws={
                          "linewidth": 15,
                          'alpha': 1
                      })
    ax.set(xlabel='Survival Timestep', ylabel='Frequency')


if __name__ == '__main__':
    main()
