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
from spotmicro.spot_env_randomizer import SpotEnvRandomizer

import os

import argparse

# ARGUMENTS
descr = "Spot Mini Mini ARS Agent Evaluator."
parser = argparse.ArgumentParser(description=descr)
parser.add_argument("-hf",
                    "--HeightField",
                    help="Use HeightField",
                    action='store_true')
parser.add_argument("-nr",
                    "--DontRender",
                    help="Don't Render environment",
                    action='store_true')
parser.add_argument("-ren",
                    "--DebugRack",
                    help="Put Spot on an Elevated Rack",
                    action='store_true')
parser.add_argument("-p",
                    "--DebugPath",
                    help="Draw Spot's Foot Path",
                    action='store_true')
parser.add_argument("-gui",
                    "--GUI",
                    help="Control The Robot Yourself With a GUI",
                    action='store_true')
parser.add_argument("-nc",
                    "--NoContactSensing",
                    help="Disable Contact Sensing",
                    action='store_true')
parser.add_argument("-a",
                    "--AgentNum",
                    help="Agent Number To Load")
parser.add_argument("-dr",
                    "--DontRandomize",
                    help="Do NOT Randomize State and Environment.",
                    action='store_true')
ARGS = parser.parse_args()


def main():
    """ The main() function. """

    print("STARTING MINITAUR ARS")

    # TRAINING PARAMETERS
    # env_name = "MinitaurBulletEnv-v0"
    seed = 0
    max_timesteps = 4e6
    file_name = "spot_ars_"

    if ARGS.DebugRack:
        on_rack = True
    else:
        on_rack = False

    if ARGS.DebugPath:
        draw_foot_path = True
    else:
        draw_foot_path = False

    if ARGS.HeightField:
        height_field = True
    else:
        height_field = False

    if ARGS.NoContactSensing:
        contacts = False
    else:
        contacts = True

    if ARGS.DontRender:
        render = False
    else:
        render = True

    if ARGS.DontRandomize:
        env_randomizer = None
    else:
        env_randomizer = SpotEnvRandomizer()

    # Find abs path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    results_path = os.path.join(my_path, "../results")
    if contacts:
        models_path = os.path.join(my_path, "../models/contact")
    else:
        models_path = os.path.join(my_path, "../models/no_contact")

    if not os.path.exists(results_path):
        os.makedirs(results_path)

    if not os.path.exists(models_path):
        os.makedirs(models_path)

    env = spotBezierEnv(render=render,
                        on_rack=on_rack,
                        height_field=height_field,
                        draw_foot_path=draw_foot_path,
                        contacts=contacts,
                        env_randomizer=env_randomizer)

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
    policy = Policy(state_dim, action_dim)

    # to GUI or not to GUI
    if ARGS.GUI:
        gui = True
    else:
        gui = False

    # Initialize Agent with normalizer, policy and gym env
    agent = ARSAgent(normalizer, policy, env, bz_step, bzg, spot, gui)
    agent_num = 0
    if ARGS.AgentNum:
        agent_num = ARGS.AgentNum
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

    t = 0
    while t < (int(max_timesteps)):

        episode_reward, episode_timesteps = agent.deployTG()

        t += episode_timesteps
        # episode_reward = agent.train()
        # +1 to account for 0 indexing.
        # +0 on ep_timesteps since it will increment +1 even if done=True
        print("Total T: {} Episode Num: {} Episode T: {} Reward: {}".format(
            t, episode_num, episode_timesteps, episode_reward))
        episode_num += 1

    env.close()


if __name__ == '__main__':
    main()
