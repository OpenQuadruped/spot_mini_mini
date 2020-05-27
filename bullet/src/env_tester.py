#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import copy

from spotmicro.spot_gym_env import spotGymEnv
from spotmicro.util.gui import GUI
from spotmicro.Kinematics.SpotKinematics import SpotModel
from spotmicro.Kinematics.LieAlgebra import RPY
from spotmicro.GaitGenerator.Bezier import BezierGait
from mini_bullet.terrain_env_randomizer import MinitaurTerrainRandomizer
import time

import torch
import os


def main():
    """ The main() function. """

    print("STARTING SPOT TEST ENV")
    seed = 0
    max_timesteps = 4e6
    file_name = "spot_ars_"

    # Find abs path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    results_path = os.path.join(my_path, "../results")
    models_path = os.path.join(my_path, "../models")

    if not os.path.exists(results_path):
        os.makedirs(results_path)

    if not os.path.exists(models_path):
        os.makedirs(models_path)

    env = spotGymEnv(render=True, on_rack=False)

    # Set seeds
    env.seed(seed)
    torch.manual_seed(seed)
    np.random.seed(seed)

    state_dim = env.observation_space.shape[0]
    print("STATE DIM: {}".format(state_dim))
    action_dim = env.action_space.shape[0]
    print("ACTION DIM: {}".format(action_dim))
    max_action = float(env.action_space.high[0])

    env.reset()

    g_u_i = GUI(env.spot.quadruped)

    spot = SpotModel()
    T_bf0 = spot.WorldToFoot
    T_bf = copy.deepcopy(T_bf0)

    bzg = BezierGait(dt=env._time_step)

    trand = MinitaurTerrainRandomizer()
    trand._generate_convex_blocks(env)

    print("STARTED SPOT TEST ENV")
    t = 0
    while t < (int(max_timesteps)):

        # GUI: x, y, z | r, p , y
        pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth = g_u_i.UserInput(
        )

        # Get Desired Foot Poses
        T_bf = bzg.GenerateTrajectory(StepLength, LateralFraction, YawRate,
                                      StepVelocity, T_bf0, T_bf,
                                      ClearanceHeight, PenetrationDepth)
        joint_angles = spot.IK(orn, pos, T_bf)
        action = joint_angles.reshape(-1)
        # action = env.action_space.sample()
        next_state, reward, done, _ = env.step(action)
        if done:
            print("DONE")

        # time.sleep(1.0)

        t += 1
    env.close()
    print(joint_angles)


if __name__ == '__main__':
    main()
