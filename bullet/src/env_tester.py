#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

from spotmicro.spot_gym_env import spotGymEnv
from spotmicro.util.gui import GUI
import time

import torch
import os


def main():
    """ The main() function. """

    print("STARTING SPOT TEST ENV")
    seed = 0
    max_timesteps = 1e6
    file_name = "spot_ars_"

    # Find abs path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    results_path = os.path.join(my_path, "../results")
    models_path = os.path.join(my_path, "../models")

    if not os.path.exists(results_path):
        os.makedirs(results_path)

    if not os.path.exists(models_path):
        os.makedirs(models_path)

    env = spotGymEnv(render=True, on_rack=True)

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

    g_u_i = GUI()

    print("STARTED SPOT TEST ENV")

    # Just to store correct action space
    action = env.action_space.sample()
    t = 0
    while t < (int(max_timesteps)):
        # time.sleep()
        # if (t % 100 == 0):
        # action = env.action_space.sample()
        # action[:] = 0.0
        # Perform action
        next_state, reward, done, _ = env.step(action)

        # time.sleep(1.0)

        t += 1
    env.close()


if __name__ == '__main__':
    main()
