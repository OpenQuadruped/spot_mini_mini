#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    reward = np.load("../../results/plen_walk_gazebo_.npy")

    plt.figure(0)
    plt.autoscale(enable=True, axis='both', tight=None)
    plt.title('Dominant Foot Trajectories - SS')
    plt.ylabel('positon (mm)')
    plt.xlabel('timestep')
    plt.plot(reward, color='b', label="Reward")

    plt.legend()

    plt.show()
