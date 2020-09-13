#!/usr/bin/env python

import numpy as np
import sys
import os
import argparse
import pickle
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
from scipy.stats import norm
sns.set()

# ARGUMENTS
descr = "Spot Mini Mini ARS Agent Evaluator."
parser = argparse.ArgumentParser(description=descr)
parser.add_argument("-nep",
                    "--NumberOfEpisodes",
                    help="Number of Episodes to Plot Data For")
parser.add_argument("-tr",
                    "--TrainingData",
                    help="Plot Training Curve instead of Survival Curve",
                    action='store_true')
parser.add_argument("-tot",
                    "--TotalReward",
                    help="Show Total Reward instead of Reward Per Timestep",
                    action='store_true')
parser.add_argument("-ar",
                    "--RandAgentNum",
                    help="Randomized Agent Number To Load")
parser.add_argument("-anor",
                    "--NoRandAgentNum",
                    help="Non-Randomized Agent Number To Load")
ARGS = parser.parse_args()


def main():
    """ The main() function. """
    file_name = "spot_ars_"

    # Find abs path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    results_path = os.path.join(my_path, "../results")

    if not os.path.exists(results_path):
        os.makedirs(results_path)

    vanilla_surv = np.random.randn(1000)
    agent_surv = np.random.randn(1000)

    nep = 1000

    if ARGS.NumberOfEpisodes:
        nep = ARGS.NumberOfEpisodes
    if ARGS.TrainingData:
        training = True
    else:
        training = False
    rand_agt = 0
    norand_agt = 0
    if ARGS.RandAgentNum:
        rand_agt = ARGS.RandAgentNum
    if ARGS.NoRandAgentNum:
        norand_agt = ARGS.NoRandAgentNum

    if not training:

        # Vanilla Data
        if os.path.exists(results_path + "/" + str(file_name) + "vanilla" +
                          '_survival_' + str(nep)):
            with open(
                    results_path + "/" + str(file_name) + "vanilla" +
                    '_survival_' + str(nep), 'rb') as filehandle:
                vanilla_surv = pickle.load(filehandle)

        # Rand Agent Data
        if os.path.exists(results_path + "/" + str(file_name) + "agent_" +
                          str(rand_agt) + '_survival_' + str(nep)):
            with open(
                    results_path + "/" + str(file_name) + "agent_" +
                    str(rand_agt) + '_survival_' + str(nep), 'rb') as filehandle:
                rand_agent_surv = pickle.load(filehandle)

        # NoRand Agent Data
        if os.path.exists(results_path + "/" + str(file_name) + "agent_" +
                          str(norand_agt) + '_survival_' + str(nep)):
            with open(
                    results_path + "/" + str(file_name) + "agent_" +
                    str(norand_agt) + '_survival_' + str(nep), 'rb') as filehandle:
                norand_agent_surv = pickle.load(filehandle)

        # convert the lists to series
        data = {'Vanilla': vanilla_surv, 'GMBC Rand': rand_agent_surv, 'GMBC NoRand': norand_agent_surv}

        colors = ['b', 'r', 'g']

        # get dataframe
        df = pd.DataFrame(data)
        print(df)

        # Plot
        for i, col in enumerate(df.columns):
            sns.distplot(df[[col]], color=colors[i])

        plt.legend(labels=['Bezier', 'GMBC Rand', 'GMBC NoRand'])
        plt.xlabel("Survived Distance (x)")
        plt.ylabel("Kernel Density Estimate")
        plt.show()

    else:
        # Training Data Plotter
        rand_data = np.load(results_path + "/spot_ars_rand.npy")
        norand_data = np.load(results_path + "/spot_ars_norand.npy")

        plt.plot()
        if ARGS.TotalReward:
            plt.plot(rand_data[:, 0], label="Randomized (Total Reward)")
            plt.plot(norand_data[:, 0], label="Non-Randomized (Total Reward)")
        else:
            plt.plot(rand_data[:, 1], label="Randomized (Reward/dt)")
            plt.plot(norand_data[:, 1], label="Non-Randomized (Reward/dt)")
        plt.xlabel("Epoch #")
        plt.ylabel("Reward")
        plt.title("Training Performance")
        plt.legend()
        plt.show()



if __name__ == '__main__':
    main()
