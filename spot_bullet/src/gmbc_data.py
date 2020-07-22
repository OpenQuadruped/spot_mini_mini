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
ARGS = parser.parse_args()


def main():
    """ The main() function. """
    file_name = "spot_ars_"

    # Find abs path to this file
    my_path = os.path.abspath(os.path.dirname(__file__))
    results_path = os.path.join(my_path, "../results")
    models_path = os.path.join(my_path, "../models")

    if not os.path.exists(results_path):
        os.makedirs(results_path)

    if not os.path.exists(models_path):
        os.makedirs(models_path)

    bezier_surv = np.random.randn(1000)
    agent_surv = np.random.randn(1000)

    nep = 1000

    if ARGS.NumberOfEpisodes:
        nep = ARGS.NumberOfEpisodes

    # Bezier Data
    if os.path.exists(results_path + "/" + str(file_name) + "vanilla" +
                      '_survival_' + str(nep)):
        with open(
                results_path + "/" + str(file_name) + "vanilla" +
                '_survival_' + str(nep), 'rb') as filehandle:
            bezier_surv = pickle.load(filehandle)

    # Agent Data
    if os.path.exists(results_path + "/" + str(file_name) + "agent" +
                      '_survival_' + str(nep)):
        with open(
                results_path + "/" + str(file_name) + "agent" + '_survival_' +
                str(nep), 'rb') as filehandle:
            agent_surv = pickle.load(filehandle)

    # convert the lists to series
    data = {'Bezier': bezier_surv, 'GMBC': agent_surv}

    colors = ['b', 'r']

    # get dataframe
    df = pd.DataFrame(data)
    print(df)

    # Plot
    for i, col in enumerate(df.columns):
        sns.distplot(df[[col]], color=colors[i])

    plt.legend(labels=['Bezier', 'GMBC'])
    plt.xlabel("Survived Timestep")
    plt.ylabel("Kernel Density Estimate")
    plt.show()


if __name__ == '__main__':
    main()
