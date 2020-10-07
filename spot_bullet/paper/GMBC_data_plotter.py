#!/usr/bin/env python

import numpy as np
import sys
import os
import argparse
import pickle
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import copy
from scipy.stats import norm
sns.set()

# ARGUMENTS
descr = "Spot Mini Mini ARS Agent Evaluator."
parser = argparse.ArgumentParser(description=descr)
parser.add_argument("-nep",
                    "--NumberOfEpisodes",
                    help="Number of Episodes to Plot Data For")
parser.add_argument("-maw",
                    "--MovingAverageWindow",
                    help="Moving Average Window for Plotting (Default: 50)")
parser.add_argument("-surv",
                    "--Survival",
                    help="Plot Survival Curve",
                    action='store_true')
parser.add_argument("-tr",
                    "--TrainingData",
                    help="Plot Training Curve",
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
parser.add_argument("-raw",
                    "--Raw",
                    help="Plot Raw Data in addition to Moving Averaged Data",
                    action='store_true')
parser.add_argument(
    "-s",
    "--Seed",
    help="Seed [UP TO, e.g. 0 | 0, 1 | 0, 1, 2 ...] (Default: 0).")
parser.add_argument("-pout",
                    "--PolicyOut",
                    help="Plot Policy Output Data",
                    action='store_true')
parser.add_argument("-rough",
                    "--Rough",
                    help="Plot Policy Output Data for Rough Terrain",
                    action='store_true')
parser.add_argument(
    "-tru",
    "--TrueAct",
    help="Plot the Agent Action instead of what the robot sees",
    action='store_true')
ARGS = parser.parse_args()

MA_WINDOW = 50
if ARGS.MovingAverageWindow:
    MA_WINDOW = int(ARGS.MovingAverageWindow)


def moving_average(a, n=MA_WINDOW):
    MA = np.cumsum(a, dtype=float)
    MA[n:] = MA[n:] - MA[:-n]
    return MA[n - 1:] / n


def main():
    """ The main() function. """
    file_name = "spot_ars_"

    seed = 0
    if ARGS.Seed:
        seed = ARGS.Seed

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
    if ARGS.Survival:
        surv = True
    else:
        surv = False
    if ARGS.PolicyOut or ARGS.Rough or ARGS.TrueAct:
        pout = True
    else:
        pout = False

    if not pout and not surv and not training:
        print(
            "Please Select which Data you would like to plot (-pout | -surv | -tr)"
        )
    rand_agt = 579
    norand_agt = 569
    if ARGS.RandAgentNum:
        rand_agt = ARGS.RandAgentNum
    if ARGS.NoRandAgentNum:
        norand_agt = ARGS.NoRandAgentNum

    if surv:

        # Vanilla Data
        if os.path.exists(results_path + "/" + file_name + "vanilla" +
                          '_survival_{}'.format(nep)):
            with open(
                    results_path + "/" + file_name + "vanilla" +
                    '_survival_{}'.format(nep), 'rb') as filehandle:
                vanilla_surv = np.array(pickle.load(filehandle))

        # Rand Agent Data
        if os.path.exists(results_path + "/" + file_name +
                          "agent_{}".format(rand_agt) +
                          '_survival_{}'.format(nep)):
            with open(
                    results_path + "/" + file_name +
                    "agent_{}".format(rand_agt) + '_survival_{}'.format(nep),
                    'rb') as filehandle:
                rand_agent_surv = np.array(pickle.load(filehandle))

        # NoRand Agent Data
        if os.path.exists(results_path + "/" + file_name +
                          "agent_{}".format(norand_agt) +
                          '_survival_{}'.format(nep)):
            with open(
                    results_path + "/" + file_name +
                    "agent_{}".format(norand_agt) + '_survival_{}'.format(nep),
                    'rb') as filehandle:
                norand_agent_surv = np.array(pickle.load(filehandle))
                # print(norand_agent_surv[:, 0])

        # Extract useful values
        vanilla_surv_x = vanilla_surv[:1000, 0]
        rand_agent_surv_x = rand_agent_surv[:, 0]
        norand_agent_surv_x = norand_agent_surv[:, 0]
        # convert the lists to series
        data = {
            'Vanilla': vanilla_surv_x,
            'GMBC Rand': rand_agent_surv_x,
            'GMBC NoRand': norand_agent_surv_x
        }

        colors = ['r', 'g', 'b']

        # get dataframe
        df = pd.DataFrame(data)
        print(df)

        # Plot
        for i, col in enumerate(df.columns):
            sns.distplot(df[[col]], color=colors[i])

        plt.legend(labels=['GMBC NoRand', 'GMBC Rand', 'Vanilla'])
        plt.xlabel("Forward Survived Distance (m)")
        plt.ylabel("Kernel Density Estimate")
        plt.show()

        # Print AVG and STDEV
        norand_avg = np.average(copy.deepcopy(norand_agent_surv_x))
        norand_std = np.std(copy.deepcopy(norand_agent_surv_x))
        rand_avg = np.average(copy.deepcopy(rand_agent_surv_x))
        rand_std = np.std(copy.deepcopy(rand_agent_surv_x))
        vanilla_avg = np.average(copy.deepcopy(vanilla_surv_x))
        vanilla_std = np.std(copy.deepcopy(vanilla_surv_x))

        print("Vanilla: AVG [{}] | STD [{}] | AMOUNT [{}]".format(
            vanilla_avg, vanilla_std, norand_agent_surv_x.shape[0]))
        print("RANDOM: AVG [{}] | STD [{}] AMOUNT [{}]".format(
            rand_avg, rand_std, rand_agent_surv_x.shape[0]))
        print("NOT RANDOM: AVG [{}] | STD [{}] AMOUNT [{}]".format(
            norand_avg, norand_std, vanilla_surv_x.shape[0]))

        # Get Survival Data for <=50m, >50m, and >90m
        # NO RAND
        less50_cond = norand_agent_surv_x <= 50.0
        norand_agent_surv_x_less_50 = np.extract(less50_cond,
                                                 norand_agent_surv_x)
        gtr50_cond = norand_agent_surv_x > 50.0
        norand_agent_surv_x_gtr_50_temp = np.extract(gtr50_cond,
                                                     norand_agent_surv_x)
        gtr50_cond = norand_agent_surv_x_gtr_50_temp < 90.0
        norand_agent_surv_x_gtr_50 = np.extract(
            gtr50_cond, norand_agent_surv_x_gtr_50_temp)
        gtr90_cond = norand_agent_surv_x >= 90.0
        norand_agent_surv_x_gtr_90 = np.extract(gtr90_cond,
                                                norand_agent_surv_x)

        # RAND
        less50_cond = rand_agent_surv_x <= 50.0
        rand_agent_surv_x_less_50 = np.extract(less50_cond, rand_agent_surv_x)
        gtr50_cond = rand_agent_surv_x > 50.0
        rand_agent_surv_x_gtr_50_temp = np.extract(gtr50_cond,
                                                   rand_agent_surv_x)
        gtr50_cond = rand_agent_surv_x_gtr_50_temp < 90.0
        rand_agent_surv_x_gtr_50 = np.extract(gtr50_cond,
                                              rand_agent_surv_x_gtr_50_temp)
        gtr90_cond = rand_agent_surv_x >= 90.0
        rand_agent_surv_x_gtr_90 = np.extract(gtr90_cond, rand_agent_surv_x)

        # VNL
        less50_cond = vanilla_surv_x <= 50.0
        vanilla_surv_x_less_50 = np.extract(less50_cond, vanilla_surv_x)
        gtr50_cond = vanilla_surv_x > 50.0
        vanilla_surv_x_gtr_50_temp = np.extract(gtr50_cond, vanilla_surv_x)
        gtr50_cond = vanilla_surv_x_gtr_50_temp < 90.0
        vanilla_surv_x_gtr_50 = np.extract(gtr50_cond,
                                           vanilla_surv_x_gtr_50_temp)
        gtr90_cond = vanilla_surv_x >= 90.0
        vanilla_surv_x_gtr_90 = np.extract(gtr90_cond, vanilla_surv_x)

        # <=50
        # Make sure all arrays filled
        if norand_agent_surv_x_less_50.size == 0:
            norand_agent_surv_x_less_50 = np.array([0])
        if rand_agent_surv_x_less_50.size == 0:
            rand_agent_surv_x_less_50 = np.array([0])
        if vanilla_surv_x_less_50.size == 0:
            vanilla_surv_x_less_50 = np.array([0])

        norand_avg = np.average(norand_agent_surv_x_less_50)
        norand_std = np.std(norand_agent_surv_x_less_50)
        rand_avg = np.average(rand_agent_surv_x_less_50)
        rand_std = np.std(rand_agent_surv_x_less_50)
        vanilla_avg = np.average(vanilla_surv_x_less_50)
        vanilla_std = np.std(vanilla_surv_x_less_50)
        print("<= 50m")
        print("Vanilla: AVG [{}] | STD [{}] | AMOUNT [{}]".format(
            vanilla_avg, vanilla_std, vanilla_surv_x_less_50.shape[0]))
        print("RANDOM: AVG [{}] | STD [{}] AMOUNT [{}]".format(
            rand_avg, rand_std, rand_agent_surv_x_less_50.shape[0]))
        print("NOT RANDOM: AVG [{}] | STD [{}] AMOUNT [{}]".format(
            norand_avg, norand_std, norand_agent_surv_x_less_50.shape[0]))

        # >50
        # Make sure all arrays filled
        if norand_agent_surv_x_gtr_50.size == 0:
            norand_agent_surv_x_gtr_50 = np.array([0])
        if rand_agent_surv_x_gtr_50.size == 0:
            rand_agent_surv_x_gtr_50 = np.array([0])
        if vanilla_surv_x_gtr_50.size == 0:
            vanilla_surv_x_gtr_50 = np.array([0])

        norand_avg = np.average(norand_agent_surv_x_gtr_50)
        norand_std = np.std(norand_agent_surv_x_gtr_50)
        rand_avg = np.average(rand_agent_surv_x_gtr_50)
        rand_std = np.std(rand_agent_surv_x_gtr_50)
        vanilla_avg = np.average(vanilla_surv_x_gtr_50)
        vanilla_std = np.std(vanilla_surv_x_gtr_50)
        print("> 50m and <90m")
        print("Vanilla: AVG [{}] | STD [{}] | AMOUNT [{}]".format(
            vanilla_avg, vanilla_std, vanilla_surv_x_gtr_50.shape[0]))
        print("RANDOM: AVG [{}] | STD [{}] AMOUNT [{}]".format(
            rand_avg, rand_std, rand_agent_surv_x_gtr_50.shape[0]))
        print("NOT RANDOM: AVG [{}] | STD [{}] AMOUNT [{}]".format(
            norand_avg, norand_std, norand_agent_surv_x_gtr_50.shape[0]))

        # >90
        # norand_agent_surv_x_gtr_90 = np.array(norand_agent_surv_x_gtr_90)
        # rand_agent_surv_x_gtr_90 = np.array(vanilla_surv_x_gtr_90)
        # vanilla_surv_x_gtr_90 = np.array(vanilla_surv_x_gtr_90)
        # Make sure all arrays filled
        if norand_agent_surv_x_gtr_90.size == 0:
            norand_agent_surv_x_gtr_90 = np.array([0])
        if rand_agent_surv_x_gtr_90.size == 0:
            rand_agent_surv_x_gtr_90 = np.array([0])
        if vanilla_surv_x_gtr_90.size == 0:
            vanilla_surv_x_gtr_90 = np.array([0])

        norand_avg = np.average(norand_agent_surv_x_gtr_90)
        norand_std = np.std(norand_agent_surv_x_gtr_90)
        rand_avg = np.average(rand_agent_surv_x_gtr_90)
        rand_std = np.std(rand_agent_surv_x_gtr_90)
        vanilla_avg = np.average(vanilla_surv_x_gtr_90)
        vanilla_std = np.std(vanilla_surv_x_gtr_90)
        print(">= 90m")
        print("Vanilla: AVG [{}] | STD [{}] | AMOUNT [{}]".format(
            vanilla_avg, vanilla_std, vanilla_surv_x_gtr_90.shape[0]))
        print("RANDOM: AVG [{}] | STD [{}] AMOUNT [{}]".format(
            rand_avg, rand_std, rand_agent_surv_x_gtr_90.shape[0]))
        print("NOT RANDOM: AVG [{}] | STD [{}] AMOUNT [{}]".format(
            norand_avg, norand_std, norand_agent_surv_x_gtr_90.shape[0]))

    elif training:
        rand_data_list = []
        norand_data_list = []
        rand_shortest_length = np.inf
        norand_shortest_length = np.inf
        for i in range(int(seed) + 1):
            # Training Data Plotter
            rand_data_temp = np.load(results_path + "/spot_ars_rand_" +
                                     "seed" + str(i) + ".npy")
            norand_data_temp = np.load(results_path + "/spot_ars_norand_" +
                                       "seed" + str(i) + ".npy")

            rand_shortest_length = min(
                np.shape(rand_data_temp[:, 1])[0], rand_shortest_length)
            norand_shortest_length = min(
                np.shape(norand_data_temp[:, 1])[0], norand_shortest_length)

            rand_data_list.append(rand_data_temp)
            norand_data_list.append(norand_data_temp)

        tot_rand_data = []
        tot_norand_data = []
        norm_rand_data = []
        norm_norand_data = []
        for i in range(int(seed) + 1):
            tot_rand_data.append(
                moving_average(rand_data_list[i][:rand_shortest_length, 0]))
            tot_norand_data.append(
                moving_average(
                    norand_data_list[i][:norand_shortest_length, 0]))
            norm_rand_data.append(
                moving_average(rand_data_list[i][:rand_shortest_length, 1]))
            norm_norand_data.append(
                moving_average(
                    norand_data_list[i][:norand_shortest_length, 1]))

        tot_rand_data = np.array(tot_rand_data)
        tot_norand_data = np.array(tot_norand_data)
        norm_rand_data = np.array(norm_rand_data)
        norm_norand_data = np.array(norm_norand_data)

        # column-wise
        axis = 0

        # MEAN
        tot_rand_mean = tot_rand_data.mean(axis=axis)
        tot_norand_mean = tot_norand_data.mean(axis=axis)
        norm_rand_mean = norm_rand_data.mean(axis=axis)
        norm_norand_mean = norm_norand_data.mean(axis=axis)

        # STD
        tot_rand_std = tot_rand_data.std(axis=axis)
        tot_norand_std = tot_norand_data.std(axis=axis)
        norm_rand_std = norm_rand_data.std(axis=axis)
        norm_norand_std = norm_norand_data.std(axis=axis)

        aranged_rand = np.arange(np.shape(tot_rand_mean)[0])
        aranged_norand = np.arange(np.shape(tot_norand_mean)[0])

        if ARGS.TotalReward:
            if ARGS.Raw:
                plt.plot(rand_data_list[0][:, 0],
                         label="Randomized (Total Reward)",
                         color='g')
                plt.plot(norand_data_list[0][:, 0],
                         label="Non-Randomized (Total Reward)",
                         color='r')
            plt.plot(aranged_norand,
                     tot_norand_mean,
                     label="MA: Non-Randomized (Total Reward)",
                     color='r')
            plt.fill_between(aranged_norand,
                             tot_norand_mean - tot_norand_std,
                             tot_norand_mean + tot_norand_std,
                             color='r',
                             alpha=0.2)
            plt.plot(aranged_rand,
                     tot_rand_mean,
                     label="MA: Randomized (Total Reward)",
                     color='g')
            plt.fill_between(aranged_rand,
                             tot_rand_mean - tot_rand_std,
                             tot_rand_mean + tot_rand_std,
                             color='g',
                             alpha=0.2)
        else:
            if ARGS.Raw:
                plt.plot(rand_data_list[0][:, 1],
                         label="Randomized (Reward/dt)",
                         color='g')
                plt.plot(norand_data_list[0][:, 1],
                         label="Non-Randomized (Reward/dt)",
                         color='r')
            plt.plot(aranged_norand,
                     norm_norand_mean,
                     label="MA: Non-Randomized (Reward/dt)",
                     color='r')
            plt.fill_between(aranged_norand,
                             norm_norand_mean - norm_norand_std,
                             norm_norand_mean + norm_norand_std,
                             color='r',
                             alpha=0.2)
            plt.plot(aranged_rand,
                     norm_rand_mean,
                     label="MA: Randomized (Reward/dt)",
                     color='g')
            plt.fill_between(aranged_rand,
                             norm_rand_mean - norm_rand_std,
                             norm_rand_mean + norm_rand_std,
                             color='g',
                             alpha=0.2)
        plt.xlabel("Epoch #")
        plt.ylabel("Reward")
        plt.title(
            "Training Performance with {} seed samples".format(int(seed) + 1))
        plt.legend()
        plt.show()

    elif pout:

        if ARGS.Rough:
            terrain_name = "rough_"
        else:
            terrain_name = "flat_"

        if ARGS.TrueAct:
            action_name = "agent_act"
        else:
            action_name = "robot_act"

        action = np.load(results_path + "/" + "policy_out_" + terrain_name +
                         action_name + ".npy")

        ClearHeight_act = action[:, 0]
        BodyHeight_act = action[:, 1]
        Residuals_act = action[:, 2:]

        plt.plot(ClearHeight_act, label='Clearance Height Mod', color='black')
        plt.plot(BodyHeight_act, label='Body Height Mod', color='darkviolet')

        # FL
        plt.plot(Residuals_act[:, 0],
                 label='Residual: FL (x)',
                 color='limegreen')
        plt.plot(Residuals_act[:, 1], label='Residual: FL (y)', color='lime')
        plt.plot(Residuals_act[:, 2], label='Residual: FL (z)', color='green')

        # FR
        plt.plot(Residuals_act[:, 3],
                 label='Residual: FR (x)',
                 color='lightskyblue')
        plt.plot(Residuals_act[:, 4],
                 label='Residual: FR (y)',
                 color='dodgerblue')
        plt.plot(Residuals_act[:, 5], label='Residual: FR (z)', color='blue')

        # BL
        plt.plot(Residuals_act[:, 6],
                 label='Residual: BL (x)',
                 color='firebrick')
        plt.plot(Residuals_act[:, 7],
                 label='Residual: BL (y)',
                 color='crimson')
        plt.plot(Residuals_act[:, 8], label='Residual: BL (z)', color='red')

        # BR
        plt.plot(Residuals_act[:, 9], label='Residual: BR (x)', color='gold')
        plt.plot(Residuals_act[:, 10],
                 label='Residual: BR (y)',
                 color='orange')
        plt.plot(Residuals_act[:, 11], label='Residual: BR (z)', color='coral')

        plt.xlabel("Epoch Iteration")
        plt.ylabel("Action Value")
        plt.title("Policy Output")
        plt.legend()
        plt.show()


if __name__ == '__main__':
    main()
