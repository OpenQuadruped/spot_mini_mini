# from tg_lib.tg_policy import TGPolicy
import pickle
import numpy as np
from scipy.signal import butter, filtfilt
from spotmicro.GaitGenerator.Bezier import BezierGait
from spotmicro.OpenLoopSM.SpotOL import BezierStepper
from spotmicro.Kinematics.SpotKinematics import SpotModel
from spotmicro.Kinematics.LieAlgebra import TransToRp
import copy
from spotmicro.util.gui import GUI

np.random.seed(0)

# Multiprocessing package for python
# Parallelization improvements based on:
# https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/ARS/ars.py

# Messages for Pipes
_RESET = 1
_CLOSE = 2
_EXPLORE = 3
_EXPLORE_TG = 4

# Params for TG
CD_SCALE = 0.05
SLV_SCALE = 0.05
RESIDUALS_SCALE = 0.015
Z_SCALE = 0.05

# Filter actions
alpha = 0.7

# Added this to avoid filtering residuals
# -1 for all
actions_to_filter = 14

# For auto yaw control
P_yaw = 5.0

# Cummulative timestep exponential reward
# cum_dt_exp = 1.1
cum_dt_exp = 0.0


def butter_lowpass_filter(data, cutoff, fs, order=2):
    """ Pass two subsequent datapoints in here to be filtered
    """
    nyq = 0.5 * fs  # Nyquist Frequency
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y


def ParallelWorker(childPipe, env, nb_states):
    """ Function to deploy multiple ARS agents in parallel
    """
    # nb_states = env.observation_space.shape[0]
    # common normalizer
    normalizer = Normalizer(nb_states)
    max_action = float(env.action_space.high[0])
    _ = env.reset()
    n = 0
    while True:
        n += 1
        try:
            # Only block for short times to have keyboard exceptions be raised.
            if not childPipe.poll(0.001):
                continue
            message, payload = childPipe.recv()
        except (EOFError, KeyboardInterrupt):
            break
        if message == _RESET:
            _ = env.reset()
            childPipe.send(["reset ok"])
            continue
        if message == _EXPLORE:
            # Payloads received by parent in ARSAgent.train()
            # [0]: normalizer, [1]: policy, [2]: direction, [3]: delta
            # we use local normalizer so no need for [0] (optional)
            # normalizer = payload[0]
            policy = payload[1]
            direction = payload[2]
            delta = payload[3]
            desired_velocity = payload[4]
            desired_rate = payload[5]
            state = env.reset(desired_velocity, desired_rate)
            sum_rewards = 0.0
            timesteps = 0
            done = False
            while not done and timesteps < policy.episode_steps:
                normalizer.observe(state)
                # Normalize State
                state = normalizer.normalize(state)
                action = policy.evaluate(state, delta, direction)
                # # Clip action between +-1 for execution in env
                # for a in range(len(action)):
                #     action[a] = np.clip(action[a], -max_action, max_action)
                state, reward, done, _ = env.step(action)
                reward = max(min(reward, 1), -1)
                sum_rewards += reward
                timesteps += 1
            childPipe.send([sum_rewards])
            continue
        if message == _EXPLORE_TG:
            # Payloads received by parent in ARSAgent.train()
            # [0]: normalizer, [1]: policy, [2]: direction, [3]: delta
            # [4]: desired_velocity, [5]: desired_rate, [6]: Trajectory Gen
            # we use local normalizer so no need for [0] (optional)
            # normalizer = payload[0]
            policy = payload[1]
            direction = payload[2]
            delta = payload[3]
            desired_velocity = payload[4]
            desired_rate = payload[5]
            TGP = payload[6]
            smach = payload[7]
            spot = payload[8]
            state = env.reset()
            sum_rewards = 0.0
            timesteps = 0
            done = False
            T_bf = copy.deepcopy(spot.WorldToFoot)
            T_b0 = copy.deepcopy(spot.WorldToFoot)
            action = env.action_space.sample()
            action[:] = 0.0
            old_act = action[:actions_to_filter]

            # For auto yaw control
            yaw = 0.0
            while not done and timesteps < policy.episode_steps:
                # smach.ramp_up()
                pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth = smach.StateMachine(
                )

                env.spot.GetExternalObservations(TGP, smach)

                # Read UPDATED state based on controls and phase
                state = env.return_state()
                normalizer.observe(state)
                # NOTE: Don't normalize contacts - must stay 0/1
                state = normalizer.normalize(state)
                action = policy.evaluate(state, delta, direction)

                contacts = state[-4:]

                action = np.tanh(action)

                # EXP FILTER
                action[:actions_to_filter] = alpha * old_act + (
                    1.0 - alpha) * action[:actions_to_filter]
                old_act = action[:actions_to_filter]

                ClearanceHeight += action[0] * CD_SCALE

                # CLIP EVERYTHING
                StepLength = np.clip(StepLength, smach.StepLength_LIMITS[0],
                                     smach.StepLength_LIMITS[1])
                StepVelocity = np.clip(StepVelocity,
                                       smach.StepVelocity_LIMITS[0],
                                       smach.StepVelocity_LIMITS[1])
                LateralFraction = np.clip(LateralFraction,
                                          smach.LateralFraction_LIMITS[0],
                                          smach.LateralFraction_LIMITS[1])
                YawRate = np.clip(YawRate, smach.YawRate_LIMITS[0],
                                  smach.YawRate_LIMITS[1])
                ClearanceHeight = np.clip(ClearanceHeight,
                                          smach.ClearanceHeight_LIMITS[0],
                                          smach.ClearanceHeight_LIMITS[1])
                PenetrationDepth = np.clip(PenetrationDepth,
                                           smach.PenetrationDepth_LIMITS[0],
                                           smach.PenetrationDepth_LIMITS[1])

                # For auto yaw control
                yaw = env.return_yaw()
                YawRate += -yaw * P_yaw

                # Get Desired Foot Poses
                if timesteps > 20:
                    T_bf = TGP.GenerateTrajectory(StepLength, LateralFraction,
                                                  YawRate, StepVelocity, T_b0,
                                                  T_bf, ClearanceHeight,
                                                  PenetrationDepth, contacts)
                else:
                    T_bf = TGP.GenerateTrajectory(0.0, 0.0, 0.0, 0.1, T_b0,
                                                  T_bf, ClearanceHeight,
                                                  PenetrationDepth, contacts)
                    action[:] = 0.0

                action[2:] *= RESIDUALS_SCALE

                # Add DELTA to XYZ Foot Poses
                T_bf_copy = copy.deepcopy(T_bf)
                T_bf_copy["FL"][:3, 3] += action[2:5]
                T_bf_copy["FR"][:3, 3] += action[5:8]
                T_bf_copy["BL"][:3, 3] += action[8:11]
                T_bf_copy["BR"][:3, 3] += action[11:14]

                # Adjust Body Height with action!
                pos[2] += abs(action[1]) * Z_SCALE

                joint_angles = spot.IK(orn, pos, T_bf_copy)
                # Pass Joint Angles
                env.pass_joint_angles(joint_angles.reshape(-1))
                # Perform action
                next_state, reward, done, _ = env.step(action)
                sum_rewards += reward
                timesteps += 1
                # Divide reward by timesteps for normalized reward + add exponential surival reward
            childPipe.send([(sum_rewards + timesteps**cum_dt_exp) / timesteps])
            continue
        if message == _CLOSE:
            childPipe.send(["close ok"])
            break
    childPipe.close()


class Policy():
    """ state --> action
    """
    def __init__(
            self,
            state_dim,
            action_dim,
            # how much weights are changed each step
            learning_rate=0.03,
            # number of random expl_noise variations generated
            # each step
            # each one will be run for 2 epochs, + and -
            num_deltas=16,
            # used to update weights, sorted by highest rwrd
            num_best_deltas=16,
            # number of timesteps per episode per rollout
            episode_steps=5000,
            # weight of sampled exploration noise
            expl_noise=0.05,
            # for seed gen
            seed=0):

        # Tunable Hyperparameters
        self.learning_rate = learning_rate
        self.num_deltas = num_deltas
        self.num_best_deltas = num_best_deltas
        # there cannot be more best_deltas than there are deltas
        assert self.num_best_deltas <= self.num_deltas
        self.episode_steps = episode_steps
        self.expl_noise = expl_noise
        self.seed = seed
        np.random.seed(seed)
        self.state_dim = state_dim
        self.action_dim = action_dim

        # input/ouput matrix with weights set to zero
        # this is the perception matrix (policy)
        self.theta = np.zeros((action_dim, state_dim))

    def evaluate(self, state, delta=None, direction=None):
        """ state --> action
        """

        # if direction is None, deployment mode: takes dot product
        # to directly sample from (use) policy
        if direction is None:
            return self.theta.dot(state)

        # otherwise, add (+-) directed expl_noise before taking dot product (policy)
        # this is where the 2*num_deltas rollouts comes from
        elif direction == "+":
            return (self.theta + self.expl_noise * delta).dot(state)
        elif direction == "-":
            return (self.theta - self.expl_noise * delta).dot(state)

    def sample_deltas(self):
        """ generate array of random expl_noise matrices. Length of
            array = num_deltas
            matrix dimension: pxn where p=observation dim and
            n=action dim
        """
        deltas = []
        # print("SHAPE THING with *: {}".format(*self.theta.shape))
        # print("SHAPE THING NORMALLY: ({}, {})".format(self.theta.shape[0],
        #                                               self.theta.shape[1]))
        # print("ACTUAL SHAPE: {}".format(self.theta.shape))
        # print("SHAPE OF EXAMPLE DELTA WITH *: {}".format(
        #     np.random.randn(*self.theta.shape).shape))
        # print("SHAPE OF EXAMPLE DELTA NOMRALLY: {}".format(
        #     np.random.randn(self.theta.shape[0], self.theta.shape[1]).shape))

        for _ in range(self.num_deltas):
            deltas.append(
                np.random.randn(self.theta.shape[0], self.theta.shape[1]))

        return deltas

    def update(self, rollouts, std_dev_rewards):
        """ Update policy weights (theta) based on rewards
            from 2*num_deltas rollouts
        """
        step = np.zeros(self.theta.shape)
        for r_pos, r_neg, delta in rollouts:
            # how much to deviate from policy
            step += (r_pos - r_neg) * delta
        self.theta += self.learning_rate / (self.num_best_deltas *
                                            std_dev_rewards) * step


class Normalizer():
    """ this ensures that the policy puts equal weight upon
        each state component.
    """

    # Normalizes the states
    def __init__(self, state_dim):
        """ Initialize state space (all zero)
        """
        self.state = np.zeros(state_dim)
        self.mean = np.zeros(state_dim)
        self.mean_diff = np.zeros(state_dim)
        self.var = np.zeros(state_dim)

    def observe(self, x):
        """ Compute running average and variance
            clip variance >0 to avoid division by zero
        """
        self.state += 1.0
        last_mean = self.mean.copy()

        # running avg
        self.mean += (x - self.mean) / self.state

        # used to compute variance
        self.mean_diff += (x - last_mean) * (x - self.mean)
        # variance
        self.var = (self.mean_diff / self.state).clip(min=1e-2)

    def normalize(self, states):
        """ subtract mean state value from current state
            and divide by standard deviation (sqrt(var))
            to normalize
        """
        state_mean = self.mean
        state_std = np.sqrt(self.var)
        return (states - state_mean) / state_std


class ARSAgent():
    def __init__(self,
                 normalizer,
                 policy,
                 env,
                 smach=None,
                 TGP=None,
                 spot=None,
                 gui=False):
        self.normalizer = normalizer
        self.policy = policy
        self.state_dim = self.policy.state_dim
        self.action_dim = self.policy.action_dim
        self.env = env
        self.max_action = float(self.env.action_space.high[0])
        self.successes = 0
        self.phase = 0
        self.desired_velocity = 0.5
        self.desired_rate = 0.0
        self.flip = 0
        self.increment = 0
        self.scaledown = True
        self.type = "Stop"
        self.smach = smach
        if smach is not None:
            self.BaseClearanceHeight = self.smach.ClearanceHeight
            self.BasePenetrationDepth = self.smach.PenetrationDepth
        self.TGP = TGP
        self.spot = spot
        if gui:
            self.g_u_i = GUI(self.env.spot.quadruped)
        else:
            self.g_u_i = None

        self.action_history = []
        self.true_action_history = []

    # Deploy Policy in one direction over one whole episode
    # DO THIS ONCE PER ROLLOUT OR DURING DEPLOYMENT
    def deploy(self, direction=None, delta=None):
        state = self.env.reset(self.desired_velocity, self.desired_rate)
        sum_rewards = 0.0
        timesteps = 0
        done = False
        while not done and timesteps < self.policy.episode_steps:
            # print("STATE: ", state)
            # print("dt: {}".format(timesteps))
            self.normalizer.observe(state)
            # Normalize State
            state = self.normalizer.normalize(state)
            action = self.policy.evaluate(state, delta, direction)
            # Clip action between +-1 for execution in env
            for a in range(len(action)):
                action[a] = np.clip(action[a], -self.max_action,
                                    self.max_action)
            # print("ACTION: ", action)
            state, reward, done, _ = self.env.step(action)
            # print("STATE: ", state)
            # Clip reward between -1 and 1 to prevent outliers from
            # distorting weights
            reward = np.clip(reward, -self.max_action, self.max_action)
            sum_rewards += reward
            timesteps += 1
            # Divide rewards by timesteps for reward-per-step + exp survive rwd
        return (sum_rewards + timesteps**cum_dt_exp) / timesteps

    # Deploy Policy in one direction over one whole episode
    # DO THIS ONCE PER ROLLOUT OR DURING DEPLOYMENT
    def deployTG(self, direction=None, delta=None):
        state = self.env.reset()
        sum_rewards = 0.0
        timesteps = 0
        done = False
        # alpha = []
        # h = []
        # f = []
        T_bf = copy.deepcopy(self.spot.WorldToFoot)
        T_b0 = copy.deepcopy(self.spot.WorldToFoot)
        self.action_history = []
        self.true_action_history = []
        action = self.env.action_space.sample()
        action[:] = 0.0
        old_act = action[:actions_to_filter]
        # For auto yaw correction
        yaw = 0.0
        while not done and timesteps < self.policy.episode_steps:
            # self.smach.ramp_up()
            pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth = self.smach.StateMachine(
            )

            if self.g_u_i:
                pos, orn, StepLength, LateralFraction, YawRate, StepVelocity, ClearanceHeight, PenetrationDepth, SwingPeriod = self.g_u_i.UserInput(
                )
                self.TGP.Tswing = SwingPeriod

            self.env.spot.GetExternalObservations(self.TGP, self.smach)

            # Read UPDATED state based on controls and phase
            state = self.env.return_state()
            self.normalizer.observe(state)
            # Don't normalize contacts
            state = self.normalizer.normalize(state)
            action = self.policy.evaluate(state, delta, direction)

            # Action History
            self.action_history.append(np.tanh(action))

            # Action History
            true_action = copy.deepcopy(np.tanh(action))
            true_action[0] *= CD_SCALE
            true_action[1] = abs(true_action[1]) * Z_SCALE
            true_action[2:] *= RESIDUALS_SCALE
            self.true_action_history.append(true_action)

            action = np.tanh(action)

            # EXP FILTER
            action[:actions_to_filter] = alpha * old_act + (
                1.0 - alpha) * action[:actions_to_filter]
            old_act = action[:actions_to_filter]

            ClearanceHeight += action[0] * CD_SCALE

            # CLIP EVERYTHING
            StepLength = np.clip(StepLength, self.smach.StepLength_LIMITS[0],
                                 self.smach.StepLength_LIMITS[1])
            StepVelocity = np.clip(StepVelocity,
                                   self.smach.StepVelocity_LIMITS[0],
                                   self.smach.StepVelocity_LIMITS[1])
            LateralFraction = np.clip(LateralFraction,
                                      self.smach.LateralFraction_LIMITS[0],
                                      self.smach.LateralFraction_LIMITS[1])
            YawRate = np.clip(YawRate, self.smach.YawRate_LIMITS[0],
                              self.smach.YawRate_LIMITS[1])
            ClearanceHeight = np.clip(ClearanceHeight,
                                      self.smach.ClearanceHeight_LIMITS[0],
                                      self.smach.ClearanceHeight_LIMITS[1])
            PenetrationDepth = np.clip(PenetrationDepth,
                                       self.smach.PenetrationDepth_LIMITS[0],
                                       self.smach.PenetrationDepth_LIMITS[1])

            contacts = copy.deepcopy(state[-4:])
            # contacts = [0, 0, 0, 0]

            # print("CONTACTS: {}".format(contacts))

            yaw = self.env.return_yaw()
            if not self.g_u_i:
                YawRate += -yaw * P_yaw

            # Get Desired Foot Poses
            if timesteps > 20:
                T_bf = self.TGP.GenerateTrajectory(StepLength, LateralFraction,
                                                   YawRate, StepVelocity, T_b0,
                                                   T_bf, ClearanceHeight,
                                                   PenetrationDepth, contacts)
            else:
                T_bf = self.TGP.GenerateTrajectory(0.0, 0.0, 0.0, 0.1, T_b0,
                                                   T_bf, ClearanceHeight,
                                                   PenetrationDepth, contacts)
                action[:] = 0.0

            action[2:] *= RESIDUALS_SCALE

            # Add DELTA to XYZ Foot Poses
            T_bf_copy = copy.deepcopy(T_bf)
            T_bf_copy["FL"][:3, 3] += action[2:5]
            T_bf_copy["FR"][:3, 3] += action[5:8]
            T_bf_copy["BL"][:3, 3] += action[8:11]
            T_bf_copy["BR"][:3, 3] += action[11:14]

            # Adjust Height!
            pos[2] += abs(action[1]) * Z_SCALE

            joint_angles = self.spot.IK(orn, pos, T_bf_copy)
            # Pass Joint Angles
            self.env.pass_joint_angles(joint_angles.reshape(-1))

            # Perform action
            next_state, reward, done, _ = self.env.step(action)
            sum_rewards += reward
            timesteps += 1

        self.TGP.reset()
        self.smach.reshuffle()
        self.smach.PenetrationDepth = self.BasePenetrationDepth
        self.smach.ClearanceHeight = self.BaseClearanceHeight
        return sum_rewards, timesteps

    def returnPose(self):
        return self.env.spot.GetBasePosition()

    def train(self):
        # Sample random expl_noise deltas
        print("-------------------------------")
        # print("Sampling Deltas")
        deltas = self.policy.sample_deltas()
        # Initialize +- reward list of size num_deltas
        positive_rewards = [0] * self.policy.num_deltas
        negative_rewards = [0] * self.policy.num_deltas

        # Execute 2*num_deltas rollouts and store +- rewards
        print("Deploying Rollouts")
        for i in range(self.policy.num_deltas):
            print("Rollout #{}".format(i + 1))
            positive_rewards[i] = self.deploy(direction="+", delta=deltas[i])
            negative_rewards[i] = self.deploy(direction="-", delta=deltas[i])

        # Calculate std dev
        std_dev_rewards = np.array(positive_rewards + negative_rewards).std()

        # Order rollouts in decreasing list using cum reward as criterion
        unsorted_rollouts = [(positive_rewards[i], negative_rewards[i],
                              deltas[i])
                             for i in range(self.policy.num_deltas)]
        # When sorting, take the max between the reward for +- disturbance
        sorted_rollouts = sorted(
            unsorted_rollouts,
            key=lambda x: max(unsorted_rollouts[0], unsorted_rollouts[1]),
            reverse=True)

        # Only take first best_num_deltas rollouts
        rollouts = sorted_rollouts[:self.policy.num_best_deltas]

        # Update Policy
        self.policy.update(rollouts, std_dev_rewards)

        # Execute Current Policy
        eval_reward = self.deploy()
        return eval_reward

    def train_parallel(self, parentPipes):
        """ Execute rollouts in parallel using multiprocessing library
            based on: # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/ARS/ars.py
        """
        # USE VANILLA OR TG POLICY
        if self.TGP is None:
            exploration = _EXPLORE
        else:
            exploration = _EXPLORE_TG

        # Initializing the perturbations deltas and the positive/negative rewards
        deltas = self.policy.sample_deltas()
        # Initialize +- reward list of size num_deltas
        positive_rewards = [0] * self.policy.num_deltas
        negative_rewards = [0] * self.policy.num_deltas

        smach = copy.deepcopy(self.smach)
        smach.ClearanceHeight = self.BaseClearanceHeight
        smach.PenetrationDepth = self.BasePenetrationDepth
        smach.reshuffle()

        if parentPipes:
            for i in range(self.policy.num_deltas):
                # Execute each rollout on a separate thread
                parentPipe = parentPipes[i]
                # NOTE: target for parentPipe specified in main_ars.py
                # (target is ParallelWorker fcn defined up top)
                parentPipe.send([
                    exploration,
                    [
                        self.normalizer, self.policy, "+", deltas[i],
                        self.desired_velocity, self.desired_rate, self.TGP,
                        smach, self.spot
                    ]
                ])
            for i in range(self.policy.num_deltas):
                # Receive cummulative reward from each rollout
                positive_rewards[i] = parentPipes[i].recv()[0]

            for i in range(self.policy.num_deltas):
                # Execute each rollout on a separate thread
                parentPipe = parentPipes[i]
                parentPipe.send([
                    exploration,
                    [
                        self.normalizer, self.policy, "-", deltas[i],
                        self.desired_velocity, self.desired_rate, self.TGP,
                        smach, self.spot
                    ]
                ])
            for i in range(self.policy.num_deltas):
                # Receive cummulative reward from each rollout
                negative_rewards[i] = parentPipes[i].recv()[0]

        else:
            raise ValueError(
                "Select 'train' method if you are not using multiprocessing!")

        # Calculate std dev
        std_dev_rewards = np.array(positive_rewards + negative_rewards).std()

        # Order rollouts in decreasing list using cum reward as criterion
        # take max between reward for +- disturbance as that rollout's reward
        # Store max between positive and negative reward as key for sort
        scores = {
            k: max(r_pos, r_neg)
            for k, (
                r_pos,
                r_neg) in enumerate(zip(positive_rewards, negative_rewards))
        }
        indeces = sorted(scores.keys(), key=lambda x: scores[x],
                         reverse=True)[:self.policy.num_deltas]
        # print("INDECES: ", indeces)
        rollouts = [(positive_rewards[k], negative_rewards[k], deltas[k])
                    for k in indeces]

        # Update Policy
        self.policy.update(rollouts, std_dev_rewards)

        # Execute Current Policy USING VANILLA OR TG
        if self.TGP is None:
            return self.deploy()
        else:
            return self.deployTG()

    def save(self, filename):
        """ Save the Policy

        :param filename: the name of the file where the policy is saved
        """
        with open(filename + '_policy', 'wb') as filehandle:
            pickle.dump(self.policy.theta, filehandle)

    def load(self, filename):
        """ Load the Policy

        :param filename: the name of the file where the policy is saved
        """
        with open(filename + '_policy', 'rb') as filehandle:
            self.policy.theta = pickle.load(filehandle)
