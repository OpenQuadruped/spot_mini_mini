from tg_lib.tg_policy import TGPolicy
import pickle
import numpy as np
np.random.seed(0)

# Multiprocessing package for python
# Parallelization improvements based on:
# https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/ARS/ars.py

# Messages for Pipes
_RESET = 1
_CLOSE = 2
_EXPLORE = 3
_EXPLORE_TG = 4


def ParallelWorker(childPipe, env):
    nb_states = env.observation_space.shape[0]
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
                # Clip action between +-1 for execution in env
                for a in range(len(action)):
                    action[a] = np.clip(action[a], -max_action, max_action)
                state, reward, done, _ = env.step(action)
                reward = max(min(reward, 1), -1)
                sum_rewards += reward
                timesteps += 1
            childPipe.send([sum_rewards])
            continue
        if message == _EXPLORE_TG:
            # Payloads received by parent in ARSAgent.train()
            # [0]: normalizer, [1]: policy, [2]: direction, [3]: delta
            # we use local normalizer so no need for [0] (optional)
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
                # Extract TG information from action
                alpha_tg = action[0]
                h_tg = action[1]
                intensity = action[2]
                f_tg = action[3]
                Beta = action[4]
                residuals = action[5:]
                # GET ACTION FROM TG
                action = policy.TGPolicy.get_utg(
                    residuals, alpha_tg, h_tg, intensity,
                    policy.env.minitaur.num_motors)
                # Clip action between +-1 for execution in env
                for a in range(len(action)):
                    action[a] = np.clip(action[a], -max_action, max_action)
                state, reward, done, _ = env.step(action)
                # PROGRESS TG
                policy.TGPolicy.increment(policy.env._time_step, f_tg, Beta)
                # APPEND PHASE OBS
                state = np.append(state, policy.TGPolicy.get_TG_state())
                # Clip reward between -1 and 1
                reward = max(min(reward, 1), -1)
                sum_rewards += reward
                timesteps += 1
            childPipe.send([sum_rewards])
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
            learning_rate=0.02,
            # number of random expl_noise variations generated
            # each step
            # each one will be run for 2 epochs, + and -
            num_deltas=16,
            # used to update weights, sorted by highest rwrd
            num_best_deltas=16,
            # number of timesteps per episode per rollout
            episode_steps=1000,
            # weight of sampled exploration noise
            expl_noise=0.01,
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

        squeezes states between 0 and 1
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
    def __init__(self, normalizer, policy, env, TGPolicy=TGPolicy()):
        self.normalizer = normalizer
        self.policy = policy
        self.state_dim = self.policy.state_dim
        self.action_dim = self.policy.action_dim
        self.env = env
        self.max_action = float(self.env.action_space.high[0])
        self.successes = 0
        self.last_reward = 0.0
        self.phase = 0
        self.desired_velocity = -0.3
        self.desired_rate = 0.0
        self.flip = 0
        self.increment = 0
        self.scaledown = True
        self.type = "Stop"
        self.TGPolicy = TGPolicy

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
        return sum_rewards

    # Deploy Policy in one direction over one whole episode
    # DO THIS ONCE PER ROLLOUT OR DURING DEPLOYMENT
    def deployTG(self, direction=None, delta=None):
        state = self.env.reset(self.desired_velocity, self.desired_rate)
        # APPEND PHASE OBS
        state = np.append(state, self.TGPolicy.get_TG_state())
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
            # Extract TG information from action
            alpha_tg = action[0]
            h_tg = action[1]
            intensity = action[2]
            f_tg = action[3]
            Beta = action[4]
            residuals = action[5:]
            # GET ACTION FROM TG
            action = self.TGPolicy.get_utg(residuals, alpha_tg, h_tg,
                                           intensity,
                                           self.env.minitaur.num_motors)
            # Clip action between +-1 for execution in env
            for a in range(len(action)):
                action[a] = np.clip(action[a], -self.max_action,
                                    self.max_action)
            state, reward, done, _ = self.env.step(action)
            # PROGRESS TG
            self.TGPolicy.increment(self.env._time_step, f_tg, Beta)
            # APPEND PHASE OBS
            state = np.append(state, self.TGPolicy.get_TG_state())
            # Clip reward between -1 and 1 to prevent outliers from
            # distorting weights
            reward = np.clip(reward, -self.max_action, self.max_action)
            sum_rewards += reward
            timesteps += 1
        return sum_rewards

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

    def train_parallel(self, parentPipes, expl="vanilla"):
        """ Execute rollouts in parallel using multiprocessing library
            based on: # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_envs/ARS/ars.py
        """
        # USE VANILLA OR TG POLICY
        if expl == "vanilla":
            exploration = _EXPLORE
        else:
            exploration = _EXPLORE_TG

        # Initializing the perturbations deltas and the positive/negative rewards
        deltas = self.policy.sample_deltas()
        # Initialize +- reward list of size num_deltas
        positive_rewards = [0] * self.policy.num_deltas
        negative_rewards = [0] * self.policy.num_deltas

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
                        self.desired_velocity, self.desired_rate
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
                        self.desired_velocity, self.desired_rate
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
        if expl == "vanilla":
            eval_reward = self.deploy()
        else:
            eval_reward = self.deployTG()
        self.last_reward = eval_reward

        # Call evolve_state function to adjust reward/state
        self.evolve_state()
        return eval_reward

    def evolve_state(self):
        """ Change desired velocity and rate parameters
            over course of training if success criteria
            is met for current goals
        """
        # if self.last_reward > 400.0:
        #     self.successes += 1
        # elif self.successes > 0:
        #     self.successes -= 1

        # if self.successes > 10:
        #     self.successes = 0

        #     # # alternating phases for fwd vel
        #     # # start at 0.5 m/s and then flip to -0.5ms
        #     # # then increment to -0.4ms, then flip to 0.4ms
        #     # # then decrement to 0.3ms etc.

        #     # if self.flip % 2 == 0:
        #     #     self.desired_velocity *= -1
        #     # else:
        #     #     if self.increment % 2 != 0:
        #     #         if self.scaledown:
        #     #             if np.sign(self.desired_velocity) < 0:
        #     #                 self.desired_velocity += 0.1
        #     #             else:
        #     #                 self.desired_velocity -= 0.1
        #     #         else:
        #     #             if np.sign(self.desired_velocity) > 0:
        #     #                 self.desired_velocity += 0.1
        #     #             else:
        #     #                 self.desired_velocity -= 0.1

        #     #     if self.desired_velocity == 0.0:
        #     #         self.scaledown = False
        #     #     elif abs(self.desired_velocity) == 0.5:
        #     #         self.increment = True
        #     if self.scaledown:
        #         self.desired_velocity -= 0.1
        #     else:
        #         self.desired_velocity += 0.1

        #     if self.desired_velocity >= 0.5:
        #         self.scaledown = True
        #     elif self.desired_velocity <= -0.5:
        #         self.scaledown = False

        #     # self.flip += 1
        #     # self.increment += 1

        # self.desired_velocity = np.random.uniform(low=0.0, high=1.0)
        self.desired_velocity = self.desired_velocity

        print("NEW DESIRED VELOCITY IS {}".format(self.desired_velocity))
        print("NEW DESIRED RATE IS {}".format(self.desired_rate))

    def save(self, filename):
        """ Save the Policy
        """
        with open(filename + '_policy', 'wb') as filehandle:
            pickle.dump(self.policy, filehandle)

    def load(self, filename):
        """ Load the Policy
        """
        with open(filename + '_policy', 'rb') as filehandle:
            self.policy = pickle.load(filehandle)
