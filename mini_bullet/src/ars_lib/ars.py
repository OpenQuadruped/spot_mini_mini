import pickle
import os
import numpy as np


class Policy():
    """ state --> action
    """
    def __init__(
            self,
            state_dim,
            action_dim,
            # how much weights are changed each step
            learning_rate=0.02,
            # number of random noise variations generated
            # each step
            # each one will be run for 2 epochs, + and -
            num_deltas=16,
            # used to update weights, sorted by highest rwrd
            num_best_deltas=16,
            # number of timesteps per episode per rollout
            episode_steps=2000,
            # weight of sampled noise
            noise=0.03,
            # for seed gen
            seed=1):

        # Tunable Hyperparameters
        self.learning_rate = learning_rate
        self.num_deltas = num_deltas
        self.num_best_deltas = num_best_deltas
        # there cannot be more best_deltas than there are deltas
        assert self.num_best_deltas <= self.num_deltas
        self.episode_steps = episode_steps
        self.noise = noise
        self.seed = seed

        # input/ouput matrix with weights set to zero
        # this is the perception matrix (policy)
        self.theta = np.zeros((action_dim, state_dim))

    def evaluate(self, input, delta=None, direction=None):
        """ state --> action
        """

        # if direction is None, deployment mode: takes dot product
        # to directly sample from (use) policy
        if direction is None:
            return self.theta.dot(input)

        # otherwise, add (+-) directed noise before taking dot product (policy)
        # this is where the 2*num_deltas rollouts comes from
        elif direction == "+":
            return (self.theta + self.noise * delta).dot(input)
        elif direction == "-":
            return (self.theta - self.noise * delta).dot(input)

    def sample_deltas(self):
        """ generate array of random noise matrices. Length of
            array = num_deltas
        """
        return [
            np.random.randn(*self.theta.shape) for _ in range(self.num_deltas)
        ]

    def update(self, rollouts, std_dev_rewards):
        """ Update policy weights (theta) based on rewards
            from 2*num_deltas rollouts
        """
        # std_dev_rewards is the standard deviation of the rewards
        step = np.zeros(self.theta.shape)
        for r_pos, r_neg, delta in rollouts:
            step += (r_pos - r_neg) * delta
        self.theta += self.learning_rate / (self.num_best_deltas *
                                            std_dev_rewards) * step


class Normalizer():
    """ this ensures that the policy puts equal weight upon
        each state component.

        squeezes inputs between 0 and 1
    """

    # Normalizes the inputs
    def __init__(self, state_dim):
        """ Initialize input space (all zero)
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

    def normalize(self, inputs):
        """ subtract mean state value from current input
            and divide by standard deviation (sqrt(var))
            to normalize
        """
        state_mean = self.mean
        state_std = np.sqrt(self.var)
        return (inputs - state_mean) / state_std


class ARSAgent():
    def __init__(self, state_dim, action_dim, normalizer, policy, env):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.normalizer = normalizer
        self.policy = policy
        self.env = env

    # Deploy Policy in one direction over one whole episode
    # DO THIS ONCE PER ROLLOUT OR DURING DEPLOYMENT
    def deploy(self, direction=None, delta=None):
        state = self.env.reset()
        done = False
        timesteps = 0.0
        sum_rewards = 0.0
        while not done and timesteps < self.policy.episode_steps:
            self.normalizer.observe(state)
            # Normalize State
            state = self.normalizer.normalize(state)
            action = self.policy.evaluate(state, delta, direction)
            state, reward, done, _ = self.env.step(action)
            # Clip reward between -1 and 1 to prevent outliers from
            # distorting weights
            reward = max(min(reward, 1), -1)
            sum_rewards += reward
            timesteps += 1
        return sum_rewards

    def train(self):
        # Sample random noise deltas
        deltas = self.policy.sample_deltas()
        # Initialize +- reward list of size num_deltas
        positive_rewards = [0] * self.policy.num_deltas
        negative_rewards = [0] * self.policy.num_deltas

        # Execute 2*num_deltas rollouts and store +- rewards
        for i in range(self.policy.num_deltas):
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
        rollouts = sorted_rollouts[:self.policy.best_num_deltas]

        # Update Policy
        self.policy.update(rollouts, std_dev_rewards)

        # Execute Current Policy
        eval_reward = self.deploy()
        return eval_reward

    def save(self, filename):
        """ Save the Policy
        """
        with open(
                filename + '_policy', 'wb') as filehandle:
            pickle.dump(self.storage, filehandle)

    def load(self, filename):
        """ Load the Policy
        """
        with open(
                filename + '_policy', 'rb') as filehandle:
            self.storage = pickle.load(filehandle)
