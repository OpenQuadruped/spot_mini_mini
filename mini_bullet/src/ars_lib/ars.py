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
            noise=0.03,
            seed=1):

        # Tunable Hyperparameters
        self.learning_rate = learning_rate
        self.num_deltas = num_deltas
        self.num_best_deltas = num_best_deltas
        # there cannot be more best_deltas than there are deltas
        assert self.num_best_deltas <= self.num_deltas
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

    def update(self, rollouts, sigma_rewards):
        """ Update policy weights (theta) based on rewards
            from 2*num_deltas rollouts
        """
        # sigma_rewards is the standard deviation of the rewards
        step = np.zeros(self.theta.shape)
        for r_pos, r_neg, delta in rollouts:
            step += (r_pos - r_neg) * delta
        self.theta += self.learning_rate / (self.num_best_deltas *
                                            sigma_rewards) * step


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
    def __init__(self, state_dim, action_dim, normalizer, policy):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.normalizer = normalizer
        self.policy = policy

    # Explore the policy on one specific direction and over one episode
    # DO THIS ONCE PER ROLLOUT
    def explore(self, direction=None, delta=None):
        state = self.env.reset()
        done = False
        num_plays = 0.0
        sum_rewards = 0.0
        while not done and num_plays < self.policy.episode_length:
            self.normalizer.observe(state)
            state = self.normalizer.normalize(state)
            action = self.policy.evaluate(state, delta, direction)
            state, reward, done, _ = self.env.step(action)
            reward = max(min(reward, 1), -1)
            sum_rewards += reward
            num_plays += 1
        return sum_rewards

    def train(self):
        # initialize the random noise deltas and the positive/negative rewards
        deltas = self.policy.sample_deltas()
        positive_rewards = [0] * self.policy.num_deltas
        negative_rewards = [0] * self.policy.num_deltas

        # play an episode each with positive deltas and negative deltas, collect rewards
        for k in range(self.policy.num_deltas):
            positive_rewards[k] = self.explore(direction="+",
                                               delta=deltas[k])
            negative_rewards[k] = self.explore(direction="-",
                                               delta=deltas[k])

        # Compute the standard deviation of all rewards
        sigma_rewards = np.array(positive_rewards + negative_rewards).std()

        # Sort the rollouts by the max(r_pos, r_neg) and select the deltas with best rewards
        scores = {
            k: max(r_pos, r_neg)
            for k, (r_pos, r_neg
                    ) in enumerate(zip(positive_rewards, negative_rewards))
        }
        order = sorted(scores.keys(),
                       key=lambda x: scores[x],
                       reverse=True)[:self.policy.num_best_deltas]
        rollouts = [(positive_rewards[k], negative_rewards[k], deltas[k])
                    for k in order]

        # Update the policy
        self.policy.update(rollouts, sigma_rewards)

        # Play an episode with the new weights and print the score
        reward_evaluation = self.explore()
        print('Step: ', step, 'Reward: ', reward_evaluation)
