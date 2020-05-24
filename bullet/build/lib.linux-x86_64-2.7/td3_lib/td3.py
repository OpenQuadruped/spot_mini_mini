#!/usr/bin/env python

import copy
import pickle
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import gym
import os

# Twin Delayed Deterministic Policy Gradient

# Algorithm Steps

# 1. Initiailize Networks


class Actor(nn.Module):
    """Initialize parameters and build model.
       An nn.Module contains layers, and a method
       forward(input)that returns the output.
       Weights (learnable params) are inherently defined here.

        Args:
            state_dim (int): Dimension of each state
            action_dim (int): Dimension of each action
            max_action (float): highest action to take

        Return:
            action output of network with tanh activation
    """
    def __init__(self, state_dim, action_dim, max_action):
        # Super calls the nn.Module Constructor
        super(Actor, self).__init__()
        # input layer
        self.fc1 = nn.Linear(state_dim, 256)
        # hidden layer
        self.fc2 = nn.Linear(256, 256)
        # output layer
        self.fc3 = nn.Linear(256, action_dim)
        # wrap from -max to +max
        self.max_action = max_action

    def forward(self, state):
        # You just have to define the forward function,
        # and the backward function (where gradients are computed)
        # is automatically defined for you using autograd.
        # Learnable params can be accessed using Actor.parameters

        # Here, we create the tensor architecture
        # state into layer 1
        a = F.relu(self.fc1(state))
        # layer 1 output into layer 2
        a = F.relu(self.fc2(a))
        # layer 2 output into layer 3 into tanh activation
        return self.max_action * torch.tanh(self.fc3(a))


class Critic(nn.Module):
    """Initialize parameters and build model.
        Args:
            state_dim (int): Dimension of each state
            action_dim (int): Dimension of each action

        Return:
            value output of network
    """
    def __init__(self, state_dim, action_dim):
        # Super calls the nn.Module Constructor
        super(Critic, self).__init__()

        # Q1 architecture
        self.fc1 = nn.Linear(state_dim + action_dim, 256)
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, 1)

        # Q2 architecture
        self.fc4 = nn.Linear(state_dim + action_dim, 256)
        self.fc5 = nn.Linear(256, 256)
        self.fc6 = nn.Linear(256, 1)

    def forward(self, state, action):
        # concatenate state and actions by adding rows
        # to form 1D input layer
        sa = torch.cat([state, action], 1)

        # s,a into input layer into relu activation
        q1 = F.relu(self.fc1(sa))
        # l1 output into l2 into relu activation
        q1 = F.relu(self.fc2(q1))
        # l2 output into l3
        q1 = self.fc3(q1)

        # s,a into input layer into relu activation
        q2 = F.relu(self.fc4(sa))
        # l4 output into l5 into relu activation
        q2 = F.relu(self.fc5(q2))
        # l5 output into l6
        q2 = self.fc6(q2)
        return q1, q2

    def Q1(self, state, action):
        # Return Q1 for gradient Ascent on Actor
        # Note that only Q1 is used for Actor Update

        # concatenate state and actions by adding rows
        # to form 1D input layer
        sa = torch.cat([state, action], 1)

        # s,a into input layer into relu activation
        q1 = F.relu(self.fc1(sa))
        # l1 output into l2 into relu activation
        q1 = F.relu(self.fc2(q1))
        # l2 output into l3
        q1 = self.fc3(q1)
        return q1


# https://github.com/openai/baselines/blob/master/baselines/deepq/replay_buffer.py
# Expects tuples of (state, next_state, action, reward, done)
class ReplayBuffer(object):
    """Buffer to store tuples of experience replay"""
    def __init__(self, max_size=1000000):
        """
        Args:
            max_size (int): total amount of tuples to store
        """

        self.storage = []
        self.max_size = max_size
        self.ptr = 0
        my_path = os.path.abspath(os.path.dirname(__file__))
        self.buffer_path = os.path.join(my_path, "../../replay_buffer")

    def add(self, data):
        """Add experience tuples to buffer

        Args:
            data (tuple): experience replay tuple
        """

        if len(self.storage) == self.max_size:
            self.storage[int(self.ptr)] = data
            self.ptr = (self.ptr + 1) % self.max_size
        else:
            self.storage.append(data)

    def save(self, iterations):

        if not os.path.exists(self.buffer_path):
            os.makedirs(self.buffer_path)

        with open(
                self.buffer_path + '/' + 'replay_buffer_' + str(iterations) +
                '.data', 'wb') as filehandle:
            pickle.dump(self.storage, filehandle)

    def load(self, iterations):

        with open(
                self.buffer_path + '/' + 'replay_buffer_' + str(iterations) +
                '.data', 'rb') as filehandle:
            self.storage = pickle.load(filehandle)

    def sample(self, batch_size):
        """Samples a random amount of experiences from buffer of batch size
           NOTE: We don't delete samples here, only overwrite when max_size

        Args:
            batch_size (int): size of sample
        """
        device = torch.device("cuda:1" if torch.cuda.is_available() else "cpu")

        ind = np.random.randint(0, len(self.storage), size=batch_size)
        states, actions, next_states, rewards, dones = [], [], [], [], []

        for i in ind:
            s, a, s_, r, d = self.storage[i]
            states.append(np.array(s, copy=False))
            state = torch.FloatTensor(np.array(states)).to(device)
            actions.append(np.array(a, copy=False))
            action = torch.FloatTensor(np.array(actions)).to(device)
            next_states.append(np.array(s_, copy=False))
            next_state = torch.FloatTensor(np.array(next_states)).to(device)
            rewards.append(np.array(r, copy=False))
            reward = torch.FloatTensor(np.array(rewards).reshape(-1,
                                                                 1)).to(device)
            dones.append(np.array(d, copy=False))
            not_done = torch.FloatTensor(
                1. - (np.array(dones).reshape(-1, 1))).to(device)

        return state, action, next_state, reward, not_done


class TD3Agent(object):
    """Agent class that handles the training of the networks and
       provides outputs as actions

        Args:
            state_dim (int): state size
            action_dim (int): action size
            max_action (float): highest action to take
            device (device): cuda or cpu to process tensors
            env (env): gym environment to use
            batch_size(int): batch size to sample from replay buffer
            discount (float): discount factor
            tau (float): soft update for main networks to target networks

    """
    def __init__(self,
                 state_dim,
                 action_dim,
                 max_action,
                 discount=0.99,
                 tau=0.005,
                 policy_noise=0.2,
                 noise_clip=0.5,
                 policy_freq=2):

        self.device = torch.device(
            "cuda:1" if torch.cuda.is_available() else "cpu")

        self.actor = Actor(state_dim, action_dim, max_action).to(self.device)
        self.actor_target = copy.deepcopy(self.actor)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(),
                                                lr=3e-4)

        self.critic = Critic(state_dim, action_dim).to(self.device)
        self.critic_target = copy.deepcopy(self.critic)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(),
                                                 lr=3e-4)

        self.max_action = max_action
        self.discount = discount
        self.tau = tau
        self.policy_noise = policy_noise
        self.noise_clip = noise_clip
        self.policy_freq = policy_freq

        self.total_it = 0

    def select_action(self, state):
        """Select an appropriate action from the agent policy

            Args:
                state (array): current state of environment

            Returns:
                action (float): action clipped within action range

        """
        # Turn float value into a CUDA Float Tensor
        state = torch.FloatTensor(state.reshape(1, -1)).to(self.device)
        action = self.actor(state).cpu().data.numpy().flatten()

        return action

    def train(self, replay_buffer, batch_size=100):
        """Train and update actor and critic networks

            Args:
                replay_buffer (ReplayBuffer): buffer for experience replay
                batch_size(int): batch size to sample from replay buffer\

            Return:
                actor_loss (float): loss from actor network
                critic_loss (float): loss from critic network

        """
        self.total_it += 1

        # Sample replay buffer
        state, action, next_state, reward, not_done = replay_buffer.sample(
            batch_size)

        with torch.no_grad():
            """
            Autograd: if you set its attribute .requires_gras as True,
            (DEFAULT)
            it tracks all operations on it. When you finish your
            computation, call .backward() to have all gradients computed
            automatically. The gradient for this tensor is then accumulated
            into the .grad attribute.

            To prevent tracking history and using memory, wrap the code block
            in "with torch.no_grad()". This is heplful when evaluating a model
            as it may have trainable params with requires_grad=True (DEFAULT),
            but for which we don't need the gradients.

            Here, we don't want to track the acyclic graph's history
            when getting our next action because we DON'T want to train
            our actor in this step. We train our actor ONLY when we perform
            the periodic policy update. Could have done .detach() at
            target_Q = reward + not_done * self.discount * target_Q
            for the same effect
            """

            # Select action according to policy and add clipped noise
            noise = (torch.randn_like(action) * self.policy_noise).clamp(
                -self.noise_clip, self.noise_clip)

            next_action = (self.actor_target(next_state) + noise).clamp(
                -self.max_action, self.max_action)

            # Compute the target Q value
            target_Q1, target_Q2 = self.critic_target(next_state, next_action)
            target_Q = torch.min(target_Q1, target_Q2)
            target_Q = reward + not_done * self.discount * target_Q

        # Get current Q estimates
        current_Q1, current_Q2 = self.critic(state, action)

        # Compute critic loss
        # A loss function takes the (output, target) pair of inputs,
        # and computes a value that estimates how far away the output
        # is from the target.
        critic_loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(
            current_Q2, target_Q)

        # Optimize the critic
        # Zero the gradient buffers of all parameters
        self.critic_optimizer.zero_grad()
        # Backprops with random gradients
        # When we call loss.backward(), the whole graph is differentiated
        # w.r.t. the loss, and all Tensors in the graph that has
        # requires_grad=True (DEFAULT) will have their .grad Tensor
        # accumulated with the gradient.
        critic_loss.backward()
        # Does the update
        self.critic_optimizer.step()

        # Delayed policy updates
        if self.total_it % self.policy_freq == 0:

            # Compute actor losse
            actor_loss = -self.critic.Q1(state, self.actor(state)).mean()

            # Optimize the actor
            # Zero the gradient buffers
            self.actor_optimizer.zero_grad()
            # Differentiate the whole graph wrt loss
            actor_loss.backward()
            # Does the update
            self.actor_optimizer.step()

            # Update target networks (Critic 1, Critic 2, Actor)
            for param, target_param in zip(self.critic.parameters(),
                                           self.critic_target.parameters()):
                target_param.data.copy_(self.tau * param.data +
                                        (1 - self.tau) * target_param.data)

            for param, target_param in zip(self.actor.parameters(),
                                           self.actor_target.parameters()):
                target_param.data.copy_(self.tau * param.data +
                                        (1 - self.tau) * target_param.data)

    def save(self, filename):
        torch.save(self.critic.state_dict(), filename + "_critic")
        torch.save(self.critic_optimizer.state_dict(),
                   filename + "_critic_optimizer")
        torch.save(self.actor.state_dict(), filename + "_actor")
        torch.save(self.actor_optimizer.state_dict(),
                   filename + "_actor_optimizer")

    def load(self, filename):
        self.critic.load_state_dict(
            torch.load(filename + "_critic", map_location=self.device))
        self.critic_optimizer.load_state_dict(
            torch.load(filename + "_critic_optimizer",
                       map_location=self.device))
        self.actor.load_state_dict(
            torch.load(filename + "_actor", map_location=self.device))
        self.actor_optimizer.load_state_dict(
            torch.load(filename + "_actor_optimizer",
                       map_location=self.device))


# Runs policy for X episodes and returns average reward
# A fixed seed is used for the eval environment
def evaluate_policy(policy, env_name, seed, eval_episodes=10, render=False):
    """run several episodes using the best agent policy

        Args:
            policy (agent): agent to evaluate
            env (env): gym environment
            eval_episodes (int): how many test episodes to run
            render (bool): show training

        Returns:
            avg_reward (float): average reward over the number of evaluations

    """
    eval_env = gym.make(env_name, render=render)
    eval_env.seed(seed + 100)

    avg_reward = 0.
    for _ in range(eval_episodes):
        state, done = eval_env.reset(), False
        while not done:
            # if render:
            #     eval_env.render()
                # sleep(0.01)
            action = policy.select_action(np.array(state))
            state, reward, done, _ = eval_env.step(action)
            avg_reward += reward

    avg_reward /= eval_episodes

    print("---------------------------------------")
    print("Evaluation over {} episodes: {}".format(eval_episodes, avg_reward))
    print("---------------------------------------")
    if render:
        eval_env.close()
    return avg_reward


def trainer(env_name,
            seed,
            max_timesteps,
            start_timesteps,
            expl_noise,
            batch_size,
            eval_freq,
            save_model,
            file_name="best_avg"):
    """ Test Script on stock OpenAI Gym Envs
    """

    if not os.path.exists("../results"):
        os.makedirs("../results")

    if not os.path.exists("../models"):
        os.makedirs("../models")

    env = gym.make(env_name)

    # Set seeds
    env.seed(seed)
    torch.manual_seed(seed)
    np.random.seed(seed)

    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    max_action = float(env.action_space.high[0])

    policy = TD3Agent(state_dim, action_dim, max_action)
    replay_buffer = ReplayBuffer()

    # Evaluate untrained policy and init list for storage
    evaluations = [evaluate_policy(policy, env_name, seed, 1)]

    state = env.reset()
    done = False
    episode_reward = 0
    episode_timesteps = 0
    episode_num = 0

    for t in range(int(max_timesteps)):

        episode_timesteps += 1

        # Select action randomly or according to policy
        # Random Action - no training yet, just storing in buffer
        if t < start_timesteps:
            action = env.action_space.sample()
        else:
            # According to policy + Exploraton Noise
            action = (policy.select_action(np.array(state)) + np.random.normal(
                0, max_action * expl_noise, size=action_dim)).clip(
                    -max_action, max_action)

        # Perform action
        next_state, reward, done, _ = env.step(action)
        done_bool = float(
            done) if episode_timesteps < env._max_episode_steps else 0

        # Store data in replay buffer
        replay_buffer.add((state, action, next_state, reward, done_bool))

        state = next_state
        episode_reward += reward

        # Train agent after collecting sufficient data for buffer
        if t >= start_timesteps:
            policy.train(replay_buffer, batch_size)

        if done:
            # +1 to account for 0 indexing.
            # +0 on ep_timesteps since it will increment +1 even if done=True
            print(
                "Total T: {} Episode Num: {} Episode T: {} Reward: {}".format(
                    t + 1, episode_num, episode_timesteps, episode_reward))
            # Reset environment
            state, done = env.reset(), False
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1

        # Evaluate episode
        if (t + 1) % eval_freq == 0:
            evaluations.append(evaluate_policy(policy, env_name, seed, 1))
            np.save("../results/" + str(file_name) + str(t), evaluations)
            if save_model:
                policy.save("../models/" + str(file_name) + str(t))


if __name__ == "__main__":
    """ The Main Function """
    trainer("BipedalWalker-v2", 0, 1e6, 1e4, 0.1, 100, 15e3, True)