import numpy as np
import gym

import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal

device = torch.device("cuda:1" if torch.cuda.is_available() else "cpu")

# Envs to run in parallel
NUM_ENVS = 1
# Hidden layer neurons
HIDDEN_DIM = 256
# passed to Adam Optimizer
LEARNING_RATE = 1e-4
# Discount factor to calculate returns
GAMMA = 0.99
# Smoothing parameter
GAE_LAMBDA = 0.95
# Clip ratio between new/old policy (+-)
PPO_EPSILON = 0.2
# Scale down critic loss since it is usually > actor loss
CRITIC_DISCOUNT = 0.5
# Improves exploration and generalization
ENTROPY_BETA = 0.001
# Num transistions sampled per iteration per parallel environment
PPO_STEPS = 256
# Num samples randomly selected from full data storage
MINI_BATCH_SIZE = 64
# Epoch = 1 pass over whole training data buffer
PPO_EPOCHS = 10
TARGET_REWARD = 2500


class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=HIDDEN_DIM, std=0.0):
        super(ActorCritic, self).__init__()

        # Notice cleaner implementation compared to TD3 lib
        # forward fcn is much simpler

        # Actor Network
        self.actor = nn.Sequential(
            # STATE -> HIDDEN
            nn.Linear(state_dim, hidden_dim),
            # ACTIVATION
            nn.ReLU(),
            # ACTIVATED LAYER -> ACTION
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh())

        # Critic Network
        self.critic = nn.Sequential(
            # STATE -> HIDDEN
            nn.Linear(state_dim, hidden_dim),
            # ACTIVATION
            nn.ReLU(),
            # ACTIVATED LAYER -> VALUE(scalar)
            nn.Linear(hidden_dim, 1))

        # Log Standard Deviation for Generating Stochastic Policy
        self.log_std = nn.Parameter(torch.ones(1, action_dim) * std)

        # Initialize Weights and Biases
        self.apply(self.init_weights)

    def init_weights(self, m):
        """ Initialize Weights and Biases
    	"""
        if isinstance(m, nn.Linear):
            nn.init.normal_(m.weight, mean=0., std=0.1)
            nn.init.constant_(m.bias, 0.1)

    def forward(self, state):
        """ State --> (stochastic) Action and Value
    	"""
        value = self.critic(state)
        mu = self.actor(state)
        std = self.log_std.exp().expand_as(mu)
        dist = Normal(mu, std)
        return dist, value


class PPO():
    def __init__(self,
                 ac,
                 learning_rate=LEARNING_RATE,
                 gamma=GAMMA,
                 gae_lambda=GAE_LAMBDA,
                 ppo_epsilon=PPO_EPSILON,
                 critic_discount=CRITIC_DISCOUNT,
                 entropy_beta=ENTROPY_BETA,
                 ppo_steps=PPO_STEPS,
                 mini_batch_size=MINI_BATCH_SIZE,
                 ppo_epochs=PPO_EPOCHS):
        self.ac = ac
        self.learning_rate = learning_rate
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.ppo_epsilon = ppo_epsilon
        self.critic_discount = critic_discount
        self.entropy_beta = entropy_beta
        self.ppo_steps = ppo_steps
        self.mini_batch_size = mini_batch_size
        self.ppo_epochs = ppo_epochs

        # Adam Optimizer
        self.optimizer = optim.Adam(ac.parameters(), lr=self.learning_rate)

    def train(self, state, envs):
        # TRAINING DATA STORAGE
        log_probs = []
        values = []
        states = []
        actions = []
        rewards = []
        done_masks = []

        # Each PPO_STEP generates s,a,r,s',d from each env
        for _ in range(PPO_STEPS):
            state = torch.FloatTensor(state).to(device)
            # Get stochastic action and value from state using ac
            dist, value = self.ac(state)

            action = torch.clamp(dist.sample(), -1, 1)
            # each state, reward, done is a list of results from each parallel environment
            next_state, reward, done, _ = envs.step(action.cpu().numpy())
            # Calculate the log probability of an action given a state
            log_prob = dist.log_prob(action)

            # STORE training data
            # Each list is PPO_STEPS long and each entry in the list is NUM_ENVS wide
            # Log probs
            log_probs.append(log_prob)
            # Critic Values
            values.append(value)
            # Rewards
            rewards.append(torch.FloatTensor(reward).unsqueeze(1).to(device))
            # Done done_masks
            done_masks.append(
                torch.FloatTensor(1 - done).unsqueeze(1).to(device))

            states.append(state)
            actions.append(action)

            state = next_state

        # Run the final next_state through the network to get its value
        # This is to properly calculate the returns
        next_state = torch.FloatTensor(next_state).to(device)
        _, next_value = self.ac(next_state)

        # Calculate Generalized Advantage Estimation
        returns = self.compute_gae(next_value, rewards, done_masks, values)

        # Concatenate returns from GAE to Torch Tensor (so 1D list instead of 2D but longer)
        returns = torch.cat(returns).detach()
        log_probs = torch.cat(log_probs).detach()
        values = torch.cat(values).detach()
        states = torch.cat(states)
        actions = torch.cat(actions)
        # Subtract values from returns to get advantages (remember we added those in for returns)
        advantage = returns - values
        # Normalize Advantages (-mean)/std
        advantage = self.normalize(advantage)

        # Update Policy
        self.update(states, actions, log_probs, returns, advantage)

        return state

    def deploy(self, env, deterministic=True):
        state = env.reset()
        done = False
        total_reward = 0
        while not done:
            state = torch.FloatTensor(state).unsqueeze(0).to(device)
            dist, _ = self.ac(state)
            action = np.clip(dist.mean.detach().cpu().numpy()[0], -1, 1) if deterministic \
                else np.clip(dist.sample().cpu().numpy()[0], -1, 1)
            next_state, reward, done, _ = env.step(action)
            state = next_state
            total_reward += reward
        return total_reward

    def normalize(self, x):
        x -= x.mean()
        x /= (x.std() + 1e-8)
        return x

    def compute_gae(self, next_value, rewards, done_masks, values):
        values = values + [next_value]
        gae = 0
        returns = []
        # Loop backwards from latest experience to first experience
        for step in reversed(range(len(rewards))):
            # NOTE: multiplying by a mask of 0 (done), we don't consider the value of the next
            # State since the current state is terminal
            delta = rewards[step] + self.gamma * values[
                step + 1] * done_masks[step] - values[step]
            # GAE is a moving average of advantages discounted by gamma * gae_lam
            gae = delta + self.gamma * self.gae_lambda * done_masks[step] * gae

            # To get the return, we add the value of the state we previously subtracted
            # prepend to get correct order back
            returns.insert(0, gae + values[step])
        return returns

    def rollouts(self, states, actions, log_probs, returns, advantage):
        batch_size = states.size(0)
        # generates random mini-batches until we have covered the full batch
        for _ in range(batch_size // self.mini_batch_size):
            rand_ids = np.random.randint(0, batch_size, self.mini_batch_size)
            yield states[rand_ids, :], actions[rand_ids, :], log_probs[
                rand_ids, :], returns[rand_ids, :], advantage[rand_ids, :]

    def update(self, states, actions, log_probs, returns, advantages):

        # PPO EPOCHS is the number of times we will go through ALL the training data to make updates
        for _ in range(self.ppo_epochs):
            # grabs random mini-batches several times until we have covered all data
            for state, action, old_log_probs, return_, advantage in self.rollouts(
                    states, actions, log_probs, returns, advantages):
                dist, value = self.ac(state)
                entropy = dist.entropy().mean()
                new_log_probs = dist.log_prob(action)

                # To get ratio in log space, subtract then exp
                ratio = (new_log_probs - old_log_probs).exp()
                # Surrogate 1: r*A
                surr1 = ratio * advantage
                # Surrogate 2: r_clip*A
                surr2 = torch.clamp(ratio, 1.0 - self.ppo_epsilon,
                                    1.0 + self.ppo_epsilon) * advantage
                # Loss is min of surr1,surr2 (mean over all envs)
                actor_loss = -torch.min(surr1, surr2).mean()
                # MSE between GAE returns and Critic-estimated state value
                critic_loss = (return_ - value).pow(2).mean()

                # Final loss for backprop
                loss = self.critic_discount * critic_loss + actor_loss - self.entropy_beta * entropy

                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
