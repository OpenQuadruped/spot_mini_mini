import math
import random
import numpy as np

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Normal


class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim, std=0.0):
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
        )

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
