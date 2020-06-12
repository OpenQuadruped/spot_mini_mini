import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Normal

class SoftQNetwork(nn.Module):
    def __init__(self, num_inputs, num_actions, hidden_size, init_w=3e-3):
        super(SoftQNetwork, self).__init__()
        self.q1 = nn.Sequential(
            nn.Linear(num_inputs + num_actions, hidden_size), nn.ReLU(),
            nn.Linear(hidden_size, hidden_size), nn.ReLU(),
            nn.Linear(hidden_size, 1)
        )
        self.q2 = nn.Sequential(
            nn.Linear(num_inputs + num_actions, hidden_size), nn.ReLU(),
            nn.Linear(hidden_size, hidden_size), nn.ReLU(),
            nn.Linear(hidden_size, 1)
        )

    def forward(self, state, action):
        state_action = torch.cat([state, action], 1)
        return self.q1(state_action), self.q2(state_action)
