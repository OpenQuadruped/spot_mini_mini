import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Normal

# alg specific imports
from .softQnetwork import SoftQNetwork
from .valuenetwork import ValueNetwork


class SoftActorCritic(object):
    def __init__(self,
                 policy,
                 state_dim,
                 action_dim,
                 replay_buffer,
                 hidden_dim=256,
                 soft_q_lr=3e-3,
                 policy_lr=3e-3,
                 device=torch.device(
                     "cuda:1" if torch.cuda.is_available() else "cpu")):
        self.device = device
        # set up the networks
        self.policy_net = policy.to(device)
        self.soft_q_net = SoftQNetwork(state_dim, action_dim,
                                       hidden_dim).to(device)
        self.target_soft_q_net = SoftQNetwork(state_dim, action_dim,
                                              hidden_dim).to(device)

        # ent coeff
        self.target_entropy = -action_dim
        self.log_ent_coef = torch.FloatTensor(np.log(np.array([.1
                                                               ]))).to(device)
        self.log_ent_coef.requires_grad = True

        # copy the target params over
        for target_param, param in zip(self.target_soft_q_net.parameters(),
                                       self.soft_q_net.parameters()):
            target_param.data.copy_(param.data)

        # set the losses
        self.soft_q_criterion = nn.MSELoss()

        # set the optimizers
        self.soft_q_optimizer = optim.Adam(self.soft_q_net.parameters(),
                                           lr=soft_q_lr)
        self.policy_optimizer = optim.Adam(self.policy_net.parameters(),
                                           lr=policy_lr)
        self.ent_coef_optimizer = optim.Adam([self.log_ent_coef], lr=3e-3)

        # reference the replay buffer
        self.replay_buffer = replay_buffer

        self.log = {'entropy_loss': [], 'q_value_loss': [], 'policy_loss': []}

    def soft_q_update(self, batch_size, gamma=0.99, soft_tau=0.01):
        state, action, reward, next_state, done = self.replay_buffer.sample(
            batch_size)

        state = torch.FloatTensor(state).to(self.device)
        next_state = torch.FloatTensor(next_state).to(self.device)
        action = torch.FloatTensor(action).to(self.device)
        reward = torch.FloatTensor(reward).unsqueeze(1).to(self.device)
        done = torch.FloatTensor(np.float32(done)).unsqueeze(1).to(self.device)

        ent_coef = torch.exp(self.log_ent_coef.detach())
        new_action, log_prob, z, mean, log_std = self.policy_net.evaluate(
            next_state)

        target_q1_value, target_q2_value = self.target_soft_q_net(
            next_state, new_action)
        target_value = reward + (1 - done) * gamma * (
            torch.min(target_q2_value, target_q2_value) - ent_coef * log_prob)

        expected_q1_value, expected_q2_value = self.soft_q_net(state, action)

        q_value_loss = self.soft_q_criterion(expected_q1_value, target_value.detach()) \
                            + self.soft_q_criterion(expected_q2_value, target_value.detach())

        self.soft_q_optimizer.zero_grad()
        q_value_loss.backward()
        self.soft_q_optimizer.step()

        new_action, log_prob, z, mean, log_std = self.policy_net.evaluate(
            state)
        expected_new_q1_value, expected_new_q2_value = self.soft_q_net(
            state, new_action)

        expected_new_q_value = torch.min(expected_new_q1_value,
                                         expected_new_q2_value)

        policy_loss = (ent_coef * log_prob - expected_new_q_value).mean()

        self.policy_optimizer.zero_grad()
        policy_loss.backward()
        self.policy_optimizer.step()

        self.ent_coef_optimizer.zero_grad()
        ent_loss = torch.mean(
            torch.exp(self.log_ent_coef) *
            (-log_prob - self.target_entropy).detach())
        ent_loss.backward()
        self.ent_coef_optimizer.step()

        for target_param, param in zip(self.target_soft_q_net.parameters(),
                                       self.soft_q_net.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - soft_tau) +
                                    param.data * soft_tau)

        self.log['q_value_loss'].append(q_value_loss.item())
        self.log['entropy_loss'].append(ent_loss.item())
        self.log['policy_loss'].append(policy_loss.item())

    def save(self, filename):
        torch.save(self.policy_net.state_dict(), filename + "_policy_net")
        torch.save(self.policy_optimizer.state_dict(),
                   filename + "_policy_optimizer")
        torch.save(self.soft_q_net.state_dict(), filename + "_soft_q_net")
        torch.save(self.soft_q_optimizer.state_dict(),
                   filename + "_soft_q_optimizer")

    def load(self, filename):
        self.policy_net.load_state_dict(
            torch.load(filename + "_policy_net", map_location=self.device))
        self.policy_optimizer.load_state_dict(
            torch.load(filename + "_policy_optimizer",
                       map_location=self.device))
        # self.soft_q_net.load_state_dict(
        #     torch.load(filename + "_soft_q_net", map_location=self.device))
        # self.soft_q_optimizer.load_state_dict(
        #     torch.load(filename + "_soft_q_optimizer",
        #                map_location=self.device))
        # self.target_soft_q_net = copy.deepcopy(self.soft_q_net)
