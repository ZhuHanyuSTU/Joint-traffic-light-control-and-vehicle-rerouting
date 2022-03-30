import copy
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable

device = torch.device("cpu")

# Implementation of Twin Delayed Deep Deterministic Policy Gradients (TD3)
# Paper: https://arxiv.org/abs/1802.09477

class Actor(nn.Module):
	def __init__(self, state_dim, action_dim, max_action):
		super(Actor, self).__init__()

		self.l1 = nn.Linear(state_dim, 256)
		self.bn1 = nn.BatchNorm1d(256)
		self.l2 = nn.Linear(256, 256)
		self.bn2 = nn.BatchNorm1d(256)
		self.l3_1 = nn.Linear(256, int(action_dim/2))
		self.l3_2 = nn.Linear(256, int(action_dim/2))
		self.max_action = max_action
		

	def forward(self, state):
		a = F.relu(self.bn1(self.l1(state)))
		a = F.relu(self.bn2(self.l2(a)))
		a_1 = F.softmax(self.l3_1(a),dim=1)-0.5
		a_2 = 0.5 * F.tanh(self.l3_2(a))

		a = torch.cat((a_1,a_2),dim=1)
		return a


class Critic(nn.Module):
	def __init__(self, state_dim, action_dim):
		super(Critic, self).__init__()

		# Q1 architecture
		self.l1 = nn.Linear(state_dim , 256)
		self.bn1 = nn.BatchNorm1d(256)
		self.l2 = nn.Linear(256+ action_dim, 256)
		self.l3 = nn.Linear(256, 1)
		# Q2 architecture
		self.l4 = nn.Linear(state_dim, 256)
		self.bn4 = nn.BatchNorm1d(256)
		self.l5 = nn.Linear(256+ action_dim, 256)
		self.l6 = nn.Linear(256, 1)

	def forward(self, state, action):
		s1 = F.relu(self.bn1(self.l1(state)))
		q1 = F.relu(self.l2(torch.cat([s1, action], 1)))
		q1 = self.l3(q1)

		s2 = F.relu(self.bn4(self.l4(state)))
		q2 = F.relu(self.l5(torch.cat([s2, action], 1)))
		q2 = self.l6(q2)
		return q1, q2


	def Q1(self, state, action):
		s1 = F.relu(self.bn1(self.l1(state)))
		q1 = F.relu(self.l2(torch.cat([s1, action], 1)))
		q1 = self.l3(q1)
		return q1


def hard_update(target, source):
    for target_param, param in zip(target.parameters(), source.parameters()):
           target_param.data.copy_(param.data)


class TD3(object):
	def __init__(
		self,
		state_dim,
		action_dim,
		max_action,
		discount=0.99,
		tau=0.005,
		policy_noise=0.2,
		noise_clip=0.5,
		policy_freq=2
	):

		self.actor = Actor(state_dim, action_dim, max_action).to(device)
		self.actor_perturbed = Actor(state_dim, action_dim, max_action).to(device)
		self.actor_target = copy.deepcopy(self.actor)
		self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=3e-4)

		self.critic = Critic(state_dim, action_dim).to(device)
		self.critic_target = copy.deepcopy(self.critic)
		self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=3e-4)

		self.max_action = max_action
		self.discount = discount
		self.tau = tau
		self.policy_noise = policy_noise
		self.noise_clip = noise_clip
		self.policy_freq = policy_freq

		self.total_it = 0


	def select_action(self, state,test=False):
		state = torch.FloatTensor(state.reshape(1, -1)).to(device)
		inp = Variable(state,requires_grad=False).to(device)
		self.actor.eval()
		self.actor_perturbed.eval()
		if test==False:
			a=self.actor_perturbed(inp).cpu().data.numpy().flatten()
		else:
			a=self.actor(inp).cpu().data.numpy().flatten()
		self.actor.train()
		return a

	def perturb_actor_parameters(self, param_noise):
		"""Apply parameter noise to actor model, for exploration"""
		hard_update(self.actor_perturbed, self.actor)
		params = self.actor_perturbed.state_dict()
		for name in params:
			if 'bn' in name: 
				pass 
			else:
				param = params[name]
				random = torch.randn(param.shape)
				param += random * param_noise.current_stddev



	def train(self, replay_buffer, batch_size=100):
		self.total_it += 1

		# Sample replay buffer 
		state, action, next_state, reward, not_done = replay_buffer.sample(batch_size)

		with torch.no_grad():

			next_action = (self.actor_target(next_state)).clamp(-self.max_action, self.max_action)

			target_Q1, target_Q2 = self.critic_target(next_state, next_action)
			target_Q = torch.min(target_Q1, target_Q2)
			target_Q = reward + not_done * self.discount * target_Q

		# Get current Q estimates
		current_Q1, current_Q2 = self.critic(state, action)

		# Compute critic loss
		critic_loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)

		# Optimize the critic
		self.critic_optimizer.zero_grad()
		critic_loss.backward()
		self.critic_optimizer.step()
		# self.critic_LR_scheduler.step()
		# Delayed policy updates
		if self.total_it % self.policy_freq == 0:

			# Compute actor losse
			actor_loss = -self.critic.Q1(state, self.actor(state)).mean()
			
			# Optimize the actor 
			self.actor_optimizer.zero_grad()
			actor_loss.backward()
			self.actor_optimizer.step()
			# self.actor_LR_scheduler.step()

			# Update the frozen target models
			for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
				target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

			for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
				target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)


	def save(self, filename):
		torch.save(self.critic.state_dict(), filename + "_critic")
		torch.save(self.critic_optimizer.state_dict(), filename + "_critic_optimizer")
		
		torch.save(self.actor.state_dict(), filename + "_actor")
		torch.save(self.actor_optimizer.state_dict(), filename + "_actor_optimizer")


	def load(self, filename):
		self.critic.load_state_dict(torch.load(filename + "_critic"))
		self.critic_optimizer.load_state_dict(torch.load(filename + "_critic_optimizer"))
		self.critic_target = copy.deepcopy(self.critic)

		self.actor.load_state_dict(torch.load(filename + "_actor"))
		self.actor_optimizer.load_state_dict(torch.load(filename + "_actor_optimizer"))
		self.actor_target = copy.deepcopy(self.actor)

