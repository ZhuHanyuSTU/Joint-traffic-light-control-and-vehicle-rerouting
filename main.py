
import json
import time
SEED = 1000

noise_period = 500
if noise_period==100:
    explore_decay_factor = 0.75 #original NoRevise 0.5; Revise 100:0.75 200:0.65 500:0.3
elif noise_period==200:
    explore_decay_factor = 0.65
elif noise_period==500:
    explore_decay_factor = 0.3
final_scale = 0.3 #original 0.3; Revise 0.3 还要修改最小值



import random
random.seed(SEED)
import numpy as np
np.random.seed(SEED)
import torch
torch.manual_seed(SEED)
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import os
from sys import platform
import xml.etree.ElementTree as ET
from SUMOAgent_reviseTT_revise0210 import SUMO_Agent, close_sumo
import shutil
from FunctionalPackage import PathSet, state2input
import scipy.io as scio
import TD3_alg
import utils


gamma = 0.99
conf_record = "one_exp"
print(platform)
if platform == "win32":

    sumoBinary = r"E:/Program_Files/Sumo/bin/sumo-gui"
    sumoCmd = [sumoBinary,
               '-c',
               r'{0}/data/{1}/cross_equal0.2.sumocfg'.format(os.path.split(os.path.realpath(__file__))[0], conf_record)]

    sumoBinary_nogui = r"E:/Program_Files/Sumo/bin/sumo"
    sumoCmd_nogui = [sumoBinary_nogui,
                     '-c',
                     r'{0}/data/{1}/cross_equal0.2.sumocfg'.format(
                         os.path.split(os.path.realpath(__file__))[0], conf_record)]
    sumo_cmd_str = sumoCmd
    batch_size = 128
elif platform == "linux" or platform == "linux2":# this is linux

    sumoBinary_nogui = '/usr/bin/sumo'
    sumoCmd_nogui = [sumoBinary_nogui,
               '-c',
               r'{0}/data/{1}/cross_equal0.2.sumocfg'.format(os.path.split(os.path.realpath(__file__))[0], conf_record),'--seed',str(SEED)]
    # cross_equal0.2  cross_equal0.4   cross_unequal0.40.2    cross_various_flows_4_0.40.2
    sumo_cmd_str = sumoCmd_nogui
    batch_size = 128
direc = '1_equallow/'
max_steps = 120  
def log_record(log_str, log_file):
    f_log = open(log_file, "a")
    f_log.write(log_str)
    f_log.close()


device = torch.device("cpu")


max_episodes = 1000

reset_epi = max_episodes
start_episodes = 20
train_start_buffer = max_steps*start_episodes

learning_step = 0

state_dim = 29+20
action_dim = 8
hidden_dim = 256


end_episodes = noise_period
ini_scale = 1


advise_flag = 1
dynamic_flag = 1 

noise = 0.3
noise_flag = 'gau'

prefix = f"TD3_WithLTF_ArrRate0.20.2_NoisePeriod{noise_period}_{SEED}_{time.strftime('%m_%d_%H_%M_%S', time.localtime(time.time()))}"


path_set = PathSet(os.path.join("data", conf_record),
                   os.path.join("records", direc, prefix),
                   os.path.join("model", direc, prefix),
                   os.path.join("eval", direc, prefix))
file_name = os.path.join(path_set.PATH_TO_OUTPUT, "log_reward.txt")
file_name_loss = os.path.join(path_set.PATH_TO_OUTPUT, "log_loss.txt")
eval_file_name = os.path.join(path_set.PATH_TO_EVAL, "log_evaluation.txt")

shutil.copy(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), 'TD3_torch_network.py'),
    os.path.join(path_set.PATH_TO_OUTPUT, 'TD3_torch_network.py'))
shutil.copy(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), 'SUMOAgent_reviseTT_revise0207_tryNewR.py'),
    os.path.join(path_set.PATH_TO_OUTPUT, 'SUMOAgent_reviseTT_revise0207_tryNewR.py'))

shutil.copy(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), 'TD3_alg.py'),
    os.path.join(path_set.PATH_TO_OUTPUT, 'TD3_alg.py'))

def eval_policy_one_episode(eval_policy_net, SEED):
    eval_sumo_agent = SUMO_Agent(sumo_cmd_str, path_set, action_dim)
    eval_state = eval_sumo_agent.state
    eval_episode_reward = 0
    eval_episode_shunt = np.zeros(4)
    eval_episode_advise1 = 0
    eval_episode_advise2 = np.zeros(4)
    eval_episode_advise3 = 0
    eval_start_time = time.time()

    eval_action_green_allocation_rec = []
    eval_action_reroute_ratio_rec = []
    

    for step in range(max_steps+20):
        eval_action = eval_policy_net.select_action(state2input(eval_state),test=True)
        eval_episode_shunt += 0.5*eval_action[4:]+0.25
        eval_reward = eval_sumo_agent.take_action(0.5*eval_action+0.25, dynamic_flag)  # network output range[-1,1], transfer to [0,1]
        eval_next_state = eval_sumo_agent.state
        np.set_printoptions(formatter={'float': '{: 0.4f}'.format})
        print(f"eval_Episode Step {step}, Action, {1*eval_action[0:4]+0.5}, {0.5*eval_action[4:]+0.25}, Reward, {eval_reward:.4f}")

        eval_state = eval_next_state
        eval_episode_reward += eval_reward
        eval_episode_advise1 += abs(eval_sumo_agent.advise1)
        eval_episode_advise2 += np.abs(np.array(eval_sumo_agent.advise2))
        eval_episode_advise3 += abs(eval_sumo_agent.advise3)
        eval_action_green_allocation_rec.append(eval_action[0:4]+0.5)
        eval_action_reroute_ratio_rec.append(0.5*eval_action[4:]+0.25)

    eval_num_veh = 0
    eval_MainNum_veh = 0
    eval_num_turn = 0
    eval_travel_time_veh = []
    eval_waiting_time_veh = []
    eval_fuel_consumption_veh = []
    eval_all_vehs_info = eval_sumo_agent.all_vehs_info
    for veh_data in eval_all_vehs_info.values():
        if (veh_data[1] / 1000)<(max_steps*30):
            eval_num_veh += 1
            if (veh_data[3] == [1]) or (veh_data[3] == 1):
                eval_MainNum_veh += 1
            if (veh_data[3] == [2]) or (veh_data[3] == 2):
                eval_num_turn += 1
            eval_travel_time_veh.append(veh_data[2] / 1000)
            eval_waiting_time_veh.append(veh_data[0])
            eval_fuel_consumption_veh.append(veh_data[4])
    RerouteRatio = eval_num_turn/(eval_MainNum_veh+eval_num_turn)
    close_sumo()

    eval_stop_time = time.time()
    np.set_printoptions(formatter={'float': '{: 0.4f}'.format})
    eval_reward_str = f"Eval_episode, Episode_reward:{eval_episode_reward:.4f}, TravelTime:{np.mean(np.array(eval_travel_time_veh)):.4f}, WaitTime:{np.mean(np.array(eval_waiting_time_veh)):.4f}, \
FuelConsum:{np.mean(np.array(eval_fuel_consumption_veh)):.4f}, GreenRatio:{np.mean(np.array(eval_action_green_allocation_rec),axis=0)}, VehicleNum:{eval_num_veh}, MainVehicleNum:{eval_MainNum_veh}, RerouteNum:{eval_num_turn}, RealRerouteRatio:{RerouteRatio:.4f} \
Episode_ActionReroutRatio:{eval_episode_shunt/(max_steps+20)}, episode_advise:{eval_episode_advise1} {eval_episode_advise2} {eval_episode_advise3}, RunTime:{(eval_stop_time-eval_start_time):.1f} \n"
    log_record(eval_reward_str, eval_file_name)
    eval_start_time = time.time()

    return eval_episode_reward, np.array(eval_action_green_allocation_rec), np.array(eval_action_reroute_ratio_rec), eval_all_vehs_info, eval_travel_time_veh, eval_waiting_time_veh, eval_fuel_consumption_veh, eval_num_veh, eval_MainNum_veh, eval_num_turn, RerouteRatio
    



def my_softmax(x):
    return np.exp(x)/np.sum(np.exp(x))


class OUNoise(object):
    def __init__(self, action_space, mu=0.0, theta=0.15, max_sigma=noise, min_sigma=0.05, decay_period=1000):
        self.mu = mu
        self.theta = theta
        self.sigma = max_sigma
        self.max_sigma = max_sigma
        self.min_sigma = min_sigma
        self.decay_period = decay_period
        self.action_dim = action_space
        self.low = -0.5
        self.high = 0.5
        self.reset()
        self.state = None

    def reset(self):
        self.state = np.ones(self.action_dim) * self.mu

    def evolve_state(self):
        if noise_flag == 'ou':
            x = self.state
            dx = self.theta * (self.mu - x) + self.sigma * np.random.randn(self.action_dim)
            self.state = x + dx
        else:
            dx = 0 * np.random.randn(self.action_dim)
            self.state = dx
        return self.state

    def get_action(self, action_input, decay, advise1, advise2,advise3, t=0):
        ou_state = np.clip(self.evolve_state(),-0.3,0.3)
        action_out = action_input
        if (advise1 !=0) and (advise3 !=0):
            if (advise1 ==1) and (advise3 ==1):
                action_out[0:4]= np.array([0.6, 0.4, 0, 0])-0.5
            elif (advise1 ==-1) and (advise3 ==1):
                action_out[0:4]= np.array([0, 0.4, 0.6, 0])-0.5
            elif (advise1 ==1) and (advise3 ==-1):
                action_out[0:4]= np.array([0.6, 0, 0, 0.4])-0.5
            elif (advise1 ==-1) and (advise3 ==-1):
                action_out[0:4]= np.array([0, 0, 0.6, 0.4])-0.5
        if (advise1 !=0) and (advise3 ==0):
            if (advise1 ==1):
                action_out[0:4]= np.array([1.0, 0, 0, 0])-0.5
            elif (advise1 ==-1):
                action_out[0:4]= np.array([0, 0, 1.0, 0])-0.5
        if (advise1 ==0) and (advise3 !=0):
            if (advise3 ==1):
                action_out[0:4]= np.array([0, 1.0, 0, 0])-0.5
            elif (advise3 ==-1):
                action_out[0:4]= np.array([0, 0, 0, 1.0])-0.5

        action_out[4:] = np.clip(action_out[4:] + decay * ou_state[4:], self.low, self.high)
        for action_idx in range(4):
            if advise2[action_idx]!=0:
                action_out[4+action_idx] = 0.1
        return action_out
                      



class AdaptiveParamNoiseSpec(object):
    def __init__(self, initial_stddev=0.1, desired_action_stddev=noise, adaptation_coefficient=1.01):
        """
        Note that initial_stddev and current_stddev refer to std of parameter noise, 
        but desired_action_stddev refers to (as name notes) desired std in action space
        """
        self.initial_stddev = initial_stddev
        self.desired_action_stddev = desired_action_stddev
        self.adaptation_coefficient = adaptation_coefficient

        self.current_stddev = initial_stddev

    def adapt(self, distance):
        if distance > self.desired_action_stddev:
            # Decrease stddev.
            self.current_stddev /= self.adaptation_coefficient
        else:
            # Increase stddev.
            self.current_stddev *= self.adaptation_coefficient

    def get_stats(self):
        stats = {
            'param_noise_stddev': self.current_stddev,
        }
        return stats

    def __repr__(self):
        fmt = 'AdaptiveParamNoiseSpec(initial_stddev={}, desired_action_stddev={}, adaptation_coefficient={})'
        return fmt.format(self.initial_stddev, self.desired_action_stddev, self.adaptation_coefficient)

def ddpg_distance_metric(actions1, actions2):
    """
    Compute "distance" between actions taken by two policies at the same states
    Expects numpy arrays
    """
    diff = actions1-actions2
    mean_diff = np.mean(np.square(diff), axis=0)
    dist = np.sqrt(np.mean(mean_diff))
    return dist


def hard_update(target, source):
    for target_param, param in zip(target.parameters(), source.parameters()):
           target_param.data.copy_(param.data)


policy_td3 = TD3_alg.TD3(state_dim,action_dim,0.5,discount=gamma)

replay_buffer_size = 50000
replay_buffer = utils.ReplayBuffer(state_dim, action_dim,max_size=replay_buffer_size)
ou_noise = OUNoise(action_dim)
param_noise = AdaptiveParamNoiseSpec(initial_stddev=0.05,desired_action_stddev=0.3, adaptation_coefficient=1.05)



t1=time.time()
eval_count = 0

evaluations_EpiReward = []
evaluations_ActionGreen_rec = []
evaluations_ActionReroute_rec = []
evaluations_all_vehs_info_rec = []
evaluations_TravelTimeVeh_rec = []
evaluations_WaitTimeVeh_rec = []
evaluations_FuelConVeh_rec = []
evaluations_NumVeh = []
evaluations_MainNumVeh = []
evaluations_NumReroutVeh = []
evaluations_RerouteRatio = []


episode_idx = 1
rewards = []
ave_shunts = []
per_metric = []
start_time = time.time()

time_count = 0

while episode_idx <= max_episodes:
    if episode_idx-1 <= reset_epi:
        sumo_agent = SUMO_Agent(sumo_cmd_str, path_set, action_dim)
    state = sumo_agent.state
    ou_noise.reset()
    policy_td3.perturb_actor_parameters(param_noise)
    noise_counter =0
    episode_reward = 0
    episode_shunt = np.zeros(4)
    episode_advise1 = 0
    episode_advise2 = np.zeros(4)
    episode_advise3 = 0
    id_str = "episode: {0}".format(episode_idx)+'\n'
    log_record(id_str, file_name_loss)

    if episode_idx>end_episodes:
        start_episodes = end_episodes
        end_episodes += noise_period
        ini_scale = ini_scale*explore_decay_factor
    scale_action = max((ini_scale-final_scale)/(start_episodes-end_episodes)*episode_idx+(ini_scale-((ini_scale-final_scale)/(start_episodes-end_episodes)*start_episodes)),final_scale)

    for step in range(max_steps):
        time_count +=1
        
        if (time_count < train_start_buffer):
            action = np.random.rand(action_dim)
            action[0:4] = my_softmax(action[0:4])
            action = action -0.5
        else:
            action = policy_td3.select_action(state2input(state))

        advise1 = advise_flag * sumo_agent.advise1
        advise2 = advise_flag * sumo_agent.advise2
        advise3 = advise_flag * sumo_agent.advise3

        action = ou_noise.get_action(action, scale_action, advise1, advise2,advise3, step)

        episode_shunt += 0.5*action[4:]+0.25


        reward = sumo_agent.take_action(0.5*action+0.25, dynamic_flag)  # network output range[-1,1], transfer to [0,1]
        next_state = sumo_agent.state
        np.set_printoptions(formatter={'float': '{: 0.4f}'.format})
        print(f"Episode {episode_idx}, Step {step}, Action, {1*action[0:4]+0.5}, {0.5*action[4:]+0.25}, Reward, {reward:.4f}")
        replay_buffer.add(state2input(state), action, state2input(next_state), reward, False)
        if (time_count > train_start_buffer) and (dynamic_flag == 1):
            ou_noise.max_sigma = noise
            policy_td3.train(replay_buffer, batch_size)
            learning_step += 1
            noise_counter += 1
        state = next_state
        episode_reward += reward
        episode_advise1 += abs(sumo_agent.advise1)
        episode_advise2 += abs(np.array(sumo_agent.advise2))
        episode_advise3 += abs(sumo_agent.advise3)
    if episode_idx <= reset_epi:
        np.save(os.path.join(path_set.PATH_TO_OUTPUT, 'all_vehs_info'+str(episode_idx)+'.npy'), sumo_agent.all_vehs_info)
        if (time_count > train_start_buffer):
            if replay_buffer.ptr-noise_counter > 0:
                noise_data_state=replay_buffer.state[replay_buffer.ptr-noise_counter:replay_buffer.ptr,:]
                noise_data_action=replay_buffer.action[replay_buffer.ptr-noise_counter:replay_buffer.ptr,:]
            else:
                noise_data_state = np.concatenate((replay_buffer.state[replay_buffer.ptr-noise_counter+replay_buffer_size:replay_buffer_size,:] ,replay_buffer.state[0:replay_buffer.ptr]))
                noise_data_action = np.concatenate((replay_buffer.action[replay_buffer.ptr-noise_counter+replay_buffer_size:replay_buffer_size,:] ,replay_buffer.action[0:replay_buffer.ptr]))
            perturbed_actions = noise_data_action
            policy_td3.actor.eval()
            unperturbed_actions = policy_td3.actor(torch.FloatTensor(noise_data_state).to(device)).cpu().data.numpy()
            policy_td3.actor.train()
            ddpg_dist = ddpg_distance_metric(perturbed_actions, unperturbed_actions)
            param_noise.desired_action_stddev = scale_action*noise
            param_noise.adapt(ddpg_dist)

            policy_td3.save(os.path.join(path_set.PATH_TO_MODEL, 'model_td3_'+str(episode_idx)))
        close_sumo()


    rewards.append(episode_reward)
    ave_shunts.append(np.mean(episode_shunt)/max_steps)
    scio.savemat(os.path.join(path_set.PATH_TO_OUTPUT, 'epi_data_list.mat'),
                 {'reward_list': rewards, 'ave_shunt': ave_shunts})
    stop_time = time.time()
    np.set_printoptions(formatter={'float': '{: 0.4f}'.format})
    reward_str = f"episode:{episode_idx}, episode_reward:{episode_reward:.4f}, episode_shunt:{episode_shunt/max_steps}, episode_advise:{episode_advise1}{episode_advise2}{episode_advise3} RunTime: {(stop_time-start_time):.1f} \n"
    log_record(reward_str, file_name)
    start_time = time.time()






    if episode_idx % 20 == 0:
        evaluations = eval_policy_one_episode(policy_td3, SEED)
        evaluations_EpiReward.append(evaluations[0])
        evaluations_ActionGreen_rec.append(evaluations[1])
        evaluations_ActionReroute_rec.append(evaluations[2])
        evaluations_all_vehs_info_rec.append(evaluations[3])
        evaluations_TravelTimeVeh_rec.append(evaluations[4])
        evaluations_WaitTimeVeh_rec.append(evaluations[5])
        evaluations_FuelConVeh_rec.append(evaluations[6])
        evaluations_NumVeh.append(evaluations[7])
        evaluations_MainNumVeh.append(evaluations[8])
        evaluations_NumReroutVeh.append(evaluations[9])
        evaluations_RerouteRatio.append(evaluations[10])

        np.save(path_set.PATH_TO_EVAL+"/evaluations_EpiReward", evaluations_EpiReward)
        np.save(path_set.PATH_TO_EVAL+"/evaluations_ActionGreen_rec", evaluations_ActionGreen_rec)
        np.save(path_set.PATH_TO_EVAL+"/ evaluations_ActionReroute_rec",  evaluations_ActionReroute_rec)
        np.save(path_set.PATH_TO_EVAL+"/evaluations_all_vehs_info_rec", evaluations_all_vehs_info_rec)
        np.save(path_set.PATH_TO_EVAL+"/evaluations_TravelTimeVeh_rec", evaluations_TravelTimeVeh_rec)
        np.save(path_set.PATH_TO_EVAL+"/evaluations_WaitTimeVeh_rec", evaluations_WaitTimeVeh_rec)
        np.save(path_set.PATH_TO_EVAL+"/evaluations_FuelConVeh_rec",  evaluations_FuelConVeh_rec)
        np.save(path_set.PATH_TO_EVAL+"/evaluations_NumVeh", evaluations_NumVeh)
        np.save(path_set.PATH_TO_EVAL+"/evaluations_MainNumVeh", evaluations_MainNumVeh)
        np.save(path_set.PATH_TO_EVAL+"/evaluations_NumReroutVeh",  evaluations_NumReroutVeh)
        np.save(path_set.PATH_TO_EVAL+"/evaluations_RerouteRatio", evaluations_RerouteRatio)
        np.save(path_set.PATH_TO_MODEL+"/replay_buffer"+str(episode_idx), {"state":replay_buffer.state,"action":replay_buffer.action,"next_state":replay_buffer.next_state,"reward":replay_buffer.reward,"done":replay_buffer.not_done})
    episode_idx += 1

    
np.save(os.path.join(path_set.PATH_TO_OUTPUT, 'all_vehs_info.npy'), sumo_agent.all_vehs_info)
policy_td3.save(os.path.join(path_set.PATH_TO_MODEL, 'model_td3_'+str(episode_idx)))
scio.savemat(os.path.join(path_set.PATH_TO_OUTPUT, 'epi_data_list.mat'),
             {'reward_list': rewards, 'ave_shunt': ave_shunts})




