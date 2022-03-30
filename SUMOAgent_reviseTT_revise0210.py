from sys import platform
import sys
import os
import numpy as np
import shutil
import json
from FunctionalPackage import State
import random
from queue import Queue  # LILO队列
import re
import copy


if platform == "linux" or platform == "linux2":# this is linux
    os.environ['SUMO_HOME'] = '/usr/share/sumo'
    try:
        import traci
        import traci.constants as tc
    except ImportError:
        if "SUMO_HOME" in os.environ:
            print(os.path.join(os.environ["SUMO_HOME"], "tools"))
            sys.path.append(
	            os.path.join(os.environ["SUMO_HOME"], "tools")
	        )
            try:
                import traci
                import traci.constants as tc
            except ImportError:
                raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")
        else:
            raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")

elif platform == "win32":
    os.environ['SUMO_HOME'] = 'C:\\Program Files (x86)\\DLR\\Sumo'

    try:
        import traci
        import traci.constants as tc
    except ImportError:
        if "SUMO_HOME" in os.environ:
            print(os.path.join(os.environ["SUMO_HOME"], "tools"))
            sys.path.append(
                os.path.join(os.environ["SUMO_HOME"], "tools")
            )
            try:
                import traci
                import traci.constants as tc
            except ImportError:
                raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")
        else:
            raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")
elif platform =='darwin':
    os.environ['SUMO_HOME'] = "/Users/{0}/sumo/sumo-git".format(os.getlogin())

    try:
        import traci
        import traci.constants as tc
    except ImportError:
        if "SUMO_HOME" in os.environ:
            print(os.path.join(os.environ["SUMO_HOME"], "tools"))
            sys.path.append(
                os.path.join(os.environ["SUMO_HOME"], "tools")
            )
            try:
                import traci
                import traci.constants as tc
            except ImportError:
                raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")
        else:
            raise EnvironmentError("Please set SUMO_HOME environment variable or install traci as python module!")

else:
    sys.exit("platform error")


CAV_rate = 0.5  # smart rate
carWidth = 3
grid_width = 4
area_length = 1000
travel_time_scale = 300
listLanes = ['edge1-0_1', 'edge1-0_2', 'edge2-0_1', 'edge2-0_2',
                                 'edge3-0_1', 'edge3-0_2', 'edge4-0_1', 'edge4-0_2']
intelli_edges = ['edge1-0', 'edge2-0', 'edge3-0', 'edge4-0']
center_edges = ['-gneE10', '-gneE11', '-gneE12', '-gneE13']
routeID_list = ['routewe', 'routeew', 'routesn', 'routens']
route_lane_id_list = [
    [[['-gneE10_1', '-gneE10_2'], ['edge1-0_1', 'edge1-0_2'], ['edge0-2_1', 'edge0-2_2']],
     [['-gneE10_0'], ['gneE3_2'], ['gneE4_1'], ['gneE5_2'], ['gneE6_0']]],
    [[['-gneE11_1', '-gneE11_2'], ['edge2-0_1', 'edge2-0_2'], ['edge0-1_1', 'edge0-1_2']],
     [['-gneE11_0'], ['gneE7_2'], ['-gneE9_1'], ['-gneE2_2'], ['-gneE1_0']]],
    [[['-gneE12_1', '-gneE12_2'], ['edge3-0_1', 'edge3-0_2'], ['edge0-4_1', 'edge0-4_2']],
     [['-gneE12_0'], ['gneE5_2'], ['gneE6_1'], ['gneE7_2'], ['-gneE9_0']]],
    [[['-gneE13_1', '-gneE13_2'], ['edge4-0_1', 'edge4-0_2'], ['edge0-3_1', 'edge0-3_2']],
     [['-gneE13_0'], ['-gneE2_2'], ['-gneE1_1'], ['gneE3_2'], ['gneE4_0']]]
]
# route_lane_id_list[四个方向中的一个][两条线路中的一个][线路中的不同edge][每个edge上的lanes]
def get_list(my_list, new_list):
    for item in my_list:
        if isinstance(item, list):
            new_list = get_list(item, new_list)
        else:
            # print(item)
            new_list.append(item)
    return new_list

intelli_lanes = [[['edge1-0_1','edge1-0_2'],['edge1-0_3']],
                 [['edge2-0_1','edge2-0_2'],['edge2-0_3']], 
                 [['edge3-0_1','edge3-0_2'],['edge3-0_3']], 
                 [['edge4-0_1','edge4-0_2'],['edge4-0_3']]]

intelli_loops = [[['e1Detector_edge1-0_1_3','e1Detector_edge1-0_2_4'],['e1Detector_edge1-0_3_5']],
                 [['e1Detector_edge2-0_1_15','e1Detector_edge2-0_2_16'],['e1Detector_edge2-0_3_17']], 
                 [['e1Detector_edge3-0_1_28','e1Detector_edge3-0_2_27'],['e1Detector_edge3-0_3_29']], 
                 [['e1Detector_edge4-0_1_37','e1Detector_edge4-0_2_38'],['e1Detector_edge4-0_3_39']]]

intelli_loops_outgoing = [[['e1Detector_edge1-0_1_3_outgoing','e1Detector_edge1-0_2_4_outgoing'],['e1Detector_edge1-0_3_5_outgoing']],
                         [['e1Detector_edge2-0_1_15_outgoing','e1Detector_edge2-0_2_16_outgoing'],['e1Detector_edge2-0_3_17_outgoing']], 
                         [['e1Detector_edge3-0_1_28_outgoing','e1Detector_edge3-0_2_27_outgoing'],['e1Detector_edge3-0_3_29_outgoing']], 
                         [['e1Detector_edge4-0_1_37_outgoing','e1Detector_edge4-0_2_38_outgoing'],['e1Detector_edge4-0_3_39_outgoing']]]


intelli_lanes_list=[]                 
intelli_lanes_list = get_list(intelli_lanes,intelli_lanes_list)
intelli_eff_lane_num = 8




through_lane_id_list = []
through_lane_id_list = get_list(route_lane_id_list, through_lane_id_list)
through_lane_id_list += ['edge1-0_3','edge2-0_3','edge3-0_3','edge4-0_3']


base_travel_time = 500 / 15  # base travel time length/ speed
timeslot_factor=1
reward_weight = 1.0
C = 30*timeslot_factor  # cycle length (sec)
s = 2*1800 / 3600  # vehicles per sec
Step_len = C # seconds
node_light_7 = "node0"
normal_speed = 13
varrho = 0.7

class SUMO_Agent(object):

    def __init__(self, sumo_cmd_str, path_set, action_dim):

        self.path_set = path_set
        self.start_sumo(sumo_cmd_str)

        self.induction_loop_ID_list = traci.inductionloop.getIDList()
        self.model_based_TT = {'0':[],'1':[],'2':[],'3':[]}
        self.speed_based_TT = {'0':[],'1':[],'2':[],'3':[]}
        self.induction_loop_num = dict()

        for loop_id in self.induction_loop_ID_list:
            self.induction_loop_num[loop_id] = Queue()
            for i in range(Step_len*4):
                self.induction_loop_num[loop_id].put(0)

        self.induction_loop_arrival_rate = dict(zip(intelli_lanes_list, list(np.zeros(len(intelli_lanes_list)))))
        self.dic_vehicles = {}
        self.current_phase = 0
        self.current_phase_duration = 0
        self.select_space_length = 50  # 50 grids per lane
        self.advise1 = 0 # straight lane
        if action_dim <= 2:
            self.advise2 = 0
        else:
            self.advise2 = list(np.zeros(4))
        self.advise3 = 0 # turn left lane


        # vehicles information
        self.all_vehs_info = dict()
        # dictionary for record the information of all vehicles in whole simulation
        # there are four elements for each vehicle
        #[accu. wait time, enter time, travel time, type_index(0: in other road; 1: straight in main road; 2: shunt in main road)]

        self.new_vehs = set()
        self.current_all_vehs = set()
        self.main_vehs = set()
        self.main_new_vehs = set()
        self.main_new_vehs_4decision = set()
        self.main_new_turn_vehs = set()
        self.last_step_all_vehs = set()
        self.last_step_main_vehs = set()
        self.over_sau_time = list(np.zeros(8))
        self.straight_num = np.zeros(4)
        self.shunt_num = np.zeros(4)
        self.smart_num = np.zeros(4)
        self.lanes_travel_time_dict = dict()
        self.lanes_veh_Num_time_dict = dict()
        self.lanes_MeanSpeed_dict = dict()
        # self.travel_time_update_lanes = []
        self.MeanSpeed_update_lanes = []
        for lane in through_lane_id_list:
            # self.travel_time_update_lanes.append(lane)
            self.MeanSpeed_update_lanes.append(lane)
            self.lanes_travel_time_dict[lane] = 500/normal_speed
            self.lanes_veh_Num_time_dict[lane]=0
            self.lanes_MeanSpeed_dict[lane] = [normal_speed]
        self.update_state()

        self.share_straight_travel_time = np.zeros(4)
        self.share_reroute_travel_time = np.zeros(4)
        self.real_g_ratio = 1/2 *np.ones(4)


    def start_sumo(self, sumo_cmd_str):
        traci.start(sumo_cmd_str)

    def status_calculator(self):
        # vehs_num,
        # queue_len,
        # current_phase,
        # est_arrival_rate,
        # ave_traval_time:

        for lane in self.MeanSpeed_update_lanes:
            # self.lanes_travel_time_dict[lane].append(np.clip(traci.lane.getTraveltime(lane),0,300))
            Lane_veh_Num = self.lanes_veh_Num_time_dict[lane]
            MeanSpeed = np.mean(self.lanes_MeanSpeed_dict[lane])
            if MeanSpeed ==0:
                est_traval_time =(Lane_veh_Num/70) *300 + 500/normal_speed
            else:
                est_traval_time = 500/MeanSpeed
            self.lanes_travel_time_dict[lane]=np.clip(est_traval_time,0,300)



        edge_NumVehiclesTracker = []
        edge_QueueTracker = []
        edge_arrival_rateTracker = []
        edge_shunt_ave_traval_timeTracker = []
        current_phaseTracker = traci.trafficlight.getPhase(node_light_7)
        edge_straight_ave_traval_timeTracker = []
        edge_straight_intelli_ave_traval_timeTracker = []
        edge_outgoing_rateTracker = []

        # ================ count vehicles in edge
        for eff_lane_idx in range(len(intelli_lanes)):
            straight_double_lanes = intelli_lanes[eff_lane_idx][0]
            lane1_veh_num = traci.lane.getLastStepVehicleNumber(straight_double_lanes[0])/100
            lane2_veh_num = traci.lane.getLastStepVehicleNumber(straight_double_lanes[1])/100
            edge_NumVehiclesTracker.append(lane1_veh_num+lane2_veh_num)
        for eff_lane_idx in range(len(intelli_lanes)):
            leftTurn_single_lanes = intelli_lanes[eff_lane_idx][1]
            lane3_veh_num = traci.lane.getLastStepVehicleNumber(leftTurn_single_lanes[0])/100
            edge_NumVehiclesTracker.append(lane3_veh_num)
        # ================= COUNT HALTED VEHICLES (I.E. QUEUE SIZE)
        for eff_lane_idx in range(len(intelli_lanes)):
            straight_double_lanes = intelli_lanes[eff_lane_idx][0]
            lane1_veh_num = traci.lane.getLastStepHaltingNumber(straight_double_lanes[0])/100
            lane2_veh_num = traci.lane.getLastStepHaltingNumber(straight_double_lanes[1])/100
            edge_QueueTracker.append(lane1_veh_num+lane2_veh_num)
        for eff_lane_idx in range(len(intelli_lanes)):
            leftTurn_single_lanes = intelli_lanes[eff_lane_idx][1]
            lane3_veh_num = traci.lane.getLastStepHaltingNumber(leftTurn_single_lanes[0])/100
            edge_QueueTracker.append(lane3_veh_num)



        # ================= Arrive Rate
        for eff_loop_idx in range(len(intelli_loops)):
            straight_double_lanes = intelli_lanes[eff_loop_idx][0]
            straight_double_loops = intelli_loops[eff_loop_idx][0]
            lane_arrive = np.zeros(2)
            for loop_idx in range(len(straight_double_loops)):
                loop_id = straight_double_loops[loop_idx]
                lane_id = straight_double_lanes[loop_idx]
                last_step_mean_speed = traci.inductionloop.getLastStepMeanSpeed(loop_id)
                last_step_vehs_num = traci.lane.getLastStepVehicleNumber(lane_id)
                if (last_step_mean_speed < 5) and (last_step_vehs_num > 70):
                    lane_arrive[loop_idx] = s/2
                else:
                    lane_arrive[loop_idx]= np.mean(np.array(self.induction_loop_num[loop_id].queue))
            edge_arrival_rateTracker.append(np.sum(lane_arrive))
        for eff_loop_idx in range(len(intelli_loops)):
            leftTurn_single_lanes = intelli_lanes[eff_loop_idx][1]
            leftTurn_single_loops = intelli_loops[eff_loop_idx][1]
            loop_id = leftTurn_single_loops[0]
            lane_id = leftTurn_single_lanes[0]
            last_step_mean_speed = traci.inductionloop.getLastStepMeanSpeed(loop_id)
            last_step_vehs_num = traci.lane.getLastStepVehicleNumber(lane_id)
            if (last_step_mean_speed < 5) and (last_step_vehs_num > 70):
                lane_arrive = s/2
            else:
                lane_arrive= np.mean(np.array(self.induction_loop_num[loop_id].queue))
            edge_arrival_rateTracker.append(lane_arrive)

        # ================= Outgoing Rate
        for eff_loop_idx in range(len(intelli_loops_outgoing)):
            straight_double_lanes = intelli_lanes[eff_loop_idx][0]
            straight_double_loops = intelli_loops_outgoing[eff_loop_idx][0]
            lane_arrive = np.zeros(2)
            for loop_idx in range(len(straight_double_loops)):
                loop_id = straight_double_loops[loop_idx]
                lane_id = straight_double_lanes[loop_idx]
                last_step_mean_speed = traci.inductionloop.getLastStepMeanSpeed(loop_id)
                last_step_vehs_num = traci.lane.getLastStepVehicleNumber(lane_id)
                lane_arrive[loop_idx]= np.mean(np.array(self.induction_loop_num[loop_id].queue))
            edge_outgoing_rateTracker.append(np.sum(lane_arrive))
        for eff_loop_idx in range(len(intelli_loops_outgoing)):
            leftTurn_single_lanes = intelli_lanes[eff_loop_idx][1]
            leftTurn_single_loops = intelli_loops_outgoing[eff_loop_idx][1]
            loop_id = leftTurn_single_loops[0]
            lane_id = leftTurn_single_lanes[0]
            last_step_mean_speed = traci.inductionloop.getLastStepMeanSpeed(loop_id)
            last_step_vehs_num = traci.lane.getLastStepVehicleNumber(lane_id)
            lane_arrive= np.mean(np.array(self.induction_loop_num[loop_id].queue))
            edge_outgoing_rateTracker.append(lane_arrive)


        for route_index in range(len(route_lane_id_list)):
            shunt_route = route_lane_id_list[route_index][1]
            route_travel_time = 0
            for lanes_list in shunt_route:
                lanes_travel = 0
                for lane in lanes_list:
                    lane_travel = self.lanes_travel_time_dict[lane]
                    lanes_travel += lane_travel
                lanes_travel = lanes_travel/len(lanes_list)
                route_travel_time += lanes_travel
            route_travel_time +=  500/15
            edge_shunt_ave_traval_timeTracker.append(route_travel_time/travel_time_scale)

        for route_index in range(len(route_lane_id_list)):
            straight_route = route_lane_id_list[route_index][0]
            straight_route_stat_based = [straight_route[0], straight_route[2]]

            route_travel_time = 0
            for lanes_list in straight_route_stat_based:
                lanes_travel = 0
                for lane in lanes_list:
                    lane_travel = self.lanes_travel_time_dict[lane]
                    lanes_travel += lane_travel
                lanes_travel = lanes_travel/len(lanes_list)
                route_travel_time += lanes_travel
                route_travel_time +=  500/15
            edge_straight_ave_traval_timeTracker.append(route_travel_time/travel_time_scale)


        for route_index in range(len(route_lane_id_list)):
            straight_route = route_lane_id_list[route_index][0]
            straight_route_intelli = [straight_route[1]]
            route_travel_time = 0
            for lanes_list in straight_route_intelli:
                lanes_travel = 0
                for lane in lanes_list:
                    lanes_travel += self.lanes_travel_time_dict[lane]
                lanes_travel = lanes_travel / len(lanes_list)
                route_travel_time += lanes_travel
            edge_straight_intelli_ave_traval_timeTracker.append(route_travel_time/travel_time_scale)
        for eff_lane_idx in range(len(intelli_lanes)):
            leftTurn_single_lanes = intelli_lanes[eff_lane_idx][1]
            lane_id = leftTurn_single_lanes[0]
            lane3_travel = self.lanes_travel_time_dict[lane_id]
            edge_straight_intelli_ave_traval_timeTracker.append(lane3_travel/travel_time_scale)
        return [edge_NumVehiclesTracker, edge_QueueTracker, current_phaseTracker, edge_arrival_rateTracker,
                edge_shunt_ave_traval_timeTracker, edge_straight_ave_traval_timeTracker, edge_straight_intelli_ave_traval_timeTracker,
                edge_outgoing_rateTracker]

    def update_state(self):
        self.status_tracker = self.status_calculator()

        max_we = get_max_queue_length(['edge1-0_1', 'edge1-0_2',  'edge2-0_1', 'edge2-0_2'])
        max_sn = get_max_queue_length(['edge3-0_1', 'edge3-0_2',  'edge4-0_1', 'edge4-0_2'])
        if max_we > 50:
            self.advise1 = 1 *(random.random()>0.5)
        elif max_sn > 50:
            self.advise1 = -1*(random.random()>0.5)
        max_we_turn_left = get_max_queue_length(['edge1-0_3',  'edge2-0_3'])
        max_sn_turn_left  = get_max_queue_length(['edge3-0_3',  'edge4-0_3'])
        if max_we_turn_left  > 50:
            self.advise3 = 1*(random.random()>0.5)
        elif max_sn_turn_left  > 50:
            self.advise3 = -1*(random.random()>0.5)



        self.state = State(vehs_num=np.reshape(np.array(self.status_tracker[0]), newshape=(1, 8)),
                           queue_len=np.reshape(np.array(self.status_tracker[1]), newshape=(1, 8)),
                           est_arrival_rate=np.reshape(np.array(self.status_tracker[3]), newshape=(1, 8)),
                           over_sau_time=np.reshape(np.array(self.over_sau_time)/300, newshape=(1, 8)),
                           ave_shunt_traval_time=np.reshape(np.array(self.status_tracker[4]), newshape=(1, 4)),
                           ave_straight_traval_time=np.reshape(np.array(self.status_tracker[5]), newshape=(1, 4)),
                           ave_itelli_traval_time=np.reshape(np.array(self.status_tracker[6]), newshape=(1, 8)),
                           current_phase=np.reshape(np.array(self.status_tracker[2]), newshape=(1, 1)),)

    def travel_time_model_based(self, g_ratio, v, route_index):
        if route_index>3:
            s_tem = (s/2)
            que_thre = 15
        else:
            s_tem = s
            que_thre = 30

        if self.status_tracker[1][route_index]*100 >que_thre:
            c = self.status_tracker[7][route_index]
        else:
            c = s_tem * g_ratio

        if c==0:
            X = 2
        else:
            X = v/c
        if g_ratio == 1:
            uniform_delay = 0
        else:
            uniform_delay = (C/2)*((1-g_ratio)**2/(1-min(X, 1)*g_ratio))
        if (X < varrho) :
            if X == 0:
                add_delay = 0
            else:
                add_delay = X**2/(2*v*(1-X))
        else:
            X0 = 0.67 + s_tem * g_ratio * C / 600
            if c == 0:
                add_delay = ((2 * self.over_sau_time[route_index] + Step_len) * 1 / 4) * (
                        (X - 1) + np.sqrt((X - 1) ** 2 + (12 * (X - X0) / (1 * (2 * self.over_sau_time[route_index] + Step_len)))))
            else:
                add_delay = ((2 * self.over_sau_time[route_index] + Step_len) * 1 / 4) * (
                        (X - 1) + np.sqrt((X - 1) ** 2 + (12 * (X - X0) / (c * (2 * self.over_sau_time[route_index] + Step_len)))))
        total_travel_time = min(base_travel_time + uniform_delay + add_delay, 300)
        return total_travel_time

    def return_reward(self, g_ratio):

        vehs_num = 0
        travel_time = 0
        travel_time_existing = 0
        vehs_num_existing = 0
        for route_index in range(4):
            vehs_num += self.shunt_num[route_index] + self.straight_num[route_index]

            travel_time += (
                        self.shunt_num[route_index] * (self.share_reroute_travel_time[route_index]/travel_time_scale) + self.straight_num[route_index] *
                        (self.share_straight_travel_time[route_index]/travel_time_scale))

            travel_time_existing += (self.travel_time_model_based(
                        g_ratio[0] if route_index < 2 else g_ratio[2], self.status_tracker[3][route_index], route_index)/travel_time_scale * self.status_tracker[0][route_index]*100 + self.travel_time_model_based(
                        g_ratio[1] if route_index < 2 else g_ratio[3], self.status_tracker[3][4+route_index], 4+route_index)/travel_time_scale * self.status_tracker[0][4+route_index]*100)

            if self.status_tracker[1][route_index]*100 >30:
                c = self.status_tracker[7][route_index]
            else:
                c = (s * (g_ratio[0] if route_index < 2 else g_ratio[2]))

            if self.status_tracker[3][route_index]> (c):
                self.over_sau_time[route_index] = min(self.over_sau_time[route_index] + Step_len, 300*timeslot_factor)
            else:
                self.over_sau_time[route_index] = max(self.over_sau_time[route_index]-Step_len, 0)

            if (self.status_tracker[1][route_index]*100 <15) and (self.status_tracker[0][route_index]*100 <15) and (self.over_sau_time[route_index]>60):
                self.over_sau_time[route_index] = 0




            if self.status_tracker[1][route_index+4]*100 >15:
                c = self.status_tracker[7][route_index+4]
            else:
                c = ((s/2) * (g_ratio[1] if route_index < 2 else g_ratio[3]))

            if self.status_tracker[3][4+route_index]>(c):
                self.over_sau_time[4+route_index] = min(self.over_sau_time[4+route_index] + Step_len, 300*timeslot_factor)
            else:
                self.over_sau_time[4+route_index] = max(self.over_sau_time[4+route_index]-Step_len, 0)
            if (self.status_tracker[1][route_index+4]*100 <7) and (self.status_tracker[0][route_index+4]*100 <7) and (self.over_sau_time[route_index+4]>60):
                self.over_sau_time[route_index+4] = 0

            vehs_num_existing += (self.status_tracker[0][route_index]+self.status_tracker[0][4+route_index])*100
        if vehs_num > 0:
            new_vehs_reward = 200/travel_time_scale - travel_time/vehs_num
        else:
            new_vehs_reward = 0

        if vehs_num_existing > 0:
            existing_vehs_reward = 50/travel_time_scale - travel_time_existing/vehs_num_existing
        else:
            existing_vehs_reward = 0
        reward = (reward_weight*existing_vehs_reward + new_vehs_reward)/2
        reward = max(min(reward, 1), -1)
        return reward

    def turn_right_ratio_based(self, ratio):
        # ratio: including the turn ratio for four edges(1*4)
        
        for veh_id in self.main_new_vehs_4decision:
            edge_id = traci.vehicle.getRoadID(veh_id)
            route_index = center_edges.index(edge_id)
            # center_edges = ['edge1-0', 'edge2-0', 'edge3-0', 'edge4-0']
            target_ratio = ratio[route_index]

            current_total = self.shunt_num[route_index]+self.straight_num[route_index]
            if self.shunt_num[route_index] == 0:
                current_ratio = 0
            else:
                current_ratio = self.shunt_num[route_index] / (current_total)
            rnd = np.random.rand(1)
            self.all_vehs_info[veh_id][6]= route_index
            if rnd < CAV_rate:
                self.smart_num[route_index] += 1
                if current_ratio < target_ratio:
                    self.shunt_num[route_index] += 1
                    self.all_vehs_info[veh_id][3] = 2
                    traci.vehicle.setRouteID(veh_id, routeID_list[route_index])
                    traci.vehicle.setColor(veh_id, (255, 0, 0))
                    self.all_vehs_info[veh_id][5] = self.share_reroute_travel_time[route_index]
                else:
                    self.straight_num[route_index] += 1
                    self.all_vehs_info[veh_id][5] = self.share_straight_travel_time[route_index]
            else:
                self.straight_num[route_index] += 1
                self.all_vehs_info[veh_id][5] = self.share_straight_travel_time[route_index]
                



    def update_vehs_set(self):
        self.main_vehs = set()
        self.main_new_vehs_4decision = set()
        self.current_all_vehs = set(traci.vehicle.getIDList())
        self.new_vehs = self.current_all_vehs.symmetric_difference(
            self.last_step_all_vehs.intersection(self.current_all_vehs))  # new vehicles
        for veh_id in (self.current_all_vehs - self.new_vehs):  # update accu. wait and travel time of existing vehicles
            self.all_vehs_info[veh_id][0] = traci.vehicle.getAccumulatedWaitingTime(veh_id)
            self.all_vehs_info[veh_id][2] = traci.simulation.getCurrentTime() - self.all_vehs_info[veh_id][1]
            self.all_vehs_info[veh_id][4] += traci.vehicle.getFuelConsumption(veh_id)
        for veh_id in self.current_all_vehs:
            edge_id = traci.vehicle.getRoadID(veh_id)
            if edge_id in center_edges:
                self.main_vehs = self.main_vehs.union(set([veh_id]))  # vehicles in main edge
        # new vehicles in main edge
        self.main_new_vehs = self.main_vehs.symmetric_difference(self.last_step_main_vehs.intersection(self.main_vehs))

        # record the set for finding the new vehicle in next duration
        self.last_step_all_vehs = self.current_all_vehs
        self.last_step_main_vehs = self.main_vehs

        # define the information about new vehicles
        #Frame form[AccumulatedWaitingTime, EnteringTime, TravelTime, Flag(0:Not in Main Road, 1:Straight in Main Road, 2:Rerouted in Main Road 3: turn left), FuelConsumption, EstimatedTravelTime, EnterDirection(1:west,2,east,3:south,4:north)]
        for veh_id in (self.new_vehs - self.main_new_vehs):
            self.all_vehs_info[veh_id] = [traci.vehicle.getAccumulatedWaitingTime(veh_id),
                                          traci.simulation.getCurrentTime(), traci.simulation.getCurrentTime(), 0, 0,-1,-1]
        for veh_id in self.main_new_vehs:
            type_id = traci.vehicle.getTypeID(veh_id)
            if type_id == "Car":
                self.main_new_vehs_4decision.add(veh_id)
                self.all_vehs_info[veh_id] = [traci.vehicle.getAccumulatedWaitingTime(veh_id),
                                            traci.simulation.getCurrentTime(), traci.simulation.getCurrentTime(), 1, 0,-1,-1]
            elif type_id == "Car2": #left turn
                self.all_vehs_info[veh_id] = [traci.vehicle.getAccumulatedWaitingTime(veh_id),
                                            traci.simulation.getCurrentTime(), traci.simulation.getCurrentTime(), 3, 0,-1,-1]
            else:
                print("Car type error")

    def induction_loop_count(self):
        for loop_id in self.induction_loop_ID_list:
            self.induction_loop_num[loop_id].put(traci.inductionloop.getLastStepVehicleNumber(loop_id))
            self.induction_loop_num[loop_id].get()  # 返回并删除队列头部元素

    def sim_step(self, action_change_ratio):
        traci.simulationStep()
        self.current_phase_duration += 1
        self.update_vehs_set()
        self.turn_right_ratio_based(action_change_ratio)  # part of vehicles turns right
        self.induction_loop_count()
        for lane in self.MeanSpeed_update_lanes:
            
            Lane_veh_Num = traci.lane.getLastStepVehicleNumber(lane)
            self.lanes_veh_Num_time_dict[lane]=Lane_veh_Num
            if Lane_veh_Num<1:
                MeanSpeed = normal_speed
            else:
                MeanSpeed = min(traci.lane.getLastStepMeanSpeed(lane),normal_speed)
            if len(self.lanes_MeanSpeed_dict[lane])>=30:
                del self.lanes_MeanSpeed_dict[lane][0]
            self.lanes_MeanSpeed_dict[lane].append(MeanSpeed)


    def take_action(self, action, dynamic_flag):
        self.advise1 = 0
        self.advise3 = 0
        for lane in self.MeanSpeed_update_lanes:
            self.lanes_MeanSpeed_dict[lane] = []
        
        if len(action) == 8:
            self.advise2 = list(np.zeros(4))
            action_change_phase, action_change_ratio = 2*action[0:4], action[4:]
        step = 0
        last_dur_end_phase = traci.trafficlight.getPhase(node_light_7)
        self.current_phase_duration = 0
        
        action_change_phase_revise = action_change_phase*(action_change_phase>(6/Step_len))
        selected_phase_list = []
        action_selected_phase_revise = []
        for phase_idx in range(action_change_phase_revise.size):
            if action_change_phase_revise[phase_idx]>0:
                selected_phase_list.append(phase_idx*2)
                action_selected_phase_revise.append(action_change_phase_revise[phase_idx])
        self.pre_g_ratio=copy.deepcopy(self.real_g_ratio)
        self.real_g_ratio = np.round((action_change_phase_revise/np.sum(action_change_phase_revise))*Step_len)/Step_len
        g_ratio = self.real_g_ratio
        action_selected_phase_revise = np.array(action_selected_phase_revise)
        action_selected_phase_revise = np.round((action_selected_phase_revise/np.sum(action_selected_phase_revise))*Step_len)


        for route_index in range(4):
            if len(self.model_based_TT[str(route_index)])>3:
                del self.model_based_TT[str(route_index)][0]
            self.model_based_TT[str(route_index)].append(self.travel_time_model_based(
                        g_ratio[0] if route_index < 2 else g_ratio[2], self.status_tracker[3][route_index], route_index))

            if len(self.speed_based_TT[str(route_index)])>3:
                del self.speed_based_TT[str(route_index)][0]
            self.speed_based_TT[str(route_index)].append(self.status_tracker[6][route_index]*travel_time_scale)
                

            self.share_straight_travel_time[route_index] = self.status_tracker[5][route_index]*travel_time_scale + (np.mean(self.model_based_TT[str(route_index)])+np.mean(self.speed_based_TT[str(route_index)]) )/2
            self.share_reroute_travel_time[route_index] = self.status_tracker[4][route_index]*travel_time_scale



        for phase_idx in range(len(selected_phase_list)):
            if phase_idx ==0:
                if last_dur_end_phase == selected_phase_list[phase_idx]:
                    for _ in range(int(action_selected_phase_revise[phase_idx]-3)):
                        self.sim_step(action_change_ratio)
                        step += 1
                else:
                    traci.trafficlight.setPhase(node_light_7, last_dur_end_phase+1)  # 3s黄灯
                    for _ in range(3):
                        self.sim_step(action_change_ratio)
                        step += 1
                    self.current_phase_duration = selected_phase_list[phase_idx]
                    traci.trafficlight.setPhase(node_light_7, selected_phase_list[phase_idx]) 
                    for _ in range(int(action_selected_phase_revise[phase_idx]-6)):
                        self.sim_step(action_change_ratio)
                        step += 1
            else:
                self.current_phase_duration = selected_phase_list[phase_idx]
                traci.trafficlight.setPhase(node_light_7, selected_phase_list[phase_idx])  
                for _ in range(int(action_selected_phase_revise[phase_idx]-3)):
                    self.sim_step(action_change_ratio)
                    step += 1

            if phase_idx ==(len(selected_phase_list)-1):
                for _ in range(Step_len-step):
                    self.sim_step(action_change_ratio)
                    step += 1
            else:
                traci.trafficlight.setPhase(node_light_7, selected_phase_list[phase_idx]+1)  
                for _ in range(3):
                    self.sim_step(action_change_ratio)
                    step += 1
        if step != Step_len:
            print(f"step is {step} which is not equal to StepLength {Step_len}")


        reward = self.return_reward(g_ratio)
        self.straight_num = np.zeros(4)
        self.shunt_num = np.zeros(4)
        self.smart_num = np.zeros(4)
        self.update_state()
        if len(action) <= 2:
            if np.mean(self.over_sau_time) > 280*timeslot_factor:
                self.advise2 = 1*(random.random()>0.5)
        else:
            for index in range(4):
                if self.over_sau_time[index] > 280*timeslot_factor:
                    self.advise2[index] = 1*(random.random()>0.5)
        return reward


def close_sumo():
    traci.close()


def get_max_queue_length(listLanes):
    max_queue_length = 0
    for lane in listLanes:
        queue_length = traci.lane.getLastStepHaltingNumber(lane)
        if max_queue_length < queue_length:
            max_queue_length = queue_length
    return max_queue_length
