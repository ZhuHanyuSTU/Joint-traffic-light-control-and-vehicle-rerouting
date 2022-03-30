import os
import json
import numpy as np



def state2input(state):
    concat_input = np.concatenate((state.vehs_num, state.queue_len, state.est_arrival_rate, state.over_sau_time,
                                   state.ave_shunt_traval_time, state.ave_straight_traval_time,
                                   state.ave_itelli_traval_time, state.current_phase), axis=1)

    state_dim = np.size(concat_input)
    net_input = np.reshape(concat_input, (state_dim,))
    return net_input


class State(object):

    def __init__(self,
                 vehs_num,
                 queue_len,
                 est_arrival_rate,
                 over_sau_time,
                 ave_shunt_traval_time,
                 ave_straight_traval_time,
                 ave_itelli_traval_time,
                 current_phase):

        self.vehs_num = vehs_num
        self.queue_len = queue_len
        self.est_arrival_rate = est_arrival_rate
        self.over_sau_time = over_sau_time
        self.ave_shunt_traval_time = ave_shunt_traval_time
        self.ave_straight_traval_time = ave_straight_traval_time
        self.ave_itelli_traval_time = ave_itelli_traval_time
        self.current_phase = current_phase

class PathSet:

    def __init__(self, path_to_data, path_to_output, path_to_model, path_to_eval):

        self.PATH_TO_DATA = path_to_data
        self.PATH_TO_OUTPUT = path_to_output
        self.PATH_TO_MODEL = path_to_model
        self.PATH_TO_EVAL = path_to_eval

        if not os.path.exists(self.PATH_TO_OUTPUT):
            os.makedirs(self.PATH_TO_OUTPUT)
        if not os.path.exists(self.PATH_TO_MODEL):
            os.makedirs(self.PATH_TO_MODEL)
        if not os.path.exists(self.PATH_TO_EVAL):
            os.makedirs(self.PATH_TO_EVAL)

