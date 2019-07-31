from base_action_calculator import BaseActionCalculator
import numpy as np

class CaffeBasedNeuralNetworkActionCalculator(BaseActionCalculator):
    def __init__(self, obs_scale, obs_bias, net_param_string, action_noise):
        super(CaffeBasedNeuralNetworkActionCalculator, self).__init__()

    def get_action(self, episode_t, input_column_vector):
        pass
