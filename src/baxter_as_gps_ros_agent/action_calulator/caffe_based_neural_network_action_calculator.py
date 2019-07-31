from base_action_calculator import BaseActionCalculator
import numpy as np
import pdb
import tempfile

class CaffeBasedNeuralNetworkActionCalculator(BaseActionCalculator):
    def __init__(self, obs_scale, obs_bias, net_param_string, action_noise):
        super(CaffeBasedNeuralNetworkActionCalculator, self).__init__()
        self.obs_scale = obs_scale
        self.obs_bias = obs_bias
        self.net_param_string = net_param_string
        self.action_noise = action_noise

        self._setup_caffe_net()

    def get_action(self, episode_t, input_column_vector):
        obs = input_column_vector.flatten()
        obs = obs.dot(self.obs_scale) + self.obs_bias
        self.net.blobs[self.net.blobs.keys()[0]].data[:] = obs
        action_mean = self.net.forward().values()[0][0]
        u = action_mean + self.action_noise[episode_t]
        return u

    def _setup_caffe_net(self):
        import caffe
        import caffe.proto.caffe_pb2 as caffe_pb2
        net_parameter = caffe_pb2.NetParameter()
        net_parameter.ParseFromString(self.net_param_string)
        tmp_f = tempfile.NamedTemporaryFile(delete=False, mode='w+')
        tmp_f.write(str(net_parameter))
        tmp_f.close()
        tmp_f_path = tmp_f.name
        self.net = caffe.Net(tmp_f_path, caffe.TEST)
