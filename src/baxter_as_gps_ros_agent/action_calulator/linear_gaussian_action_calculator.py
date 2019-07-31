from base_action_calculator import BaseActionCalculator
import numpy as np

class LinearGaussianActionCalculator(BaseActionCalculator):
    def __init__(self, K, k, nominal_x=None, nominal_u=None):
        super(LinearGaussianActionCalculator, self).__init__()

        episode_length, dim_of_u, dim_of_x = K.shape

        assert k.shape[0] == episode_length
        assert k.shape[1] == dim_of_u
        assert k.shape[2] == 1

        if nominal_x is None:
            nominal_x = np.zeros((episode_length, dim_of_x, 1))

        if nominal_u is None:
            nominal_u = np.zeros((episode_length, dim_of_u, 1))
         
        self.K = K
        self.k = k
        self.episode_length = episode_length
        self.dim_of_u = dim_of_u
        self.dim_of_x = dim_of_x
        self.nominal_x = nominal_x
        self.nominal_u = nominal_u

    def get_action(self, episode_t, input_column_vector):
        if episode_t >= self.episode_length:
            raise Exception('episode_t %s >= episode_length %s'%(episode_t, self.episode_length))
        elif episode_t < 0:
            raise Exception('episode_t %s < 0'%(episode_t,))

        # u-nominal_u = K (x-nominal_x) + k

        t = episode_t
        x = input_column_vector
        nominal_u = self.nominal_u[t] 
        nominal_x = self.nominal_x[t]
        K = self.K[t]
        k = self.k[t]

        u = np.matmul(K, x-nominal_x) + k + nominal_u
        return u.flatten()

if __name__ == '__main__':
    K = np.random.rand(100, 7,7)
    k = np.random.rand(100, 7,1)
    lgac = LinearGaussianActionCalculator(K, k)

    for i in [1, 10]:
        print lgac.get_action(i, np.random.rand(7,1))

    for i in [101, -1]:
        try:
            lgac.get_action(i, np.random.rand(7,1))
            print 'exception should be thrown'
        except Exception as e:
            print 'exception received: %s'%e
