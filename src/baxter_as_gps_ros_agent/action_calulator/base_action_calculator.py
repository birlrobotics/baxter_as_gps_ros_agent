

class BaseActionCalculator(object):
    def __init__(self):
        pass

    def get_action(self, episode_t, input_vector):
        raise Exception('not implemented')

