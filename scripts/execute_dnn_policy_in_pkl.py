#!/usr/bin/env python

import argparse
import rospy
import imp

import os
import gps_agent_pkg
import sys
sys.path.append(os.path.join(
    os.sep.join(gps_agent_pkg.__path__[0].split(os.sep)[:-7]),
    'python',
))


from gps.agent.ros.ros_utils import policy_to_msg

from gps_agent_pkg.msg import TrialCommand, SampleResult

import pickle
import pdb
import numpy as np

if __name__ == '__main__':

    parser = argparse.ArgumentParser() 
    parser.add_argument("--hyperparams-py", dest='hyperparams_py', type=str, help='path to hyperparams.py', required=True)
    parser.add_argument("--algorithm-pkl", dest='algorithm_pkl', type=str, help='path to algorithm pickle', required=True)

    args = parser.parse_args()


    with open(args.algorithm_pkl, 'rb') as f:
        algorithm = pickle.load(f)

    hyperparams = imp.load_source('hyperparams', args.hyperparams_py)
    config = hyperparams.config

    agent = config['agent']['type'](config['agent'])

    policy =  algorithm.policy_opt.policy

    T = algorithm.T
    dU = algorithm.dU

    noise = np.zeros((T, dU))

    pub = rospy.Publisher('gps_controller_trial_command', TrialCommand, queue_size=None)

    for condition in range(config['common']['conditions']):
        agent.sample(policy, condition, save=False, noisy=False)
