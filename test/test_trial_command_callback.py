
import rospy

# The original GPS package doesn't use catkin.
# So the folloing lines are necessary if we want
#   to import packages from gps_agent_pkg
import roslib
roslib.load_manifest('gps_agent_pkg')

import os
import gps_agent_pkg
import sys
sys.path.append(os.path.join(
    os.sep.join(gps_agent_pkg.__path__[0].split(os.sep)[:-4]),
    'python',
))

from gps_agent_pkg.msg import (
    TrialCommand, 
)
from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, \
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE
import numpy as np
import pickle
import pdb
import os

if __name__ == '__main__':
    rospy.init_node('test_trial_command_callback')
    pub = rospy.Publisher('gps_controller_trial_command', TrialCommand, queue_size=None)


    end_point = np.array([
        [0.1, 0.2, 0.3],
        [-0.1, -0.2, -0.3],
    ])

    end_point_tgt = 3+np.array([
        [0.1, 0.2, 0.3],
        [-0.1, -0.2, -0.3],
    ])
    with open(
        os.path.join(
            os.path.dirname(os.path.realpath(__file__)), 'trial_commnd_msg.pkl'
        ),
        'rb') as f:
        saved_trial_command_msg = pickle.load(f)

    '''
    msg = TrialCommand(
        T=100,
        frequency=20,
        state_datatypes=[JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES],
        obs_datatypes=[JOINT_ANGLES, JOINT_VELOCITIES, END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES],
        ee_points=end_point.flatten(),
        ee_points_tgt=end_point_tgt.flatten(),
    )
    '''
    msg = saved_trial_command_msg

    rospy.sleep(1)
    pub.publish(msg)
