import pdb

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

from gps.proto.gps_pb2 import (
    ACTION,
    JOINT_ANGLES,
    JOINT_VELOCITIES,
    END_EFFECTOR_POINTS,
    END_EFFECTOR_POINT_VELOCITIES,
    END_EFFECTOR_POINT_JACOBIANS,
    END_EFFECTOR_POINT_ROT_JACOBIANS,
    END_EFFECTOR_POSITIONS,
    END_EFFECTOR_ROTATIONS,
    END_EFFECTOR_JACOBIANS,
)
from gps_agent_pkg.msg import SampleResult, DataType
from rostopics_to_timeseries import RosTopicFilteringScheme, TopicMsgFilter, OfflineRostopicsToTimeseries
from sensor_msgs.msg import JointState
from baxter_as_gps_ros_agent import JointBasedEndPointRelatedValuesCalculator
from baxter_as_gps_ros_agent.msg import BaxterRightArmAction
import numpy as np
from operator import itemgetter
from gps_agent_pkg.msg import SampleResult
from baxter_as_gps_ros_agent import CONSTANT

JOINT_BASED_DATATYPES = set([
    JOINT_ANGLES,
    JOINT_VELOCITIES,
    END_EFFECTOR_POINTS,
    END_EFFECTOR_POINT_VELOCITIES,
    END_EFFECTOR_POINT_JACOBIANS,
    END_EFFECTOR_POINT_ROT_JACOBIANS,
    END_EFFECTOR_POSITIONS,
    END_EFFECTOR_ROTATIONS,
    END_EFFECTOR_JACOBIANS,
])


def get_topic_names_that_will_be_recorded_into_rosbag(datatypes):
    topics = set()
    for i in datatypes:
        if i in JOINT_BASED_DATATYPES:        
            topics.add('/robot/joint_states')
        elif i == ACTION:
            topics.add(CONSTANT.action_topic)
        else:
            raise Exception('datatype not supported yet')
    return list(topics)


def get_RosTopicFilteringScheme_and_time_series_remapping_array(datatypes, frequency, offset_points, target_points):
    tfc = RosTopicFilteringScheme(resampling_rate=frequency)
    joint_based_datatypes = [i for i in datatypes if i in JOINT_BASED_DATATYPES]
    joint_based_datatypes_size, joint_based_datatypes_shape = _setup_joint_based_datatype_filter(tfc, joint_based_datatypes, offset_points, target_points)
    other_datatypes = [i for i in datatypes if i not in JOINT_BASED_DATATYPES]
    other_datatypes_size, other_datatypes_shape = _setup_other_datatype_filter(tfc, other_datatypes)

    datatypes_of_new_order = joint_based_datatypes+other_datatypes
    datatypes_size_of_new_order = joint_based_datatypes_size+other_datatypes_size
    datatypes_end_idx_of_new_order = np.cumsum(datatypes_size_of_new_order)

    remapping_array = []
    
    for dtype in datatypes:
        dtype_idx_in_new_order = datatypes_of_new_order.index(dtype)

        end_idx = datatypes_end_idx_of_new_order[dtype_idx_in_new_order]
        start_idx = end_idx-datatypes_size_of_new_order[dtype_idx_in_new_order]
        remapping_array += range(start_idx, end_idx)
        
    return tfc, remapping_array



def process_rosbag_to_SampleResult(rosbag_path, datatypes, frequency, offset_points, target_points):
    assert(offset_points.shape[0] == 3)
    assert(target_points.shape[0] == 3)
    assert(offset_points.shape[1] == target_points.shape[1])
    num_of_points = offset_points.shape[1]


    tfc = RosTopicFilteringScheme(resampling_rate=frequency)
    
    joint_based_datatypes = [i for i in datatypes if i in JOINT_BASED_DATATYPES]
    joint_based_datatypes_size, joint_based_datatypes_shape = _setup_joint_based_datatype_filter(tfc, joint_based_datatypes, offset_points, target_points)

    other_datatypes = [i for i in datatypes if i not in JOINT_BASED_DATATYPES]
    other_datatypes_size, other_datatypes_shape = _setup_other_datatype_filter(tfc, other_datatypes)

    # re-order datatypes so that they match with the order of filtering
    datatypes = joint_based_datatypes+other_datatypes
    datatypes_size = joint_based_datatypes_size+other_datatypes_size
    datatypes_shape = joint_based_datatypes_shape+other_datatypes_shape

    ofrt = OfflineRostopicsToTimeseries(tfc) 
    t, mat = ofrt.get_timeseries_mat(rosbag_path)

    T = mat.shape[0]
    datatypes_falttened = np.split(mat, indices_or_sections=np.cumsum(datatypes_size)[:-1], axis=1)

    assert len(datatypes_falttened) == len(datatypes)

    sr = SampleResult()
    for i in range(len(datatypes)):
        data_type = datatypes[i]
        shape = (T,)+datatypes_shape[i]
        data = datatypes_falttened[i].flatten()
        sr.sensor_data.append(
            DataType(
                data_type=data_type,
                shape=shape,
                data=data
            )
        )     

    return sr

def _setup_other_datatype_filter(tfc, other_datatypes):
    # TODO: set up filters for other datatypes

    other_datatypes_size = []
    other_datatypes_shape = []

    for i in other_datatypes:
        if i == ACTION:
            other_datatypes_size.append(7)
            other_datatypes_shape.append((7,))
            class ActionFilter(TopicMsgFilter):
                target_joints = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
                def __init__(self):
                    super(ActionFilter, self).__init__()

                @staticmethod
                def vector_size():
                    return 7

                @staticmethod
                def vector_meaning():
                    return [i+'.action' for i in ActionFilter.target_joints]

                def convert(self, msg):
                    return msg.action
            tfc.add_filter(
                "/baxter_right_arm_torque_action", 
                BaxterRightArmAction,
                ActionFilter,
            )
        else:
            raise Exception('datatype %s not supported yet'%i)

    return other_datatypes_size, other_datatypes_shape

def _setup_joint_based_datatype_filter(tfc, joint_based_datatypes, offset_points, target_points):
    jbeprvc = JointBasedEndPointRelatedValuesCalculator(
        values_to_calculate=joint_based_datatypes,
        end_point_offset=offset_points,
        target_end_point=target_points,
    )
    test_return = jbeprvc.get_calculation_result(np.array([0.3]*7), np.array([0.3]*7))

    joint_based_datatypes_shape = [
        test_return[i].shape for i in joint_based_datatypes
    ]
    joint_based_datatypes_size = [
        test_return[i].size for i in joint_based_datatypes
    ]
    class RightArmJointAngleFilter(TopicMsgFilter):
        all_joints = ["head_pan", "l_gripper_l_finger_joint", "l_gripper_r_finger_joint", "left_e0", "left_e1", "left_s0",
    "left_s1", "left_w0", "left_w1", "left_w2", "r_gripper_l_finger_joint", "r_gripper_r_finger_joint",
    "right_e0", "right_e1", "right_s0", "right_s1", "right_w0", "right_w1", "right_w2"]
        target_joints = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
        def __init__(self):
            super(RightArmJointAngleFilter, self).__init__()
            self.getter = itemgetter(*[RightArmJointAngleFilter.all_joints.index(i) for i in RightArmJointAngleFilter.target_joints])

        @staticmethod
        def vector_size():
            return sum(joint_based_datatypes_size)

        @staticmethod
        def vector_meaning():
            return ["NA"]*sum(joint_based_datatypes_size)

        def convert(self, msg):
            d = jbeprvc.get_calculation_result(
                self.getter(msg.position),
                self.getter(msg.velocity),
            )
            return np.concatenate([
                np.asarray(d[i]).flatten() for i in joint_based_datatypes
            ])

    tfc.add_filter(
        "/robot/joint_states", 
        JointState,
        RightArmJointAngleFilter,
    )

    return joint_based_datatypes_size, joint_based_datatypes_shape
