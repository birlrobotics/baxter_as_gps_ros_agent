#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from operator import itemgetter
from baxter_as_gps_ros_agent import JointBasedEndPointRelatedValuesCalculator
import numpy as np
from gps.proto.gps_pb2 import (
    END_EFFECTOR_POINTS,
    END_EFFECTOR_POINT_VELOCITIES,
    END_EFFECTOR_POINT_JACOBIANS,
    END_EFFECTOR_POINT_ROT_JACOBIANS,
    END_EFFECTOR_POSITIONS,
    END_EFFECTOR_ROTATIONS,
    END_EFFECTOR_JACOBIANS,
)
import pdb


end_point_offset = np.array([
    [0.1, 0.2, 0.3],
    [-0.1, -0.2, -0.3],
])


all_joints = ["head_pan", "l_gripper_l_finger_joint", "l_gripper_r_finger_joint", "left_e0", "left_e1", "left_s0",
"left_s1", "left_w0", "left_w1", "left_w2", "r_gripper_l_finger_joint", "r_gripper_r_finger_joint",
"right_e0", "right_e1", "right_s0", "right_s1", "right_w0", "right_w1", "right_w2"]
target_joints = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

angle_getter = itemgetter(*[all_joints.index(i) for i in target_joints])

def topic_cb(msg):
    global jbeprvc
    joint_angles = angle_getter(msg.position)
    joint_velocities = angle_getter(msg.velocity)
    
    jac = jbeprvc.get_calculation_result(joint_angles, joint_velocities)

if __name__ == '__main__':
    rospy.init_node('test_baxter_end_point_related_calculation_from_joint_values_node', log_level=rospy.INFO)

    jbeprvc = JointBasedEndPointRelatedValuesCalculator(
        values_to_calculate = [
            END_EFFECTOR_POINTS,
            END_EFFECTOR_POINT_VELOCITIES,
            END_EFFECTOR_POINT_JACOBIANS,
            END_EFFECTOR_POINT_ROT_JACOBIANS,
            END_EFFECTOR_POSITIONS,
            END_EFFECTOR_ROTATIONS,
            END_EFFECTOR_JACOBIANS,
        ],
        end_point_offset=end_point_offset.transpose(),
    )

    joint_state_sub = rospy.Subscriber('/robot/joint_states', JointState, topic_cb)

    rospy.spin()
