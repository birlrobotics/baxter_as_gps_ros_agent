#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from operator import itemgetter
from baxter_as_gps_ros_agent import BaxterRightArmEndPointJacobianCalculator
from baxter_as_gps_ros_agent.srv import SetUpEndPointOffsetAndPublishEndPointJacobian, SetUpEndPointOffsetAndPublishEndPointJacobianRequest, SetUpEndPointOffsetAndPublishEndPointJacobianResponse
import numpy as np
import pdb


all_joints = ["head_pan", "l_gripper_l_finger_joint", "l_gripper_r_finger_joint", "left_e0", "left_e1", "left_s0",
"left_s1", "left_w0", "left_w1", "left_w2", "r_gripper_l_finger_joint", "r_gripper_r_finger_joint",
"right_e0", "right_e1", "right_s0", "right_s1", "right_w0", "right_w1", "right_w2"]
target_joints = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']

angle_getter = itemgetter(*[all_joints.index(i) for i in target_joints])


joint_state_sub = None

def topic_cb(msg):
    global braepjc, joint_state_sub
    angles = angle_getter(msg.position)
    jac = braepjc.get_end_point_jacobian(dict(zip(target_joints, angles)))
    rospy.logdebug(jac)

def service_cb(req):
    global joint_state_sub

    if len(req.end_point_offset.data) == 0:
        rospy.loginfo("turn off jacobian publishing")
        joint_state_sub.unregister()
        joint_state_sub = None
    else:
        rospy.loginfo("turn on jacobian publishing")
        dim = [i.size for i in req.end_point_offset.layout.dim]
        end_point_offset = np.array(req.end_point_offset.data).reshape(dim)
        rospy.logdebug(end_point_offset)
        joint_state_sub = rospy.Subscriber('/robot/joint_states', JointState, topic_cb)

    return SetUpEndPointOffsetAndPublishEndPointJacobianResponse()

if __name__ == '__main__':
    rospy.init_node('baxter_right_arm_end_point_jacobian_publisher_node', log_level=rospy.DEBUG)

    server = rospy.Service('SetUpEndPointOffsetAndPublishEndPointJacobian', SetUpEndPointOffsetAndPublishEndPointJacobian, service_cb)

    braepjc = BaxterRightArmEndPointJacobianCalculator()

    rospy.spin()
