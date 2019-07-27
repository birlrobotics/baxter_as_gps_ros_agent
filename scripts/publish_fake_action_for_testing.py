#!/usr/bin/env python

import rospy
from baxter_as_gps_ros_agent.msg import BaxterRightArmAction
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

def topic_cb(joint_msg):
    global pub
    pub.publish(BaxterRightArmAction(header=Header(stamp=joint_msg.header.stamp), action=[0,0,0,0,0,0,0])) 
if __name__ == '__main__':
    rospy.init_node("publish_fake_action_node")
    
    joint_state_sub = rospy.Subscriber('/robot/joint_states', JointState, topic_cb)
    pub = rospy.Publisher('/baxter_right_arm_torque_action', BaxterRightArmAction, queue_size=100)
    rospy.spin()

