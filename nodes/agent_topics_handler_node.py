#!/usr/bin/env python
import rospy
from baxter_as_gps_ros_agent import AgentTopicsHandler

if __name__ == '__main__':
    rospy.init_node('agent_topics_handler_node')
    agent_topics_handler = AgentTopicsHandler()
    rospy.loginfo("agent_topics_handler_node starts")
    rospy.spin()
