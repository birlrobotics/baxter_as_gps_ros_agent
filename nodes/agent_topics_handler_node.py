#!/usr/bin/env python
import rospy
from baxter_as_gps_ros_agent import AgentTopicsHandler

if __name__ == '__main__':
    rospy.init_node('agent_topics_handler_node', log_level=rospy.INFO)
    agent_topics_handler = AgentTopicsHandler()
    agent_topics_handler.setup_baxter()
    agent_topics_handler.setup_subscriber_and_publisher_and_service()
    rospy.loginfo("agent_topics_handler_node starts")
    rospy.spin()
