#!/usr/bin/env python

import rospy
from baxter_as_gps_ros_agent.srv import SetupControllerAndRolloutOneEpisode, SetupControllerAndRolloutOneEpisodeRequest, SetupControllerAndRolloutOneEpisodeResponse
import pdb
import numpy as np
from rostopics_to_timeseries.msg import Timeseries
from baxter_as_gps_ros_agent.msg import BaxterRightArmAction
from threading import Event


event_start_new_episode = Event()
event_episode_is_done = Event()
start_time_in_sec = None
end_time_in_sec = None
duration_in_sec = None
publish_action_to_this_topic = None
sensor_time_series_topic = None

sensor_time_series_sub = None
action_pub = None

def cb(req):
    global event_start_new_episode, event_episode_is_done, start_time_in_sec, end_time_in_sec, duration_in_sec, publish_action_to_this_topic, sensor_time_series_topic 


    event_episode_is_done.clear()
    start_time_in_sec = None
    end_time_in_sec = None
    duration_in_sec = float(req.T)/req.frequency
    sensor_time_series_topic = req.sensor_time_series_topic
    publish_action_to_this_topic = req.publish_action_to_this_topic

    rospy.loginfo('gonna issue action for %s secs'%(duration_in_sec,))

    event_start_new_episode.set()
    rospy.loginfo('start a new episode, gonna wait for it to finish')
    event_episode_is_done.clear()
    event_episode_is_done.wait()

    event_start_new_episode.clear()
    event_episode_is_done.clear()
    rospy.loginfo('episode finish')
    
    return SetupControllerAndRolloutOneEpisodeResponse()

def topic_cb(msg):
    global event_episode_is_done, start_time_in_sec, end_time_in_sec, duration_in_sec, sensor_time_series_sub, action_pub


    now_time_in_sec = msg.header.stamp.to_sec()
    if start_time_in_sec is None:
        start_time_in_sec = now_time_in_sec
    elif now_time_in_sec-start_time_in_sec > duration_in_sec:
        sensor_time_series_sub.unregister()
        end_time_in_sec = now_time_in_sec 
        event_episode_is_done.set()
        return

    action = [0,0,0,0,0,0,0]
    action_pub.publish(BaxterRightArmAction(
            header=msg.header,
            action=action,
    ))

if __name__ == '__main__':
    rospy.init_node('make_action_and_rollout_one_episode_service_node', log_level=rospy.DEBUG)
    server = rospy.Service('SetupControllerAndRolloutOneEpisode_service', SetupControllerAndRolloutOneEpisode, cb)

    rate = rospy.Rate(1)
    event_start_new_episode.clear()
    while not rospy.is_shutdown():
        if event_start_new_episode.is_set():
            event_start_new_episode.clear()
            action_pub = rospy.Publisher(publish_action_to_this_topic, BaxterRightArmAction, queue_size=None)         
            sensor_time_series_sub = rospy.Subscriber(sensor_time_series_topic, Timeseries, topic_cb)
        rate.sleep() 
