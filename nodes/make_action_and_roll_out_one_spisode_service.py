#!/usr/bin/env python

import rospy
from baxter_as_gps_ros_agent.srv import SetupControllerAndRolloutOneEpisode, SetupControllerAndRolloutOneEpisodeRequest, SetupControllerAndRolloutOneEpisodeResponse
import pdb
import numpy as np
from rostopics_to_timeseries.msg import Timeseries
from baxter_as_gps_ros_agent.msg import BaxterRightArmAction
from threading import Event
from gps.proto.gps_pb2 import LIN_GAUSS_CONTROLLER, CAFFE_CONTROLLER
from baxter_as_gps_ros_agent import LinearGaussianActionCalculator, CaffeBasedNeuralNetworkActionCalculator
import baxter_interface
from baxter_interface import CHECK_VERSION

event_start_new_episode = Event()
event_episode_is_done = Event()
start_time_in_sec = None
end_time_in_sec = None
duration_in_sec = None
publish_action_to_this_topic = None
sensor_time_series_topic = None

sensor_time_series_sub = None
action_pub = None
action_calculator = None
control_frequency = None
time_series_remapping = None

def cb(req):
    global event_start_new_episode, event_episode_is_done, start_time_in_sec, end_time_in_sec, duration_in_sec, publish_action_to_this_topic, sensor_time_series_topic, action_calculator, control_frequency, time_series_remapping

    control_frequency = req.frequency
    time_series_remapping = req.time_series_remapping

    controller_type = req.controller.controller_to_execute
    T = req.T
    if controller_type == LIN_GAUSS_CONTROLLER:
        dim_of_u = req.controller.lingauss.dU
        dim_of_x = req.controller.lingauss.dX
        K = np.array(req.controller.lingauss.K_t).reshape((T, dim_of_u, dim_of_x))
        k = np.array(req.controller.lingauss.k_t).reshape((T, dim_of_u, 1))
        action_calculator = LinearGaussianActionCalculator(K, k)
    elif controller_type == CAFFE_CONTROLLER:
        dim_of_u = req.controller.caffe.dU
        dim_of_bias = req.controller.caffe.dim_bias
        action_calculator = CaffeBasedNeuralNetworkActionCalculator(
            obs_scale = np.array(req.controller.caffe.scale).reshape((dim_of_bias, dim_of_bias)),
            obs_bias = np.array(req.controller.caffe.bias),
            net_param_string = req.controller.caffe.net_param,
            action_noise = np.array(req.controller.caffe.noise).reshape((T, dim_of_u)),
        )
    else:
        raise Exception('controller type %s not supported yet'%controller_type)


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
    
    return SetupControllerAndRolloutOneEpisodeResponse(
        episode_start_time_in_sec=start_time_in_sec,
        episode_end_time_in_sec=end_time_in_sec,     
    )

def topic_cb(msg):
    global event_episode_is_done, start_time_in_sec, end_time_in_sec, duration_in_sec, sensor_time_series_sub, action_pub, action_calculator, control_frequency, time_series_remapping




    now_time_in_sec = msg.header.stamp.to_sec()
    if start_time_in_sec is None:
        start_time_in_sec = now_time_in_sec
        time_lapsed = 0
    else:
        time_lapsed = now_time_in_sec-start_time_in_sec
        if time_lapsed >= duration_in_sec:
            sensor_time_series_sub.unregister()

            action = [0,0,0,0,0,0,0]
            baxter_right_arm.set_joint_torques(dict(zip(baxter_right_arm.joint_names(), action)))

            baxter_right_arm.set_joint_positions(baxter_right_arm.joint_angles())

            end_time_in_sec = now_time_in_sec 
            event_episode_is_done.set()
            return

    input_length = len(msg.sample)
    input_vector = np.zeros((input_length, 1))
    for i in range(input_length):
        input_vector[i][0] = msg.sample[time_series_remapping[i]]

    episode_step = int(time_lapsed*control_frequency)
    action = action_calculator.get_action(episode_step, input_vector)
    baxter_right_arm.set_joint_torques(dict(zip(baxter_right_arm.joint_names(), action.flatten())))
    action_pub.publish(BaxterRightArmAction(
            header=msg.header,
            action=action.flatten(),
    ))

if __name__ == '__main__':
    rospy.init_node('make_action_and_rollout_one_episode_service_node', log_level=rospy.INFO)

    baxter_right_arm = baxter_interface.Limb('right')

    server = rospy.Service('SetupControllerAndRolloutOneEpisode_service', SetupControllerAndRolloutOneEpisode, cb)

    rate = rospy.Rate(1)
    event_start_new_episode.clear()
    while not rospy.is_shutdown():
        if event_start_new_episode.is_set():
            event_start_new_episode.clear()
            action_pub = rospy.Publisher(publish_action_to_this_topic, BaxterRightArmAction, queue_size=None)         
            sensor_time_series_sub = rospy.Subscriber(sensor_time_series_topic, Timeseries, topic_cb)
        rate.sleep() 
