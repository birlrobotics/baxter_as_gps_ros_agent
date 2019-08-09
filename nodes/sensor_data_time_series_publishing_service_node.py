#!/usr/bin/env python

import rospy
from baxter_as_gps_ros_agent.srv import CollectSensorDataThenPublishItAsTimeSeries, CollectSensorDataThenPublishItAsTimeSeriesRequest, CollectSensorDataThenPublishItAsTimeSeriesResponse
import pdb
import numpy as np
from rostopics_to_timeseries import OnlineRostopicsToTimeseries
import baxter_as_gps_ros_agent.util as util

onrt = None

def cb(req):
    global onrt
    if len(req.datatypes) != 0:
        if onrt is not None:
            onrt.stop_publishing_timeseries()
        # start publishing 
        rospy.logdebug("req: %s"%req)
        rospy.loginfo('start sensor data publishing')
        tfc, time_series_remapping = config.get_RosTopicFilteringScheme_and_time_series_remapping_array(
                req.datatypes,
                req.frequency,
                np.array(req.ee_points).reshape((3,-1)),
                np.array(req.ee_points_tgt).reshape((3,-1)), 
        )

        onrt = OnlineRostopicsToTimeseries(tfc) 
        onrt.start_publishing_timeseries(req.time_series_topic_name, blocking=False)


        return CollectSensorDataThenPublishItAsTimeSeriesResponse(
            time_series_remapping=time_series_remapping
        )

    else:
        # stop publishing
        rospy.loginfo('stop sensor data publishing')
        if onrt is not None:
            onrt.stop_publishing_timeseries()
            onrt = None
        return CollectSensorDataThenPublishItAsTimeSeriesResponse()

if __name__ == '__main__':
    rospy.init_node('sensor_data_time_series_publishing_service_node', log_level=rospy.INFO)
    config = util.get_config()

    server = rospy.Service('CollectSensorDataThenPublishItAsTimeSeries_service', CollectSensorDataThenPublishItAsTimeSeries, cb)
    rospy.spin()
