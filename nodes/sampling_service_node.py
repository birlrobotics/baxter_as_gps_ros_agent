#!/usr/bin/env python
import rospy
from baxter_as_gps_ros_agent.srv import RecordSensorsToRosbagThenReturnSample, RecordSensorsToRosbagThenReturnSampleRequest, RecordSensorsToRosbagThenReturnSampleResponse
import argparse
import pdb
import imp
from baxter_as_gps_ros_agent import RosbagProc
import tempfile
import numpy as np

rosbag_proc = None
rosbag_path = None
saved_req = None

def cb(req):
    global rosbag_proc, rosbag_path, saved_req
    if len(req.datatypes) != 0:
        saved_req = req
        # start recording
        rospy.logdebug("req: %s"%req)
        topics_to_be_recorded_into_rosbag = config.get_topic_names_that_will_be_recorded_into_rosbag(req.datatypes)
        rospy.logdebug('topcis_to_record: %s'%topics_to_be_recorded_into_rosbag)
        rosbag_f = tempfile.NamedTemporaryFile(delete=False, suffix='.bag')
        rosbag_f.close()
        rosbag_path = rosbag_f.name
        rospy.loginfo('start rosbag recording process, temporary rosbag path: %s'%rosbag_path)

        if rosbag_proc is not None:
            rosbag_proc.stop()
    
        rosbag_proc = RosbagProc(
            rosbag_path,
            topics_to_be_recorded_into_rosbag
        )
        rosbag_proc.start()
        return RecordSensorsToRosbagThenReturnSampleResponse()
    else:
        # stop rosbag recording and process it to SampleResult

        # Note, we should use saved_req instead of req here
        # we set req to be None here so no misuse is possible
        req = None

        rospy.loginfo('stop rosbag recording process')
        if rosbag_proc is not None:
            rosbag_proc.stop()
            rospy.sleep(1)

            # TODO: process rosbag into sample result
            sample_result = config.process_rosbag_to_SampleResult(
                rosbag_path,
                saved_req.datatypes,
                saved_req.frequency,
                np.array(saved_req.ee_points).reshape((3,-1)),
                np.array(saved_req.ee_points_tgt).reshape((3,-1)), 
            )

            rosbag_proc = None
            rosbag_path = None
            saved_req = None
        return RecordSensorsToRosbagThenReturnSampleResponse(sample_result=sample_result)
        

if __name__ == '__main__':
    parser = argparse.ArgumentParser() 
    parser.add_argument("config", type=str, help='path to config file')

    args = parser.parse_args()
    config = imp.load_source('config', args.config)

    rospy.init_node('sampling_service_service_node', log_level=rospy.DEBUG)
    server = rospy.Service('/sampling_service_for_gps_baxter', RecordSensorsToRosbagThenReturnSample, cb)
    rospy.spin()