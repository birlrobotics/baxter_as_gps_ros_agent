#!/usr/bin/env python
import rospy
from baxter_as_gps_ros_agent.srv import RecordSensorsToRosbagThenReturnSample, RecordSensorsToRosbagThenReturnSampleRequest, RecordSensorsToRosbagThenReturnSampleResponse
import argparse
import pdb
import imp

def cb(RecordSensorsToRosbagThenReturnSampleRequest):
    rospy.logdebug("RecordSensorsToRosbagThenReturnSampleRequest: %s"%RecordSensorsToRosbagThenReturnSampleRequest)

if __name__ == '__main__':
    parser = argparse.ArgumentParser() 
    parser.add_argument("config", type=str, help='path to config file')

    args = parser.parse_args()
    config = imp.load_source('config', args.config)

    pdb.set_trace()

    rospy.init_node('sampling_service_service_node', log_level=rospy.DEBUG)
    server = rospy.Service('/sampling_service_for_gps_baxter', RecordSensorsToRosbagThenReturnSample, cb)
    rospy.spin()
