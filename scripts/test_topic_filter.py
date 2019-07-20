#!/usr/bin/env python

from rostopics_to_timeseries import (
    RosTopicFilteringScheme, 
    OnlineRostopicsToTimeseries,
)

import rospy
from scipy import signal
import time

import argparse
import imp

if __name__ == '__main__':
    parser = argparse.ArgumentParser() 
    parser.add_argument("config", type=str, help='path to config file')
    args = parser.parse_args()
    config = imp.load_source('config', args.config)

    rospy.init_node("test_topic_filter_node")
    time.sleep(1)

    tfc = RosTopicFilteringScheme(resampling_rate=100)
    for k, v in config.map_datatype_to_filter.iteritems():
        tfc.add_filter(**v)

    print tfc.info

    onrt = OnlineRostopicsToTimeseries(tfc) 
    onrt.start_publishing_timeseries("/rostopics_to_timeseries_topic")
