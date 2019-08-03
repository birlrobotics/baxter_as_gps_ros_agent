#!/usr/bin/env python

import pdb
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import ColorRGBA, Header
import sys
import os
import imp
import argparse
import numpy as np

import roslib
roslib.load_manifest('gps_agent_pkg')
import gps_agent_pkg

sys.path.append(os.path.join(
    os.path.join(gps_agent_pkg.__path__[0], '..', '..', '..', '..'),
    'python',
))




if __name__ == '__main__':
    parser = argparse.ArgumentParser() 
    parser.add_argument("hyperparams_path", type=str, help='path to hyperparams.py')
    parser.add_argument("--base-frame", dest='base_frame', type=str, help='base frame', default='/base')
    parser.add_argument("--endeffector-frame", dest='endeffector_frame', type=str, help='endeffector frame', default='/right_gripper')

    args = parser.parse_args()

    rospy.init_node("show_target_end_points_in_rviz_node")


    hyperparams = imp.load_source('hyperparams', args.hyperparams_path)

    ee_points_tgt = hyperparams.agent['ee_points_tgt']
    ee_points = hyperparams.EE_POINTS

    number_of_targets = len(ee_points_tgt)
    number_of_offset_points = ee_points.shape[0]

    ee_points_tgt = np.array(ee_points_tgt).reshape((number_of_targets, number_of_offset_points, 3))

    marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=None)
    rospy.sleep(1)
    ee_tgt_marker_array = MarkerArray()

    import colorsys
    N = number_of_offset_points
    HSV_tuples = [(x*1.0/N, 0.5, 0.5) for x in range(N)]
    RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x)+(1,), HSV_tuples)

    scale = Vector3(0.05,0.05,0.05)

    marker_id = 0
    for i in range(number_of_targets):
        for j in range(number_of_offset_points):
            marker = Marker(
                id=marker_id,
                ns='target_end_points',
                header=Header(
                    stamp=rospy.Time(),
                    frame_id=args.base_frame,
                ),
                type=Marker.SPHERE,
                pose=Pose(
                    position=Point(*ee_points_tgt[i][j])
                ),
                color=ColorRGBA(*RGB_tuples[j]),
                scale=scale,
            )
            ee_tgt_marker_array.markers.append(marker)
            marker_id += 1

    for i in range(number_of_offset_points):
        marker = Marker(
            id=marker_id,
            ns='target_end_points',
            header=Header(
                stamp=rospy.Time(),
                frame_id=args.endeffector_frame,
            ),
            type=Marker.SPHERE,
            pose=Pose(
                position=Point(*ee_points[i])
            ),
            color=ColorRGBA(*RGB_tuples[i]),
            scale=scale,
        )
        ee_tgt_marker_array.markers.append(marker)
        marker_id += 1

    print ee_tgt_marker_array
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        marker_pub.publish(ee_tgt_marker_array)
        rate.sleep()
        
