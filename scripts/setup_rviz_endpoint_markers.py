#!/usr/bin/env python

import pdb
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import ColorRGBA, Header
import sys
import os
import imp
import roslib

roslib.load_manifest('gps_agent_pkg')
import gps_agent_pkg

sys.path.append(os.path.join(
    os.path.join(gps_agent_pkg.__path__[0], '..', '..', '..', '..'),
    'python',
))

hyperparams = imp.load_source('hyperparams', '/home/sklaw/Coding/birl_gps/gps/experiments/baxter_badmm_example/hyperparams.py')


ee_points = hyperparams.EE_POINTS.flatten()
ee_points_tgt = hyperparams.agent['ee_points_tgt'][0]

point_num = len(ee_points)/3

ee_point_list = []
ee_tgt_point_list = []
for i in range(point_num):
    ee_point_list.append({
        'x':ee_points[3*i],
        'y':ee_points[3*i+1],
        'z':ee_points[3*i+2],
    })
    ee_tgt_point_list.append({
        'x':ee_points_tgt[3*i],
        'y':ee_points_tgt[3*i+1],
        'z':ee_points_tgt[3*i+2],
    })

if __name__ == '__main__':
    rospy.init_node("setup_rviz_endpoint_markers_node")
    marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=None)
    rospy.sleep(1)
    ee_marker_array = MarkerArray()
    ee_tgt_marker_array = MarkerArray()

    import colorsys
    N = point_num
    HSV_tuples = [(x*1.0/N, 0.5, 0.5) for x in range(N)]
    RGB_tuples = map(lambda x: colorsys.hsv_to_rgb(*x)+(1,), HSV_tuples)

    for i in range(point_num):
        marker = Marker(
            id=i*2,
            ns='ee_points',
            header=Header(
                stamp=rospy.Time(),
                frame_id='/right_gripper',
            ),
            type=Marker.SPHERE,
            pose=Pose(
                position=Point(**ee_point_list[i])
            ),
            color=ColorRGBA(*RGB_tuples[i]),
            scale=Vector3(0.01,0.01,0.01),
        )
        ee_marker_array.markers.append(marker)

        marker = Marker(
            id=i*2+1,
            ns='ee_points',
            header=Header(
                stamp=rospy.Time(),
                frame_id='/base',
            ),
            type=Marker.SPHERE,
            pose=Pose(
                position=Point(**ee_tgt_point_list[i])
            ),
            color=ColorRGBA(*RGB_tuples[i]),
            scale=Vector3(0.01,0.01,0.01),
        )
        ee_tgt_marker_array.markers.append(marker)

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        marker_pub.publish(ee_marker_array)
        marker_pub.publish(ee_tgt_marker_array)
        rate.sleep()
        
