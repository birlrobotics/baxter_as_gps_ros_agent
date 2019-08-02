#!/usr/bin/env python
import pdb
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from tf.transformations import quaternion_matrix

if __name__ == '__main__':
    rospy.init_node("just_a_node")

    right_arm = baxter_interface.limb.Limb("right")

    d = right_arm.endpoint_pose()
    pos = d['position']
    ori = d['orientation']

    print 'pos'
    print [pos.x, pos.y, pos.z]
    print 'rot mat'
    print quaternion_matrix(ori)[:3, :3].tolist()
    

