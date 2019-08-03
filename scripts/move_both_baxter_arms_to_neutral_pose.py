#!/usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

if __name__ == '__main__':
    rospy.init_node("move_both_baxter_arms_to_neutral_pose_node")

    left_arm = baxter_interface.limb.Limb("left")
    right_arm = baxter_interface.limb.Limb("right")

    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rospy.loginfo("Enabling robot... ")
    rs.enable()

    rospy.sleep(1)
    left_arm.move_to_neutral()
    right_arm.move_to_neutral()
    rospy.sleep(1)

