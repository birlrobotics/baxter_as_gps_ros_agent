# The original GPS package doesn't use catkin.
# So the folloing lines are necessary if we want
#   to import packages from gps_agent_pkg
import roslib
roslib.load_manifest('gps_agent_pkg')

import os
import gps_agent_pkg
import sys
sys.path.append(os.path.join(
    os.sep.join(gps_agent_pkg.__path__[0].split(os.sep)[:-4]),
    'python',
))

from gps.proto.gps_pb2 import JOINT_ANGLES, JOINT_VELOCITIES, \
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION
from rostopics_to_timeseries import RosTopicFilteringScheme, OfflineRostopicsToTimeseries
from baxter_as_gps_ros_agent.ros_topic_filters.right_arm_joint_angle_filter import RightArmJointAngleFilter
from sensor_msgs.msg import JointState

map_datatype_to_filter = {
    JOINT_ANGLES: {
        "topic_name": "/robot/joint_states",
        "msg_type": JointState,
        "filter_class": RightArmJointAngleFilter,
    }
}
