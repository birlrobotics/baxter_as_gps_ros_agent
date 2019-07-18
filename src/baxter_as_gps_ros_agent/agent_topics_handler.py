import pdb

import rospy

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
        END_EFFECTOR_POINTS, END_EFFECTOR_POINT_VELOCITIES, ACTION, \
        TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE

from gps_agent_pkg.msg import (
    TrialCommand, 
    PositionCommand, 
    RelaxCommand, 
    DataRequest, 
    SampleResult, 
)

class AgentTopicsHandler(object):
    def __init__(self):
        pass

    def trial_command_callback(self, trial_command):
        rospy.logdebug('receive trial command: %s'%trial_command)

    def position_command_callback(self, position_command):
        rospy.logdebug('receive position command: %s'%position_command)

        mode_id = position_command.mode
        arm_id = position_command.arm
        if mode_id == JOINT_SPACE:
            joint_names = self.arms[arm_id].joint_names()
            joint_angles = position_command.data
            self.arms[arm_id].move_to_joint_positions(dict(zip(joint_names, joint_angles)))
        else:
            raise Exception("mode %s unsupported yet"%mode)

        self.sample_result_pub.publish(SampleResult())

    def relax_command_callback(self, relax_command):
        rospy.logdebug('receive relax command: %s'%relax_command)

    def data_request_callback(self, data_request):
        rospy.logdebug('receive data request: %s'%data_request)

    def setup_subscriber_and_publisher(self):
        self.trial_command_sub = rospy.Subscriber("gps_controller_trial_command", TrialCommand, self.trial_command_callback)
        self.position_command_sub = rospy.Subscriber("gps_controller_position_command", PositionCommand, self.position_command_callback)
        self.relax_command_sub = rospy.Subscriber("gps_controller_relax_command", RelaxCommand, self.relax_command_callback)
        self.data_request_sub = rospy.Subscriber("gps_controller_data_request", DataRequest, self.data_request_callback)
        self.sample_result_pub = rospy.Publisher("gps_controller_report", SampleResult, queue_size=None)

    def setup_baxter(self):
        import baxter_interface
        from baxter_interface import CHECK_VERSION

        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        rs.enable()
        left_arm = baxter_interface.Limb('left')
        right_arm = baxter_interface.Limb('right')

        self.arms = {
            TRIAL_ARM: right_arm, 
            AUXILIARY_ARM: left_arm,
        }


