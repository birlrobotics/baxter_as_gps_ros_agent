import rospy

# The original GPS package doesn't use catkin.
# So the folloing lines are necessary if we want
#   to import packages from gps_agent_pkg
import roslib
roslib.load_manifest('gps_agent_pkg')

from gps_agent_pkg.msg import (
    TrialCommand, 
    SampleResult, 
    PositionCommand, 
    RelaxCommand, 
    DataRequest, 
    TfActionCommand, 
    TfObsData,
)
import trial_command_handler

class AgentTopicsHandler(object):
    def __init__(self):
        pass

    def setup_subscriber_and_publisher(self):
        rospy.Subscriber("gps_controller_trial_command", TrialCommand, trial_command_handler.callback)

