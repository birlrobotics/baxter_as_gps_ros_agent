import rospy

# The original GPS package doesn't use catkin.
# So the folloing lines are necessary if we want
#   to import packages from gps_agent_pkg
import roslib
roslib.load_manifest('gps_agent_pkg')

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

