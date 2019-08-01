import rospy
from gps_agent_pkg.msg import (
    TrialCommand, 
    PositionCommand, 
    RelaxCommand, 
    DataRequest, 
    SampleResult, 
)
from baxter_as_gps_ros_agent.debug_util import sample_result_rviz_plotter
import numpy as np
import pdb

ee_points_tgt = None
def trial_command_callback(msg):
    global ee_points_tgt
    ee_points_tgt = msg.ee_points_tgt

def sample_result_callback(msg):
    global ee_points_tgt

    if ee_points_tgt is None:
        return
    elif len(ee_points_tgt) != 9:
        return

    sample_result = msg
    sample_result_rviz_plotter.set_frame_id('torso_lift_link')
    sample_result_rviz_plotter.delete_prev_markers()
    sample_result_rviz_plotter.plot_target_end_point(np.array(ee_points_tgt).reshape((3,-1)))
    sample_result_rviz_plotter.plot(sample_result, ee_points_tgt)

if __name__ == '__main__':
    rospy.init_node("spoof_pr2_demo_node")
    trial_command_sub = rospy.Subscriber("gps_controller_trial_command", TrialCommand, trial_command_callback)
    sample_result_pub = rospy.Subscriber("gps_controller_report", SampleResult, sample_result_callback)
    rospy.spin()
