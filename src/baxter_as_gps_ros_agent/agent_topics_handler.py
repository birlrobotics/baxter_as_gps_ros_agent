import pdb

import rospy

import os
import gps_agent_pkg
import sys
sys.path.append(os.path.join(
    os.sep.join(gps_agent_pkg.__path__[0].split(os.sep)[:-7]),
    'python',
))
from gps.proto.gps_pb2 import (
    ACTION, TRIAL_ARM, AUXILIARY_ARM, JOINT_SPACE,
    JOINT_ANGLES,
    JOINT_VELOCITIES,
    END_EFFECTOR_POINTS,
    END_EFFECTOR_POINT_VELOCITIES,
    END_EFFECTOR_POINT_JACOBIANS,
    END_EFFECTOR_POINT_ROT_JACOBIANS,
    END_EFFECTOR_POSITIONS,
    END_EFFECTOR_ROTATIONS,
    END_EFFECTOR_JACOBIANS,
    LIN_GAUSS_CONTROLLER,
)

from gps_agent_pkg.msg import (
    TrialCommand, 
    PositionCommand, 
    RelaxCommand, 
    DataRequest, 
    SampleResult, 
)
from baxter_as_gps_ros_agent.srv import RecordSensorsToRosbagThenReturnSample, RecordSensorsToRosbagThenReturnSampleRequest, RecordSensorsToRosbagThenReturnSampleResponse
from baxter_as_gps_ros_agent.srv import CollectSensorDataThenPublishItAsTimeSeries, CollectSensorDataThenPublishItAsTimeSeriesRequest, CollectSensorDataThenPublishItAsTimeSeriesResponse
from baxter_as_gps_ros_agent.srv import SetupControllerAndRolloutOneEpisode, SetupControllerAndRolloutOneEpisodeRequest, SetupControllerAndRolloutOneEpisodeResponse
from baxter_as_gps_ros_agent import CONSTANT

class AgentTopicsHandler(object):
    def __init__(self):
        pass

    def _trial_command_callback(self, trial_command):
        rospy.logdebug('receive trial command: %100s'%trial_command)
        state_datatypes = trial_command.state_datatypes
        observation_datatypes = trial_command.obs_datatypes
        controller_type = trial_command.controller.controller_to_execute

        s = set(state_datatypes+observation_datatypes)
        s.add(ACTION)
        s.add(JOINT_ANGLES)
        s.add(JOINT_VELOCITIES)
        s.add(END_EFFECTOR_POINTS)
        s.add(END_EFFECTOR_POINT_VELOCITIES)
        s.add(END_EFFECTOR_POINT_JACOBIANS)
        s.add(END_EFFECTOR_POINT_ROT_JACOBIANS)
        s.add(END_EFFECTOR_POSITIONS)
        s.add(END_EFFECTOR_ROTATIONS)
        s.add(END_EFFECTOR_JACOBIANS)
        sample_datatypes = list(s)
        req = RecordSensorsToRosbagThenReturnSampleRequest(
            frequency=trial_command.frequency,
            datatypes=sample_datatypes,
            ee_points=trial_command.ee_points,
            ee_points_tgt=trial_command.ee_points_tgt,
        )
        resp = self.sampling_service.call(req)


        if controller_type == LIN_GAUSS_CONTROLLER:
            sensor_datatypes = state_datatypes
            time_series_topic_name = 'time_series_state_x'
        else:
            sensor_datatypes = observation_datatypes
            time_series_topic_name = 'time_series_obs_o'

        req = CollectSensorDataThenPublishItAsTimeSeriesRequest(
            frequency=trial_command.frequency,
            datatypes=sensor_datatypes,
            ee_points=trial_command.ee_points,
            ee_points_tgt=trial_command.ee_points_tgt,
            time_series_topic_name=time_series_topic_name,
        ) 
        resp = self.sensor_data_time_series_publishing_service(req)

        time_series_remapping = resp.time_series_remapping

        req = SetupControllerAndRolloutOneEpisodeRequest(
            time_series_remapping=time_series_remapping,
            controller=trial_command.controller,
            frequency=trial_command.frequency,
            T=trial_command.T,
            publish_action_to_this_topic=CONSTANT.action_topic,
            sensor_time_series_topic = time_series_topic_name,
        )
        resp = self.make_action_and_rollout_one_episode_service.call(req)
        start_time_in_sec = resp.episode_start_time_in_sec
        end_time_in_sec = resp.episode_end_time_in_sec

        # stop sampling
        req = RecordSensorsToRosbagThenReturnSampleRequest(
            episode_start_time_in_sec=start_time_in_sec,
            episode_end_time_in_sec=end_time_in_sec, 
            T=trial_command.T,
        )
        resp = self.sampling_service.call(req)
        
        sample_result = resp.sample_result
        self.sample_result_pub.publish(sample_result)

        # stop sensor data publishing
        req = CollectSensorDataThenPublishItAsTimeSeriesRequest()
        resp = self.sensor_data_time_series_publishing_service(req)

    def _position_command_callback(self, position_command):
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

    def _relax_command_callback(self, relax_command):
        rospy.logdebug('receive relax command: %s'%relax_command)

    def _data_request_callback(self, data_request):
        rospy.logdebug('receive data request: %s'%data_request)

    def setup_subscriber_and_publisher_and_service(self):
        self.trial_command_sub = rospy.Subscriber("gps_controller_trial_command", TrialCommand, self._trial_command_callback)
        self.position_command_sub = rospy.Subscriber("gps_controller_position_command", PositionCommand, self._position_command_callback)
        self.relax_command_sub = rospy.Subscriber("gps_controller_relax_command", RelaxCommand, self._relax_command_callback)
        self.data_request_sub = rospy.Subscriber("gps_controller_data_request", DataRequest, self._data_request_callback)
        self.sample_result_pub = rospy.Publisher("gps_controller_report", SampleResult, queue_size=None)
        self.sampling_service = rospy.ServiceProxy('/sampling_service_for_gps_baxter', RecordSensorsToRosbagThenReturnSample)
        self.sensor_data_time_series_publishing_service= rospy.ServiceProxy('CollectSensorDataThenPublishItAsTimeSeries_service', CollectSensorDataThenPublishItAsTimeSeries)
        self.make_action_and_rollout_one_episode_service = rospy.ServiceProxy("SetupControllerAndRolloutOneEpisode_service", SetupControllerAndRolloutOneEpisode)

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
