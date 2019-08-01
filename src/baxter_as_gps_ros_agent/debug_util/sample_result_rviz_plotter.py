import pdb
from gps.proto.gps_pb2 import (
    ACTION,
    JOINT_ANGLES,
    JOINT_VELOCITIES,
    END_EFFECTOR_POINTS,
    END_EFFECTOR_POINT_VELOCITIES,
    END_EFFECTOR_POINT_JACOBIANS,
    END_EFFECTOR_POINT_ROT_JACOBIANS,
    END_EFFECTOR_POSITIONS,
    END_EFFECTOR_ROTATIONS,
    END_EFFECTOR_JACOBIANS,
)
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Header
import rospy
import numpy as np

frame_id = 'base'
def set_frame_id(link):
    global frame_id 
    frame_id = link

def plot_target_end_point(ee_points_tgt):
    _send_rviz_pose_array(ee_points_tgt, color=ColorRGBA(*[0, 0, 1, 1]))

def plot(sample_result, ee_points_tgt=None):
    for i in sample_result.sensor_data:
        data_type=i.data_type
        shape=i.shape
        data=np.array(i.data).reshape(shape)
        if data_type == END_EFFECTOR_POINTS:
            if ee_points_tgt is not None:
                data += ee_points_tgt 
            number_of_points = shape[1]/3
            for j in range(number_of_points):
                _send_rviz_pose_array(data[:, 3*j:3*j+3], color=ColorRGBA(*[0, 1, 0, 1])) 
        elif data_type == END_EFFECTOR_POSITIONS:
            _send_rviz_pose_array(data.reshape(shape), color=ColorRGBA(*[1, 0, 0, 1]))


marker_last_id = 0
marker_pub = None
def delete_prev_markers():
    global marker_last_id, marker_pub

    if marker_pub is None:
        marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=None)
        rospy.sleep(1)

    # delete previous batch
    marker_array = MarkerArray()
    for i in range(marker_last_id):
        marker = Marker(id=i, action=Marker.DELETE)
        marker_array.markers.append(marker)
    marker_pub.publish(marker_array)
    marker_last_id = 0

def _send_rviz_pose_array(position_matrix, quaternion_matrix=None, color=None):
    global marker_last_id, frame_id, marker_pub

    if marker_pub is None:
        marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=None)
        rospy.sleep(1)
    number_of_marker = position_matrix.shape[0]

    if quaternion_matrix is None:
        quaternion_matrix = np.zeros((number_of_marker, 4)) 
        quaternion_matrix[:, 3] = 1
    if color is None:
        color = ColorRGBA(*[0, 1, 0, 1])

    marker_array = MarkerArray()
    for i in range(number_of_marker):
        marker = Marker(
            id=marker_last_id,
            ns='plot_sample_result',
            header=Header(
                stamp=rospy.Time(),
                frame_id=frame_id,
            ),
            type=Marker.ARROW,
            pose=Pose(
                position=Point(
                    x=position_matrix[i][0],
                    y=position_matrix[i][1],
                    z=position_matrix[i][2],
                ),
                orientation=Quaternion(
                    x=quaternion_matrix[i][0],
                    y=quaternion_matrix[i][1],
                    z=quaternion_matrix[i][2],
                    w=quaternion_matrix[i][3],
                ),
            ),
            color=color,
            scale=Vector3(0.01,0.01,0.01),
            action=Marker.ADD
        )
        marker_last_id += 1
        marker_array.markers.append(marker)
    marker_pub.publish(marker_array)
