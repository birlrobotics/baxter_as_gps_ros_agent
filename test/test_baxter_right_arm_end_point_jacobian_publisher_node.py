#!/usr/bin/env python
import rospy
from baxter_as_gps_ros_agent.srv import SetUpEndPointOffsetAndPublishEndPointJacobian, SetUpEndPointOffsetAndPublishEndPointJacobianRequest, SetUpEndPointOffsetAndPublishEndPointJacobianResponse
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
import numpy as np

if __name__ == '__main__':
    rospy.init_node('test_baxter_right_arm_end_point_jacobian_publisher_node', log_level=rospy.DEBUG)

    proxy = rospy.ServiceProxy('SetUpEndPointOffsetAndPublishEndPointJacobian', SetUpEndPointOffsetAndPublishEndPointJacobian)

    end_point = np.array([
        [0.1, 0.2, 0.3],
        [-0.1, -0.2, -0.3],
    ])

    req = SetUpEndPointOffsetAndPublishEndPointJacobianRequest(
        end_point_offset = Float64MultiArray(
            data = end_point.flatten(),
            layout = MultiArrayLayout(
                dim=[MultiArrayDimension(size=i) for i in end_point.shape]
            ),
        )
    )

    print req
    
    proxy.call(req)

    rospy.sleep(3)


    req = SetUpEndPointOffsetAndPublishEndPointJacobianRequest()
    print req
    proxy.call(req)