import baxter_pykdl
import PyKDL
import numpy as np

import pdb

class BaxterRightArmEndPointJacobianCalculator(object):
    def __init__(self):
        self.bk = baxter_pykdl.baxter_kinematics('right')

    def get_end_point_jacobian(self, joint_angles, end_point_offset=None):
        """
            We assume end_point_offset contains offsets of more than one end points
        """
        jac = PyKDL.Jacobian(7)
        self.bk._jac_kdl.JntToJac(self.bk.joints_to_kdl('position', joint_angles), jac)
        jac = self.bk.kdl_to_mat(jac)

        end_frame = PyKDL.Frame()
        self.bk._fk_p_kdl.JntToCart(self.bk.joints_to_kdl('position', joint_angles), end_frame)
        rot_mat = np.zeros((3,3))
        for i in range(3):
            for j in range(3):
                rot_mat[i][j] = end_frame.M[i, j]

        if end_point_offset is None:
            return jac

        num_of_points = end_point_offset.shape[0]
        end_point_jacs = []
        for i in range(num_of_points):
            offset = end_point_offset[i].reshape((-1, 1))
            i_jac_vel = jac[:3].copy()
            i_jac_rot = jac[3:].copy()

            i_jac_vel += np.cross(i_jac_rot, np.dot(rot_mat, offset), axisa=0, axisb=0, axisc=0)

            i_jac = np.vstack((i_jac_vel, i_jac_rot))
            end_point_jacs.append(i_jac)

        end_point_jacs = np.array(end_point_jacs)

        return end_point_jacs

