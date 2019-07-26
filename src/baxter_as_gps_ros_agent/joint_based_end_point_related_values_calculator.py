from gps.proto.gps_pb2 import (
    END_EFFECTOR_POINTS,
    END_EFFECTOR_POINT_VELOCITIES,
    END_EFFECTOR_POINT_JACOBIANS,
    END_EFFECTOR_POINT_ROT_JACOBIANS,
    END_EFFECTOR_POSITIONS,
    END_EFFECTOR_ROTATIONS,
    END_EFFECTOR_JACOBIANS,
    TOTAL_DATA_TYPES,
)
import pdb
import baxter_pykdl
import kdl_util
import PyKDL
import numpy as np

TMP_END_EFFECTOR_LINEAR_VEL = TOTAL_DATA_TYPES+1
TMP_END_EFFECTOR_ANGULAR_VEL = TOTAL_DATA_TYPES+2

class JointBasedEndPointRelatedValuesCalculator(object):
    def __init__(
        self,
        values_to_calculate,
        end_point_offset=None,
        target_end_point=None,
    ):
        """
            We assume end_point_offset.shape = (3, num_of_points)
        """
        self.value_store = {}
        self.values_to_calculate = values_to_calculate
        self.end_point_offset = end_point_offset
        self.target_end_point = target_end_point
        self.bk = baxter_pykdl.baxter_kinematics('right')

    def get_calculation_result(self, joint_angles, joint_velocities):
        self.joint_angles = joint_angles
        self.joint_velocities = joint_velocities
        self.value_store = {}
        for i in self.values_to_calculate:
            if i == END_EFFECTOR_POINTS:
                self._calculate_offset_point_xyz()
            elif i == END_EFFECTOR_POSITIONS or i == END_EFFECTOR_ROTATIONS:
                self._calculate_end_effector_xyz_and_rotmat()
            elif i == END_EFFECTOR_POINT_VELOCITIES:
                self._calculate_offset_point_linear_velocity()
            elif i == END_EFFECTOR_JACOBIANS:
                self._calculate_end_effector_jacobian()
            elif i == END_EFFECTOR_POINT_ROT_JACOBIANS:
                self._calculate_offset_point_rot_jac()
            elif i == END_EFFECTOR_POINT_JACOBIANS:
                self._calculate_offset_point_vel_jac()
            else:
                raise Exception('type not supported yet')
        return self.value_store
    
            
    def _calculate_end_effector_xyz_and_rotmat(self):
        if self.value_store.get(END_EFFECTOR_POSITIONS, None) is not None\
            and self.value_store.get(END_EFFECTOR_ROTATIONS, None) is not None:
            return
        end_frame = PyKDL.Frame()
        kdl_array = kdl_util.array_to_kdl_JntArray(self.joint_angles)
        self.bk._fk_p_kdl.JntToCart(kdl_array, end_frame)
        xyz = np.zeros((3,1))
        for i in range(3):
            xyz[i][0] = end_frame.p[i]
        rotmat = np.zeros((3,3))
        for i in range(3):
            for j in range(3):
                rotmat[i][j] = end_frame.M[i, j]

        self.value_store[END_EFFECTOR_POSITIONS] = xyz
        self.value_store[END_EFFECTOR_ROTATIONS] = rotmat

    def _calculate_offset_point_xyz(self):
        if self.value_store.get(END_EFFECTOR_POINTS, None) is not None:
            return

        if self.end_point_offset is None:
            raise Exception("self.end_point_offset needed but not found")

        self._calculate_end_effector_xyz_and_rotmat()

        num_of_points = self.end_point_offset.shape[1]
        end_point_xyz = np.tile(
            self.value_store[END_EFFECTOR_POSITIONS], 
            (1, num_of_points)
        )
        end_point_xyz += np.dot(
            self.value_store[END_EFFECTOR_ROTATIONS], 
            self.end_point_offset
        )

        self.value_store[END_EFFECTOR_POINTS] = end_point_xyz

    def _calculate_end_effector_linear_and_angular_velocity(self):
        if self.value_store.get(TMP_END_EFFECTOR_LINEAR_VEL, None) is not None\
            and self.value_store.get(TMP_END_EFFECTOR_ANGULAR_VEL, None) is not None:
            return
        end_frame = PyKDL.FrameVel()
        jnt_array_vel = PyKDL.JntArrayVel(
            kdl_util.array_to_kdl_JntArray(self.joint_angles),
            kdl_util.array_to_kdl_JntArray(self.joint_velocities),
        )
        self.bk._fk_v_kdl.JntToCart(jnt_array_vel, end_frame)
        twist = end_frame.GetTwist()
        
        lin_vel = np.zeros((3,1))
        angular_vel = np.zeros((3,1))

        for i in range(3):
            lin_vel[i][0] = twist[i]
            angular_vel[i][0] = twist[i+3]

        self.value_store[TMP_END_EFFECTOR_LINEAR_VEL] = lin_vel
        self.value_store[TMP_END_EFFECTOR_ANGULAR_VEL] = angular_vel 

    def _calculate_offset_point_linear_velocity(self):
        if self.value_store.get(END_EFFECTOR_POINT_VELOCITIES, None) is not None:
            return

        if self.end_point_offset is None:
            raise Exception("self.end_point_offset needed but not found")

        self._calculate_end_effector_linear_and_angular_velocity()

        num_of_points = self.end_point_offset.shape[1]
        end_point_lin_vel = np.tile(
            self.value_store[TMP_END_EFFECTOR_LINEAR_VEL], 
            (1, num_of_points)
        )

        end_point_lin_vel += np.cross(
            self.value_store[TMP_END_EFFECTOR_ANGULAR_VEL], 
            np.dot(
                self.value_store[END_EFFECTOR_ROTATIONS], 
                self.end_point_offset
            ),
            axisa=0, axisb=0, axisc=0,
        )

    def _calculate_end_effector_jacobian(self):
        if self.value_store.get(END_EFFECTOR_JACOBIANS, None) is not None:
            return
        kdl_jac = PyKDL.Jacobian(7)
        kdl_array = kdl_util.array_to_kdl_JntArray(self.joint_angles)
        self.bk._jac_kdl.JntToJac(kdl_array, kdl_jac)
        jac = self.bk.kdl_to_mat(kdl_jac)
        self.value_store[END_EFFECTOR_JACOBIANS] = jac

    def _calculate_offset_point_rot_jac(self):
        if self.value_store.get(END_EFFECTOR_POINT_ROT_JACOBIANS, None) is not None:
            return

        self._calculate_end_effector_jacobian()

        num_of_points = self.end_point_offset.shape[1]

        rot_jac = np.tile(
            self.value_store[END_EFFECTOR_JACOBIANS][3:, :],
            (num_of_points,1),
        )

        self.value_store[END_EFFECTOR_POINT_ROT_JACOBIANS] = rot_jac

    def _calculate_offset_point_vel_jac(self):
        if self.value_store.get(END_EFFECTOR_POINT_JACOBIANS, None) is not None:
            return

        if self.end_point_offset is None:
            raise Exception("self.end_point_offset needed but not found")

        self._calculate_end_effector_jacobian()
        self._calculate_offset_point_rot_jac()
        self._calculate_end_effector_xyz_and_rotmat()

        num_of_points = self.end_point_offset.shape[1]

        vel_jac = np.tile(
            self.value_store[END_EFFECTOR_JACOBIANS][:3, :],
            (num_of_points,1),
        )
        rot_jac = self.value_store[END_EFFECTOR_POINT_ROT_JACOBIANS]

        rotmat = self.value_store[END_EFFECTOR_ROTATIONS]

        for i in range(num_of_points):
            vel_jac[3*i:3*i+3, :] += np.cross(
                rot_jac[3*i:3*i+3, :],
                np.dot(rotmat, self.end_point_offset[:, i]),
                axisa=0, axisb=0, axisc=0,
            )

        self.value_store[END_EFFECTOR_POINT_JACOBIANS] = vel_jac
