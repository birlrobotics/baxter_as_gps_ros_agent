import baxter_pykdl
import pdb

class BaxterRightArmEndPointJacobianCalculator(object):
    def __init__(self):
        self.bk = baxter_pykdl.baxter_kinematics('right')

    def get_end_point_jacobian(self, joint_angles, end_point_offset=None):
        jac = self.bk.jacobian(joint_angles)
        if end_point_offset is None:
            return jac
