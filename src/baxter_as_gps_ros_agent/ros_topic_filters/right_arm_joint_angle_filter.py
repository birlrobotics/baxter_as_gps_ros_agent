from operator import itemgetter
from rostopics_to_timeseries import TopicMsgFilter
class RightArmJointAngleFilter(TopicMsgFilter):
    all_joints = ["head_pan", "l_gripper_l_finger_joint", "l_gripper_r_finger_joint", "left_e0", "left_e1", "left_s0",
"left_s1", "left_w0", "left_w1", "left_w2", "r_gripper_l_finger_joint", "r_gripper_r_finger_joint",
"right_e0", "right_e1", "right_s0", "right_s1", "right_w0", "right_w1", "right_w2"]
    target_joints = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
    def __init__(self):
        super(RightArmJointAngleFilter, self).__init__()
    
        self.getter = itemgetter(*[RightArmJointAngleFilter.all_joints.index(i) for i in RightArmJointAngleFilter.target_joints])

    @staticmethod
    def vector_size():
        return len(RightArmJointAngleFilter.target_joints)

class RightArmJointAngleFilterPosition(RightArmJointAngleFilter):
    def __init__(self):
        super(RightArmJointAngleFilterPosition, self).__init__()

    def convert(self, msg):
        return self.getter(msg.position)

    @staticmethod
    def vector_meaning():
        return [i+'.position' for i in RightArmJointAngleFilter.target_joints]

class RightArmJointAngleFilterVelocity(RightArmJointAngleFilter):
    def __init__(self):
        super(RightArmJointAngleFilterVelocity, self).__init__()

    def convert(self, msg):
        return self.getter(msg.velocity)

    @staticmethod
    def vector_meaning():
        return [i+'.velocity' for i in RightArmJointAngleFilter.target_joints]
