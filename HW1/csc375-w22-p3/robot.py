import time
import numpy as np
import pybullet as p

class Robot():

    def __init__(self):
        self.robot_id = p.loadURDF('assets/ur5/ur5.urdf')
        num_joints = p.getNumJoints(self.robot_id)

        joint_info = [p.getJointInfo(self.robot_id, i) for i in range(num_joints)]
        self.joint_indices = [x[0] for x in joint_info if x[2] == p.JOINT_REVOLUTE]
        self.tool_joint_idx = [x[0] for x in joint_info if "tool0_fixed_joint" in x[1].decode("utf-8")][0]
        self.home_joints = np.array([-np.pi, -np.pi / 2 - 0.1, np.pi / 2, -np.pi / 2, -np.pi / 2, 0])
        self.num_joints = len(self.joint_indices)

    def move_joints(self, target_joints, blocking=False, velocity=0.03):
        """Move to a target joints configuration with a given velocity. Option to do so in blocking or non-blocking mode."""
        ########### YOUR CODE STARTS HERE ###########
        raise NotImplementedError
        ########### YOUR CODE ENDS HERE ###########


    def move_pose(self, target_pose, blocking=False, velocity=0.03):
        """Move to a target pose with a given velocity. Option to do so in blocking or non-blocking mode."""
        ########### YOUR CODE STARTS HERE ###########
        raise NotImplementedError
        ########### YOUR CODE ENDS HERE ###########

    def go_home(self, blocking=False, velocity=0.1):
        """Go to home position."""
        self.move_joints(self.home_joints)
