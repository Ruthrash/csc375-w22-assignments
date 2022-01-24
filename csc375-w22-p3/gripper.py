#!/usr/bin/env python

import time
import threading
import numpy as np
import pybullet as p
import math


class Gripper:

    def __init__(self, robot):
        self.body_id = p.loadURDF("assets/gripper/robotiq_2f_85.urdf", [0.5, 0.1, 0.2], p.getQuaternionFromEuler([np.pi, 0, 0]))
        self.num_joints = p.getNumJoints(self.body_id)
        self.tip_offset = [0, 0, 0.17]
        self.gripper_range = [0, 0.03]

        p.createConstraint(robot.robot_id, robot.tool_joint_idx, self.body_id, 0, jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, -0.05], childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]))

        for i in range(p.getNumJoints(self.body_id)):
            p.changeDynamics(self.body_id, i, lateralFriction=1.0, spinningFriction=1.0, rollingFriction=0.0001, frictionAnchor=True)

        self.motor_joint_idx = 1
        self.constraints_thread = threading.Thread(target=self.step_constraints)
        self.constraints_thread.daemon = True
        self.constraints_thread.start()

    def step_constraints(self):
        """Add hard constraints for the gripper."""
        while True:
            joint_config = np.array([p.getJointState(self.body_id, i)[0] for i in range(self.num_joints)])
            p.setJointMotorControlArray(self.body_id, [6, 3, 8, 5, 10], p.POSITION_CONTROL, [joint_config[1], -joint_config[1], -joint_config[1], joint_config[1], joint_config[1]], positionGains=np.ones(5))
            time.sleep(0.001)

    def activate(self, blocking=True):
        """Close gripper fingers and check if the width between fingers exceeds some threshold to determine successful grasp."""
        open_angle = 0.715 - math.asin((self.gripper_range[1] - 0.010) / 0.1143)
        p.setJointMotorControl2(self.body_id, self.motor_joint_idx, p.POSITION_CONTROL, targetPosition=open_angle, force=1000)
        grasp_success = None
        if blocking:
            time.sleep(1)
            grasp_success = p.getJointState(self.body_id, 1)[0] < 0.834 - 0.001
        return grasp_success

    def release(self, blocking=True):
        """Open gripper fingers."""
        p.setJointMotorControl2(self.body_id, self.motor_joint_idx, p.VELOCITY_CONTROL, targetVelocity=-1, force=1000)
        if blocking:
            time.sleep(1)
