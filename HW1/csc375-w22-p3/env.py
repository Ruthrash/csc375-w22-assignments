import pybullet as p
import pybullet_data
import numpy as np
import time
import glob
import os
import random

from robot import Robot
from camera import RealSenseD415
from gripper import Gripper

COLORS = {
    'blue': [078.0 / 255.0, 121.0 / 255.0, 167.0 / 255.0],
    'red': [255.0 / 255.0, 087.0 / 255.0, 089.0 / 255.0],
    'green': [089.0 / 255.0, 169.0 / 255.0, 079.0 / 255.0],
    'orange': [242.0 / 255.0, 142.0 / 255.0, 043.0 / 255.0],
    'yellow': [237.0 / 255.0, 201.0 / 255.0, 072.0 / 255.0],
    'purple': [176.0 / 255.0, 122.0 / 255.0, 161.0 / 255.0],
    'pink': [255.0 / 255.0, 157.0 / 255.0, 167.0 / 255.0],
    'cyan': [118.0 / 255.0, 183.0 / 255.0, 178.0 / 255.0],
    'brown': [156.0 / 255.0, 117.0 / 255.0, 095.0 / 255.0],
    'gray': [186.0 / 255.0, 176.0 / 255.0, 172.0 / 255.0]
}

class SIMEnv():
    def __init__(self):
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.setPhysicsEngineParameter(enableFileCaching=0)
        p.setRealTimeSimulation(1)
        p.setGravity(0, 0, -9.8)
        self.robot = None
        self.camera = None
        self._urdf_root = pybullet_data.getDataPath()
        self.reset()

    def seed(self, seed=None):
        self._random = np.random.RandomState(seed)
        return seed

    def reset(self):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf", [0, 0, -0.001])
        p.loadURDF("assets/ur5/workspace.urdf", [0.45, 0, 0])
        p.loadURDF("assets/ur5/mount.urdf", [0, 0, 0.1])
        p.loadURDF("tray/traybox.urdf", [0.45, 0.3, 0], globalScaling=0.7)

        self.seed()
        self.robot = Robot()
        self.robot.go_home(blocking=True, velocity=1.0)

        self.camera = RealSenseD415()
        self.camera.render_camera(self._random)

        self.gripper = Gripper(self.robot)
        self.objs = self._randomly_place_objects()
        # Give simulator time to catch up
        time.sleep(1.)

    def get_updated_block_positions(self):
        objectPosQuat = dict()
        for k in self.objs.keys():
            blockPos, blockOrn = p.getBasePositionAndOrientation(k)
            objectPosQuat[k] = [blockPos, blockOrn]

        self.objs = objectPosQuat

    def _randomly_place_objects(self):
        """Randomly choose a number of blocks between 1-10 and place them on the workspace outside of the bin on random positions."""
        colors = [
            COLORS['purple'], COLORS['blue'], COLORS['green'],
            COLORS['yellow'], COLORS['orange'], COLORS['red']
        ]
        how_many = np.random.randint(low=1, high=10, size=(1,))[0]
        block_random = np.random.uniform(low=0, high=1, size=(1,))
        objectUids = []
        objectPosQuat = dict()
        obj_x_range = (0.451, 0.55)
        obj_y_range = (-0.25, -0.5)
        for i in range(how_many):
          xpos = np.random.uniform(low=obj_x_range[0], high=obj_x_range[1], size=(1,))[0]
          ypos = np.random.uniform(low=obj_y_range[0], high=obj_y_range[1], size=(1,))[0]
          angle = np.pi / 2 + block_random * np.pi * random.random()
          orn = p.getQuaternionFromEuler([0, 0, angle])
          uid = p.loadURDF("assets/block/block.urdf", [xpos, ypos, .0], [orn[0], orn[1], orn[2], orn[3]])
          p.changeVisualShape(uid, -1, rgbaColor=colors[i % 6] + [1])
          objectUids.append(uid)
          objectPosQuat[uid] = [[xpos, ypos, .15], [orn[0], orn[1], orn[2], orn[3]]]
          # Let each object fall to the tray individual, to prevent object
          # intersection.
          for _ in range(500):
            p.stepSimulation()
        return objectPosQuat
