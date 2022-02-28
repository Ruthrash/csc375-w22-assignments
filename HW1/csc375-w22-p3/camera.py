import numpy as np
import pybullet as p
import threading
import time


class RealSenseD415():

    def __init__(self):
        # Mimic RealSense D415 RGB-D camera parameters.
        self.fps = 10
        self.image_size = (480, 640)
        self.intrinsics = (450., 0, 320., 0, 450., 240., 0, 0, 1)
        self.zrange = (0.01, 10.)
        self.noise = False

        # Set default camera pose.
        self.position = (1., 0, 0.75)
        self.rotation = (np.pi / 4, np.pi, -np.pi / 2)
        self.rotation = p.getQuaternionFromEuler(self.rotation)

    def render_camera(self, random_seed):
        # OpenGL camera settings.
        lookdir = np.float32([0, 0, 1]).reshape(3, 1)
        updir = np.float32([0, -1, 0]).reshape(3, 1)
        rot = p.getMatrixFromQuaternion(self.rotation)
        rotm = np.float32(rot).reshape(3, 3)
        lookdir = (rotm @ lookdir).reshape(-1)
        updir = (rotm @ updir).reshape(-1)
        lookat = self.position + lookdir
        focal_len = self.intrinsics[0]
        znear, zfar = self.zrange
        self.view_matrix = p.computeViewMatrix(self.position, lookat, updir)
        fovh = (self.image_size[0] / 2) / focal_len
        fovh = 180 * np.arctan(fovh) * 2 / np.pi

        # Notes: 1) FOV is vertical FOV 2) aspect must be float
        aspect_ratio = self.image_size[1] / self.image_size[0]
        self.projection_matrix = p.computeProjectionMatrixFOV(fovh, aspect_ratio, znear, zfar)

        # Render with OpenGL camera settings.
        _, _, color, depth, segm = p.getCameraImage(
            width=self.image_size[1],
            height=self.image_size[0],
            viewMatrix=self.view_matrix,
            projectionMatrix=self.projection_matrix,
            shadow=1,
            flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
            # Note when use_egl is toggled, this option will not actually use openGL
            # but EGL instead.
            renderer=p.ER_BULLET_HARDWARE_OPENGL)

        # Get color image.
        color_image_size = (self.image_size[0], self.image_size[1], 4)
        color = np.array(color, dtype=np.uint8).reshape(color_image_size)
        color = color[:, :, :3]  # remove alpha channel
        if self.noise:
          color = np.int32(color)
          color += np.int32(random_seed.normal(0, 3, self.image_size))
          color = np.uint8(np.clip(color, 0, 255))

        # Get depth image.
        depth_image_size = (self.image_size[0], self.image_size[1])
        zbuffer = np.array(depth).reshape(depth_image_size)
        depth = (zfar + znear - (2. * zbuffer - 1.) * (zfar - znear))
        depth = (2. * znear * zfar) / depth
        if self.noise:
          depth += random_seed.normal(0, 0.003, depth_image_size)

        # Get segmentation image.
        segm = np.uint8(segm).reshape(depth_image_size)

        # Start thread to stream RGB-D images
        self.color_img = None
        self.depth_img = None
        self.segm_mask = None
        self.get_data()
        stream_thread = threading.Thread(target=self.stream_data)
        stream_thread.daemon = True
        stream_thread.start()

    def stream_data(self):
        self.get_data()
        time.sleep(1 / self.fps)

    def get_data(self, noise=True):

        data = p.getCameraImage(self.image_size[1], self.image_size[0],
                                self.view_matrix, self.projection_matrix,
                                shadow=1, flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
                                renderer=p.ER_BULLET_HARDWARE_OPENGL)

        # Get color image
        rgb_pixels = np.array(data[2]).reshape((self.image_size[0], self.image_size[1], 4))
        self.color_img = rgb_pixels[:, :, :3]  # remove alpha channel

        # Get depth image (from z-buffer)
        z_buffer = np.array(data[3]).reshape((self.image_size[0], self.image_size[1]))
        depth_img = (2.0 * self.zrange[0] * self.zrange[1]) / (self.zrange[1] + self.zrange[0] - (2.0 * z_buffer - 1.0) * (self.zrange[1] - self.zrange[0]))
        if noise:
            depth_img += np.random.normal(loc=0, scale=0.005, size=(self.image_size[0], self.image_size[1]))
        self.depth_img = depth_img

        self.segm_mask = None
