from abc import ABC, abstractmethod
import cv2

import external.pykinect_azure as pykinect
from external.pykinect_azure.utils.postProcessing import smooth_depth_image

import numpy as np
import pyrealsense2 as rs

import numpy as np
import pyrealsense2 as rs
from collections import OrderedDict


class RealWorldBaseCamera(ABC):
    @abstractmethod
    def startStreaming(self):
        pass
    
    @abstractmethod
    def getRGB(self):
        """
        [RCareWorld API]
        """
        pass
    
    @abstractmethod
    def getDepthEXR(self):
        """
        [RCareWorld API]
        """
        pass
    
    @abstractmethod
    def getCameraInfo(self):
        """
        [RCareWorld API]
        """
        pass

class AzureKinect(RealWorldBaseCamera):
    def __init__(self,):
        pykinect.initialize_libraries()
        # Modify camera configuration
        self.device_config = pykinect.default_configuration

    def startStreaming(self):
        self.device = pykinect.start_device(config=self.device_config)
    
        
    def getCameraInfo(self):
        return pykinect.calibration.get_all_parameters()
    
    def getRGB(self):
        """
        [RCareWorld API]
        get RGB
        """
        capture = self.device.update()
        while True:
            ret, color_image = capture.get_color_image()
            if ret:
                return color_image
    
    def getDepthEXR(self):
        """
        [RCareWorld API]
        get depth that is aligned with color image
        """
        capture = self.device.update()
        while True:
            ret, depth_image = capture.get_colored_depth_image()
            if ret:
                return depth_image
            else:
                continue
    
    def getDepth(self):
        """
        get depth that is aligned with color image
        """
        capture = self.device.update()
        while True:
            ret, depth_image = capture.get_colored_depth_image()
            if ret:
                return depth_image
            else:
                continue
    
    def getPointCloud(self):
        """
        get point cloud that is aligned with color image
        """
        while True:
            # Get capture
            capture = self.device.update()
            # Get the 3D point cloud
            ret_point, points = capture.get_transformed_pointcloud()

            # Get the color image in the depth camera axis
            ret_color, color_image = capture.get_color_image()

            if not ret_color or not ret_point:
                continue
            else:
                return points, color_image
    
    def getIRImage(self):
        while True:
            # Get capture
            capture = self.device.update()
            ret_ir, ir_image = capture.get_ir_image()

            if not ret_ir:
                continue
            else:
                return ir_image
    
    def getSmoothedDepth(self, maximum_hole_size=10):
        depth = self.getDepth()
        smoothed_depth = smooth_depth_image(depth)
        return smoothed_depth

class RealSense(RealWorldBaseCamera):
    def __init__(self, height=480, width=640, fps=30, warm_start=60):
        self.height = height
        self.width = width
        self.fps = fps
        self.warm_start = warm_start

        # Identify devices
        self.device_ls = []
        for c in rs.context().query_devices():
            self.device_ls.append(c.get_info(rs.camera_info(1)))
    
    def getCameraInfo(self):
        pass

    def startStreaming(self):
        # Start stream
        print(f"Connecting to RealSense cameras ({len(self.device_ls)} found) ...")
        self.pipes = []
        self.profiles = OrderedDict()
        for i, device_id in enumerate(self.device_ls):
            pipe = rs.pipeline()
            config = rs.config()

            config.enable_device(device_id)
            config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            config.enable_stream(
                rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps
            )

            self.pipes.append(pipe)
            self.profiles[device_id]=pipe.start(config)

            print(f"Connected to camera {i+1} ({device_id}).")

        self.align = rs.align(rs.stream.color)
        # Warm start camera (realsense automatically adjusts brightness during initial frames)
        for _ in range(self.warm_start):
            self._get_frames()

    def _get_frames(self):
        framesets = [pipe.wait_for_frames() for pipe in self.pipes]
        return [self.align.process(frameset) for frameset in framesets]

    def getIntrinsics(self):
        intrinsics_ls = []
        for profile in self.profiles.values():
            stream = profile.get_streams()[1]
            intrinsics = stream.as_video_stream_profile().get_intrinsics()

            intrinsics_ls.append(intrinsics)

        return intrinsics_ls

    def getIntrinsicsDepth(self):
        intrinsics_ls = OrderedDict()
        for device_id, profile in self.profiles.items():
            stream = profile.get_streams()[1]
            intrinsics = stream.as_video_stream_profile().get_intrinsics()
            param_dict = dict([(p, getattr(intrinsics, p)) for p in dir(intrinsics) if not p.startswith('__')])
            param_dict['model'] = param_dict['model'].name

            intrinsics_ls[device_id] = param_dict

        return intrinsics_ls
    
    def getNumCameras(self):
        return len(self.device_ls)

    def get_rgbd(self):
        """Returns a numpy array of [n_cams, height, width, RGBD]"""
        framesets = self._get_frames()
        num_cams = self.get_num_cameras()

        rgbd = np.empty([num_cams, self.height, self.width, 4], dtype=np.uint16)

        for i, frameset in enumerate(framesets):
            color_frame = frameset.get_color_frame()
            rgbd[i, :, :, :3] = np.asanyarray(color_frame.get_data())

            depth_frame = frameset.get_depth_frame()
            rgbd[i, :, :, 3] = np.asanyarray(depth_frame.get_data())

        return rgbd

    def getRGB(self):
        """Returns a numpy array of [n_cams, height, width, RGBD]"""
        framesets = self._get_frames()
        num_cams = self.get_num_cameras()


        for i, frameset in enumerate(framesets):
            color_frame = frameset.get_color_frame()
            rgb= np.asanyarray(color_frame.get_data())

        return rgb
        
    
    def getDepthEXR(self):
        framesets = self._get_frames()
        num_cams = self.get_num_cameras()

        for i, frameset in enumerate(framesets):
            depth_frame = frameset.get_depth_frame()
            depth = np.asanyarray(depth_frame.get_data())
        
        return depth
    
    def getDepth(self):
        depth = self.getDepthEXR()
        return depth

    

            

    