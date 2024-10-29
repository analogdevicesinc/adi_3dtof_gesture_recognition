#!/usr/bin/env python
""" ADTF31xx Input mode
"""
import rospy
import adi_3dtof_gesture_recognition.pcd_utils as pcd_utils
import aditofpython as tof
import numpy as np
import os


modemapping = {
    "lr-native": {"width": 1024, "height": 1024},
    "lr-qnative": {"width": 512, "height": 512},
    "sr-native": {"width": 1024, "height": 1024},
    "sr-qnative": {"width": 512, "height": 512},
}


class InputADTF31xx:
    """ ADTF31xx Input class """

    __slot__ = [
        "_frame_width",
        "_frame_height",
        "_camera",
        "_sensor",
        "_camera_info_k",
        "_camera_info_d",
        "_range_to_3d_lut",
        "_ir_image",
        "_depth_image",
        "_xyz_points",
        "_time_stamp",
        "_num_of_frames_read",
        "_processing_scale",
    ]

    def __init__(self):
        """ Constructor
        """

        self._frame_width = None
        self._frame_height = None
        self._camera = None
        self._sensor = None
        self._camera_info_k = None
        self._camera_info_d = None
        self._range_to_3d_lut = None
        self._ir_image = None
        self._depth_image = None
        self._xyz_points = None
        self._time_stamp = None
        self._num_of_frames_read = 0
        self._processing_scale = 1

    @property
    def frame_width(self):
        """The getter method for frame_width

        Returns:
            int: frame_width 
        """
        return self._frame_width

    @frame_width.setter
    def frame_width(self, frame_width):
        """The setter method for frame_width

        Args:
            frame_width (int): frame_width
        """
        self._frame_width = frame_width

    @property
    def frame_height(self):
        """The getter method for frame_height

        Returns:
            frame_height (int): frame_height
        """
        return self._frame_height

    @frame_height.setter
    def frame_height(self, frame_height):
        """The setter method for frame_height

        Args:
            frame_height (int): frame height
        """
        self._frame_height = frame_height

    @property
    def camera_info_k(self):
        """The getter method for camera_info_k 

        Returns:
            list : Camera info
        """
        return self._camera_info_k

    @property
    def camera_info_d(self):
        """The getter method for camera_info_d

        Returns:
            list : Distortion coefs
        """
        return self._camera_info_d

    def callbackFunction(callbackStatus):
        """ Callback function for the ADSD3500

        Args:
            callbackStatus (int): Status flag
        """
        print(
            "Running the python callback for which the status of ADSD3500 has been forwarded. ADSD3500 status = ",
            callbackStatus,
        )

    def open_sensor(self, ip, mode=None, config=None):
        """Opens the sensor
        Args:
            ip (str): IP adress of the sensor
            mode (str, optional): Operating mode. Defaults to None.
            config (str, optional): Config filename. Defaults to None.
        """
        # Checks on input
        if mode not in modemapping:
            print(f"Error: Unknown mode - {mode}")
        if os.path.exists(config) == False:
            print(f"Error: Config file cannot be found - {config}")

        self._mode = mode
        self._frame_width = modemapping[mode]["width"]
        self._frame_height = modemapping[mode]["height"]
        self._processing_scale = 1024 / self._frame_width
        print(
            "ADI ToF SDK version: ",
            tof.getApiVersion(),
            " | branch: ",
            tof.getBranchVersion(),
            " | commit: ",
            tof.getCommitVersion(),
        )

        cameras = []
        system = tof.System()
        status = system.getCameraList(cameras, ip)
        # Raise exception and return
        print("system.getCameraList()", status)
        self._camera = cameras[0]
        # create callback and register it to the interrupt routine

        self._sensor = self._camera.getSensor()
        status = self._sensor.adsd3500_register_interrupt_callback(
            self.callbackFunction
        )

        status = self._camera.initialize(config)
        # Raise exception and return
        print("self._camera.initialize()", status)

    def configure_sensor(self):
        """Configures the sensor
        """

        types = []
        status = self._camera.getAvailableFrameTypes(types)
        print("self._camera.getAvailableFrameTypes()", status)
        print(types)

        camDetails = tof.CameraDetails()
        status = self._camera.getDetails(camDetails)
        print("self._camera.getDetails()", status)
        print(
            "self._camera details:",
            "id:",
            camDetails.cameraId,
            "connection:",
            camDetails.connection,
        )

        status = self._camera.setFrameType(self._mode)
        print("self._camera.setFrameType()", status)

        status = self._camera.start()
        print("camera1.start()", status)

        # Get camera intrinsics
        cam_intr = camDetails.intrinsics

        # Camera Matrix
        # fx, fy : Camera focal lengths (pixels)
        # cx, cy : Camera principal points or optical centers (pixels)
        self._camera_info_k = [
            cam_intr.fx / self._processing_scale,
            0.0,
            cam_intr.cx / self._processing_scale,
            0.0,
            cam_intr.fy / self._processing_scale,
            cam_intr.cy / self._processing_scale,
            0.0,
            0.0,
            1.0,
        ]
        print(f"Frame Width: {self._frame_width}, Frame Height: {self._frame_height}")
        print("Camera Matrix K: ", self._camera_info_k)

        # Distortion Matrix
        # k1, k2, k3, k4, k5, k6 : Camera radial distortion cofficients
        # p1, p2 : Camera tangential distortion coefficients
        self._camera_info_d = [
            cam_intr.k1,
            cam_intr.k2,
            cam_intr.p1,
            cam_intr.p2,
            cam_intr.k3,
            cam_intr.k4,
            cam_intr.k5,
            cam_intr.k6,
        ]
        print("Distortion Matrix D: ", self._camera_info_d)
        self._range_to_3d_lut = pcd_utils.generate_range_to_3d_lut(
            np.array(self._camera_info_k).reshape([3, 3]).astype(np.float64),
            np.array(self._camera_info_d).reshape([1, 8]).astype(np.float64),
            self._frame_width,
            self._frame_height,
        )

        frame = tof.Frame()
        status = self._camera.requestFrame(frame)
        print("camera1.requestFrame()", status)

        # Depth frame
        depth = np.array(frame.getData("depth"), copy=False)

        # IR image
        ir = np.array(frame.getData("ab"), copy=False)

    def get_frame(self):
        """Get the current frame

        Returns:
            dict: Frame info
        """

        # Time stamp
        self._time_stamp = rospy.get_rostime()

        frame = tof.Frame()
        status = self._camera.requestFrame(frame)
        self._num_of_frames_read += 1

        # Depth frame
        self._depth_image = np.array(frame.getData("depth"), dtype="uint16", copy=False)

        # IR image
        self._ir_image = np.array(frame.getData("ab"), dtype="uint16", copy=False)

        # Compute point cloud
        self._xyz_points = pcd_utils.compute_point_cloud(
            self._depth_image.astype(np.uint16), self._range_to_3d_lut
        )

        # Add the frame info to the Queue
        input_frame_info = {
            "depth_image": self._depth_image.copy(),
            "ir_image": self._ir_image.copy(),
            "xyz_points": self._xyz_points.copy(),
            "time_stamp": self._time_stamp,
            "frame_num": self._num_of_frames_read,
            "width": self._frame_width,
            "height": self._frame_height,
        }
        self._num_of_frames_read += 1
        return input_frame_info

    def close_sensor(self):
        """Close the sensor
        """
        status = self._camera.stop()
        print("camera1.stop()", status)
        # Unregister callback
        status = self._sensor.adsd3500_unregister_interrupt_callback(
            self.callbackFunction
        )

    def set_AB_invalidation_threshold(self, threshold):
        """Sets AB threshold

        Args:
            threshold (float): ab_threshold
        """
        status, currentIniParams = self._camera.getIniParams()
        currentIniParams["ab_thresh_min"] = threshold
        self._camera.setIniParams(currentIniParams)

    def set_confidence_threshold(self, threshold):
        """Sets the confidence threshold

        Args:
            threshold (float): confidence threshold
        """
        status, currentIniParams = self._camera.getIniParams()
        currentIniParams["conf_thresh"] = threshold
        self._camera.setIniParams(currentIniParams)
