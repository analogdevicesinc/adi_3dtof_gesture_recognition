#!/usr/bin/env python
""" File Input mode
"""

import time
import struct
import adi_3dtof_gesture_recognition.pcd_utils as pcd_utils
import numpy as np
import rospy


class InputFile:
    """ File Input class """

    __slot__ = [
        "_topic_prefix",
        "_file_name",
        "_is_file_open",
        "_file",
        "_frame_width",
        "_frame_height",
        "_camera_info_k",
        "_camera_info_d",
        "_range_to_3d_lut",
        "_ir_image",
        "_depth_image",
        "_xyz_points",
        "_time_stamp",
        "_bytes_per_pixel",
        "_header_version",
        "_first_frame_pos",
        "_frame_pitch",
        "_device_timestamp",
        "_total_frames",
        "_num_of_frames_read",
        "_is_camera_info_set",
    ]

    def __init__(self):
        """constructor """
        self._topic_prefix = ""
        self._file_name = ""
        self._is_file_open = False
        self._file = None
        self._total_frames = 0
        self._frame_width = 0
        self._frame_height = 0
        self._camera_info_k = None
        self._camera_info_d = None
        self._range_to_3d_lut = None
        self._ir_image = None
        self._depth_image = None
        self._xyz_points = None
        self._time_stamp = None
        self._bytes_per_pixel = 0
        self._header_version = 0
        self._first_frame_pos = 0
        self._frame_pitch = 0
        self._device_timestamp = 0
        self._num_of_frames_read = 0
        self._is_camera_info_set = False

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

    def open_sensor(self, file_name, mode=None, config=None):
        """Opens the sensor

        Args:
            file_name (str): Input filename 
        """
        self._file_name = file_name
        try:
            self._file = open(self._file_name, "rb")
            self._is_file_open = True
        except IOError as e:
            # Handle the error
            print(f"Could not open the file: {e}")

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

    def configure_sensor(self):
        """Configures the sensor"""
        if not self._is_file_open:
            raise IOError("Input file is not opened.")
        # 36 bytes header
        self._total_frames = struct.unpack("<I", self._file.read(4))[0]
        self._frame_width = struct.unpack("<I", self._file.read(4))[0]
        self._frame_height = struct.unpack("<I", self._file.read(4))[0]
        self._bytes_per_pixel = struct.unpack("<I", self._file.read(4))[0]
        self._header_version = struct.unpack("<I", self._file.read(4))[0]
        self._first_frame_pos = struct.unpack("<I", self._file.read(4))[0]
        self._frame_pitch = struct.unpack("<I", self._file.read(4))[0]
        self._device_timestamp = struct.unpack("<Q", self._file.read(8))[0]

        # Read camera info the size = "first_frame_pos - 36"
        self._camera_info_k = struct.unpack("<9d", self._file.read(9 * 8))
        size_of_d = struct.unpack("<I", self._file.read(4))[0]
        fmt = f"<{size_of_d}d"
        self._camera_info_d = struct.unpack(fmt, self._file.read(size_of_d * 8))
        if self._header_version == 1:
            self._camera_info_d = self._camera_info_d[8:]

        self._range_to_3d_lut = pcd_utils.generate_range_to_3d_lut(
            np.array(self._camera_info_k).reshape([3, 3]).astype(np.float64),
            np.array(self._camera_info_d).reshape([1, 8]).astype(np.float64),
            self._frame_width,
            self._frame_height,
        )
        self._is_camera_info_set = True

        # Seek to the first frame position
        self._file.seek(self._first_frame_pos, 0)

    def get_frame_timestamp(self, frame_timestamp):
        """ Get the current timestamp

        Args:
            frame_timestamp (int64): Timestamp

        Returns:
            rospy.Time : Timestamp
        """
        if frame_timestamp > 0:
            seconds = frame_timestamp // 1000000000
            nanoseconds = frame_timestamp % 1000000000
            timestamp = rospy.Time(seconds, nanoseconds)
        else:
            timestamp = rospy.Time.now()

        return timestamp

    def get_frame(self):
        """Get the current frame

        Returns:
            dict: Frame info
        """
        if self._is_camera_info_set:
            # Check for End of file
            if self._num_of_frames_read >= self._total_frames:
                return

            # Time stamp - 8 bytes
            self._time_stamp = struct.unpack("<Q", self._file.read(8))[0]
            self._time_stamp = self.get_frame_timestamp(self._time_stamp)

            # Depth frame
            num_samples_in_frame = self._frame_width * self._frame_height
            fmt = f"<{num_samples_in_frame}H"
            self._depth_image = (
                np.array(
                    struct.unpack(
                        fmt,
                        self._file.read(num_samples_in_frame * self._bytes_per_pixel),
                    )
                )
                .reshape([self._frame_width, self._frame_height])
                .astype(np.uint16)
            )
            # IR image
            ir_time_stamp = struct.unpack("<Q", self._file.read(8))[0]
            self._ir_image = (
                np.array(
                    struct.unpack(
                        fmt,
                        self._file.read(num_samples_in_frame * self._bytes_per_pixel),
                    )
                )
                .reshape([self._frame_width, self._frame_height])
                .astype(np.uint16)
            )

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

            # Adding some sleep to simulate 2 frame/sec processing.
            time.sleep(0.5)

            return input_frame_info

    def set_AB_invalidation_threshold(self, threshold):
        """Dummy function
        """
        pass

    def set_confidence_threshold(self, threshold):
        """Dummy function
        """
        pass
