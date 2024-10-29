#!/usr/bin/env python
"""ROS topic as Input
"""

import time
import collections
import rospy
import message_filters
from sensor_msgs.msg import CameraInfo, Image
import adi_3dtof_gesture_recognition.pcd_utils as pcd_utils
import numpy as np
from cv_bridge import CvBridge


class InputROSTopic:
    """ROS Topic input Class """

    __slot__ = [
        "_queue_buffer_size",
        "_topic_prefix",
        "_depth_image_topic_name",
        "_ir_image_topic_name",
        "_camera_info_topic_name",
        "_is_camera_info_set",
        "_frame_width",
        "_frame_height",
        "_depth_image_subscriber",
        "_ir_image_subscriber",
        "_camera_info_subscriber",
        "_message_filter_sync_depth_ir",
        "_camera_info_k",
        "_camera_info_d",
        "_range_to_3d_lut",
        "_ir_image",
        "_depth_image",
        "_xyz_points",
        "_depth_time_stamp",
        "_num_of_frames_read",
        "_input_frame_queue",
    ]

    def __init__(self):
        """constructor"""
        self._queue_buffer_size = 1
        self._topic_prefix = ""
        self._depth_image_topic_name = ""
        self._ir_image_topic_name = ""
        self._camera_info_topic_name = ""
        self._is_camera_info_set = False
        self._frame_width = 0
        self._frame_height = 0
        self._depth_image_subscriber = None
        self._ir_image_subscriber = None
        self._camera_info_subscriber = None
        self._message_filter_sync_depth_ir = None
        self._camera_info_k = None
        self._camera_info_d = None
        self._range_to_3d_lut = None
        self._ir_image = None
        self._depth_image = None
        self._xyz_points = None
        self._depth_time_stamp = None
        self._num_of_frames_read = 0
        self._input_frame_queue = collections.deque(maxlen=self._queue_buffer_size)

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

    def open_sensor(self, topic_prefix, mode=None, config=None):
        """Opens the sensor

        Args:
            topic_prefix (str):  ROS topic prefix
        """

        self._topic_prefix = topic_prefix
        self._depth_image_topic_name = f"{topic_prefix}/depth_image"
        self._ir_image_topic_name = f"{topic_prefix}/ir_image"
        self._camera_info_topic_name = f"{topic_prefix}/camera_info"

        # Subscribe to the topics
        self._depth_image_subscriber = message_filters.Subscriber(
            self._depth_image_topic_name, Image
        )
        self._ir_image_subscriber = message_filters.Subscriber(
            self._ir_image_topic_name, Image
        )
        self._camera_info_subscriber = rospy.Subscriber(
            self._camera_info_topic_name, CameraInfo, self.camera_info_callback
        )

        self._message_filter_sync_depth_ir = message_filters.TimeSynchronizer(
            [self._depth_image_subscriber, self._ir_image_subscriber],
            self._queue_buffer_size,
        )
        self._message_filter_sync_depth_ir.registerCallback(
            self.depth_ir_time_sync_combined_callback
        )

    def configure_sensor(self):
        """Configures the sensor
        """
        # Wait for the camera_info to be received at least once before proceeding
        while not self._is_camera_info_set and not rospy.is_shutdown():
            print("Waiting for the camera_info topic..")
            time.sleep(1)

    def camera_info_callback(self, msg):
        """Camera info callback

        Args:
            msg (CameraInfo): Camera info message
        """
        if not self._is_camera_info_set:
            self._camera_info_k = msg.K
            self._camera_info_d = msg.D
            self._frame_width = msg.width
            self._frame_height = msg.height
            self._range_to_3d_lut = pcd_utils.generate_range_to_3d_lut(
                np.array(msg.K).reshape([3, 3]).astype(np.float64),
                np.array(msg.D).reshape([1, 8]).astype(np.float64),
                self._frame_width,
                self._frame_height,
            )
            self._is_camera_info_set = True

    def depth_ir_time_sync_combined_callback(self, msg_depth, msg_IR):
        """Synced Depth and IR callback

        Args:
            msg_depth (Image): Depth image message
            msg_IR (Image): IR image message
        """
        if self._is_camera_info_set:
            # convert image message to cv
            self._depth_image = CvBridge().imgmsg_to_cv2(msg_depth)
            self._depth_time_stamp = msg_depth.header.stamp
            self._ir_image = CvBridge().imgmsg_to_cv2(msg_IR)

            # Compute point cloud
            self._xyz_points = pcd_utils.compute_point_cloud(
                self._depth_image.astype(np.uint16), self._range_to_3d_lut
            )

            # Add the frame info to the Queue
            input_frame_info = {
                "depth_image": self._depth_image,
                "ir_image": self._ir_image,
                "xyz_points": self._xyz_points,
                "time_stamp": self._depth_time_stamp,
                "frame_num": self._num_of_frames_read,
                "width": self._frame_width,
                "height": self._frame_height,
            }

            self._num_of_frames_read += 1
            # Note : The old data gets discarded automatically, if the queue is full
            self._input_frame_queue.append(input_frame_info)

    def get_frame(self):
        """get the current frame"""
        if len(self._input_frame_queue) != 0:
            return self._input_frame_queue.popleft()

    def set_AB_invalidation_threshold(self, threshold):
        """Dummy function"""
        pass

    def set_confidence_threshold(self, threshold):
        """Dummy function"""
        pass
