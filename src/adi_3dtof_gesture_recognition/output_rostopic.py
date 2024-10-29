#!/usr/bin/env python
"""ROS Topic+file output mode 
"""

import os
import csv
import math
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int32
from std_msgs.msg import Header


class OutputROSTopic:
    """Class defintion for the ROSTopic output mode"""

    __slots__ = [
        "_queue_size",
        "_time_stamp",
        "_topic_prefix",
        "_camera_info_topic_name",
        "_depth_image_topic_name",
        "_ir_image_topic_name",
        "_output_image_topic_name",
        "_output_gesture_topic_name",
        "_frame_id",
        "_frame_width",
        "_frame_height",
        "_camera_info_k",
        "_camera_info_d",
        "_depth_image_publisher",
        "_ir_image_publisher",
        "_output_image_publisher",
        "_output_gesture_publisher",
        "_cam_info_publisher",
        "_ena_fileout",
        "_type",
        "_video_writer",
        "_csv_file",
        "_csv_writer",
        "_ena_compressed_output_image",
        "_ena_output_image_flip",
    ]

    def __init__(self, output_type):
        """Constructor

        Args:
            output_type (str): Output type
        """
        self._queue_size = 10
        self._time_stamp = None
        self._topic_prefix = ""
        self._camera_info_topic_name = ""
        self._depth_image_topic_name = ""
        self._ir_image_topic_name = ""
        self._output_image_topic_name = ""
        self._output_gesture_topic_name = ""
        self._frame_id = None
        self._frame_width = 0
        self._frame_height = 0
        self._camera_info_k = None
        self._camera_info_d = None
        self._output_image_publisher = None
        self._output_gesture_publisher = None
        self._depth_image_publisher = None
        self._ir_image_publisher = None
        self._cam_info_publisher = None
        self._ena_fileout = False
        self._type = output_type
        self._video_writer = None
        self._csv_file = None
        self._csv_writer = None
        self._ena_compressed_output_image = False
        self._ena_output_image_flip = False
        if self._type == "rostopic_file":
            self._ena_fileout = True

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

    def open_sensor(
        self,
        frame_width,
        frame_height,
        topic_prefix,
        filename,
        enable_output_image_compression,
        enable_output_image_flip,
    ):
        """Opens the sensor

        Args:
            topic_prefix (str): Output topic prefix
        """
        self._frame_width = frame_width
        self._frame_height = frame_height
        self._topic_prefix = topic_prefix
        self._camera_info_topic_name = f"{topic_prefix}/camera_info"
        self._depth_image_topic_name = f"{topic_prefix}/depth_image"
        self._ir_image_topic_name = f"{topic_prefix}/ir_image"
        self._output_image_topic_name = f"{topic_prefix}/output_image"
        self._output_gesture_topic_name = f"{topic_prefix}/output_gesture"
        self._ena_compressed_output_image = enable_output_image_compression
        self._ena_output_image_flip = enable_output_image_flip

        # Create the Publisher
        self._cam_info_publisher = rospy.Publisher(
            self._camera_info_topic_name, CameraInfo, queue_size=self._queue_size
        )
        self._depth_image_publisher = rospy.Publisher(
            self._depth_image_topic_name, Image, queue_size=self._queue_size
        )
        self._ir_image_publisher = rospy.Publisher(
            self._ir_image_topic_name, Image, queue_size=self._queue_size
        )
        if self._ena_compressed_output_image:
            self._output_image_publisher = rospy.Publisher(
                f"{self._output_image_topic_name}/compressed",
                CompressedImage,
                queue_size=self._queue_size,
            )
        else:
            self._output_image_publisher = rospy.Publisher(
                self._output_image_topic_name, Image, queue_size=self._queue_size
            )

        self._output_gesture_publisher = rospy.Publisher(
            self._output_gesture_topic_name, Int32, queue_size=self._queue_size
        )

        if self._ena_fileout:
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self._video_writer = cv2.VideoWriter(
                filename, fourcc, 10, (self._frame_width * 2, self._frame_height)
            )
            # Create a csv file, replace the extension with csv
            base_name, _ = os.path.splitext(filename)
            csv_filename = base_name + ".csv"
            self._csv_file = open(csv_filename, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            output_row = ["Frame Num", "Gesture Label"]
            self._csv_writer.writerow(output_row)

    def configure_sensor(self):
        """Dummy function"""
        pass

    def fill_and_publish_camera_info(self, header):
        """Populate and publish camera info

        Args:
            header (CameraInfo.Header): Camerainfo header
        """
        cam_info_msg = CameraInfo()

        # Header
        cam_info_msg.header = header

        cam_info_msg.width = self._frame_width
        cam_info_msg.height = self._frame_height

        cam_info_msg.K = self._camera_info_k
        cam_info_msg.P = 12 * [0.0]
        cam_info_msg.P[0] = cam_info_msg.K[0]
        cam_info_msg.P[2] = cam_info_msg.K[2]
        cam_info_msg.P[3] = 0.0  # Transalation x
        cam_info_msg.P[5] = cam_info_msg.K[4]
        cam_info_msg.P[6] = cam_info_msg.K[5]
        cam_info_msg.P[7] = 0.0  # Transalation y
        cam_info_msg.P[10] = 1.0
        cam_info_msg.P[11] = 0.0  # Transalation z

        cam_info_msg.distortion_model = "rational_polynomial"
        cam_info_msg.D = self._camera_info_d
        cam_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        self._cam_info_publisher.publish(cam_info_msg)

    def publish_frame(self, output_frame_info, output_args):
        """ Publish the output frame

        Args:
            output_frame_info (dict): Output frame info
        """

        if output_frame_info is None:
            return

        self._camera_info_k = output_args["camera_info_k"]
        self._camera_info_d = output_args["camera_info_d"]
        self._frame_id = output_args["optical_camera_link"]

        frame_num = output_frame_info["frame_num"]
        time_stamp = output_frame_info["time_stamp"]
        ir_image = output_frame_info["ir_image"]
        depth_image = output_frame_info["depth_image"]
        output_gesture = output_frame_info["output_gesture"]
        hand_segmentation_debug_info = output_frame_info["hand_segmentation_debug_info"]

        image_header = Header()
        image_header.seq = frame_num
        image_header.stamp = time_stamp
        image_header.frame_id = self._frame_id

        # Publish camera info
        self.fill_and_publish_camera_info(image_header)

        # Publish gesture output
        self._output_gesture_publisher.publish(output_gesture)

        # Depth Image
        # Adding eps to avoid log 0
        depth_image_8bit = (
            (255 * np.log(depth_image + np.finfo(float).eps)) / math.log(8192)
        ).astype(np.uint8)
        depth_image_rgb = cv2.cvtColor(depth_image_8bit, cv2.COLOR_GRAY2BGR)

        # Convert IR image to 8-bit, also perform Gamma Correction
        # Adding eps to avoid log 0
        ir_image_8bit = (
            (255 * np.log(ir_image + np.finfo(float).eps)) / math.log(2048)
        ).astype(np.uint8)
        ir_image_rgb = cv2.cvtColor(ir_image_8bit, cv2.COLOR_GRAY2BGR)

        # Publish Depth and IR
        bridge = CvBridge()
        depth_image_ros_msg = bridge.cv2_to_imgmsg(
            depth_image, encoding="mono16", header=image_header
        )
        self._depth_image_publisher.publish(depth_image_ros_msg)

        ir_image_ros_msg = bridge.cv2_to_imgmsg(
            ir_image, encoding="mono16", header=image_header
        )
        self._ir_image_publisher.publish(ir_image_ros_msg)

        # Publish output image
        # Draw the ROI in green box(for both IR and Depth)
        if output_frame_info["processing_roi"] is not None:
            color_bb = (0, 255, 0)
            for rgb_image in [ir_image_rgb, depth_image_rgb]:
                # Draw the refernece bounding box
                cv2.rectangle(
                    rgb_image,
                    output_frame_info["processing_roi"][0],
                    output_frame_info["processing_roi"][1],
                    color_bb,
                    3,
                )

        if hand_segmentation_debug_info is not None:
            stage2_mask = hand_segmentation_debug_info["stage2_mask"].reshape(
                [self._frame_height, self._frame_width]
            )

            blue_img = np.zeros(depth_image_rgb.shape, depth_image_rgb.dtype)
            blue_img[:, :] = (0, 0, 0)
            blue_img[stage2_mask] = (255, 0, 0)
            alpha = 0.3
            depth_image_rgb = cv2.addWeighted(
                blue_img, alpha, depth_image_rgb, 1 - alpha, 0
            )

        if self._ena_output_image_flip:
            # Horizontal Flip the image
            depth_image_rgb = cv2.flip(depth_image_rgb, 1)

        # Draw gesture type on the image
        gesture_text = f"[{frame_num}] Gesture:[{output_gesture}]"
        cv2.putText(
            depth_image_rgb,
            gesture_text,
            (40, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
        )

        if self._ena_compressed_output_image:
            # Compress image to JPEG format
            compressed_img = cv2.imencode(".jpg", depth_image_rgb)[1]
            # Prepare CompressedImage message
            output_image_ros_msg = CompressedImage()
            output_image_ros_msg.header.stamp = rospy.Time.now()
            output_image_ros_msg.format = "jpeg"
            output_image_ros_msg.data = compressed_img.tostring()
        else:
            output_image_ros_msg = bridge.cv2_to_imgmsg(
                depth_image_rgb, encoding="bgr8", header=image_header
            )

        self._output_image_publisher.publish(output_image_ros_msg)

        concat_image = cv2.hconcat([ir_image_rgb, depth_image_rgb])

        if self._ena_fileout:
            self._video_writer.write(concat_image)
            if output_gesture != None:
                output_row = [frame_num, output_gesture]
                self._csv_writer.writerow(output_row)

    def close_sensor(self):
        """Closes the sensor"""
        if self._ena_fileout:
            self._video_writer.release()
            self._csv_file.close()
