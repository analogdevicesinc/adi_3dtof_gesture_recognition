#!/usr/bin/env python
""" An example script to subscribe to gesture recognition output
"""
import math
import rospy
import numpy as np
import cv2
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header, Int32
import message_filters
import adi_3dtof_gesture_recognition.pcd_utils as pcd_utils
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class ADI3DToFGestureRecognitionHost:
    """ Gesture recognition Host class """

    def __init__(self, topic_prefix, enable_point_cloud):
        """ Constructor

        Args:
            topic_prefix (string): Topic prefix
            enable_point_cloud (bool): Enable point cloud generation and publish
        """

        self._queue_buffer_size = 10
        self._frame_width = 0
        self._frame_height = 0
        self._frame_id = None
        self._is_camera_info_set = False
        self._depth_sync_flag = False
        self._camera_info_k = None
        self._camera_info_d = None
        self._range_to_3d_lut = None
        self._ir_image = None
        self._depth_image = None
        self._xyz_points = None
        self._gesture = -1
        self._depth_image_rgb = None
        self._frame_num = 0
        self._enable_point_cloud = enable_point_cloud

        # Subscribers
        self._topic_prefix = "/" + topic_prefix
        self._camera_info_topic_name = f"{self._topic_prefix}/camera_info"
        self._depth_image_topic_name = f"{self._topic_prefix}/depth_image"
        self._ir_image_topic_name = f"{self._topic_prefix}/ir_image"
        self._gesture_topic_name = f"{self._topic_prefix}/output_gesture"

        self._camera_info_subscriber = rospy.Subscriber(
            self._camera_info_topic_name, CameraInfo, self.camera_info_callback
        )
        self._depth_image_subscriber = message_filters.Subscriber(
            self._depth_image_topic_name, Image
        )
        self._ir_image_subscriber = message_filters.Subscriber(
            self._ir_image_topic_name, Image
        )

        self._gesture_subscriber = rospy.Subscriber(
            self._gesture_topic_name, Int32, self.gesture_callback
        )
        # Synchronizer
        self._message_filter_sync_depth_ir = message_filters.TimeSynchronizer(
            [self._depth_image_subscriber, self._ir_image_subscriber],
            self._queue_buffer_size,
        )
        self._message_filter_sync_depth_ir.registerCallback(
            self.depth_ir_synced_callback
        )

        # Publishers
        self._output_image_topic_name = f"/host/output_image"
        # Header
        self._header = Header()
        self._output_image_publisher = rospy.Publisher(
            self._output_image_topic_name, Image, queue_size=self._queue_buffer_size
        )

        if self._enable_point_cloud:
            self._ptcld_fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            self._output_ptcld_topic_name = f"/host/pointcloud"
            self._output_ptcld_publisher = rospy.Publisher(
                self._output_ptcld_topic_name,
                PointCloud2,
                queue_size=self._queue_buffer_size,
            )

        self.get_output()

    def camera_info_callback(self, msg):
        """Camera info callback function

        Args:
            msg (CameraInfo): Camera info message 
        """
        if not self._is_camera_info_set:
            self._camera_info_k = msg.K
            self._camera_info_d = msg.D
            self._frame_width = msg.width
            self._frame_height = msg.height
            self._frame_id = msg.header.frame_id
            if self._enable_point_cloud:
                self._range_to_3d_lut = pcd_utils.generate_range_to_3d_lut(
                    np.array(msg.K).reshape([3, 3]).astype(np.float64),
                    np.array(msg.D).reshape([1, 8]).astype(np.float64),
                    self._frame_width,
                    self._frame_height,
                )
            self._is_camera_info_set = True

    def gesture_callback(self, msg_gesture):
        """Gesture Label callback

        Args:
            msg_gesture (Int32): Gesture Label
        """
        self._gesture = msg_gesture.data

    def depth_ir_synced_callback(self, msg_depth, msg_IR):
        """Synced Depth and IR callback

        Args:
            msg_depth (Image): Depth Image message
            msg_IR (Image): IR Image message
        """
        self._depth_sync_flag = False
        if self._is_camera_info_set:
            # convert image message to cv
            self._depth_image = CvBridge().imgmsg_to_cv2(msg_depth)
            self._ir_image = CvBridge().imgmsg_to_cv2(msg_IR)

            if self._enable_point_cloud:
                # Compute point cloud
                self._xyz_points = pcd_utils.compute_point_cloud(
                    self._depth_image.astype(np.uint16), self._range_to_3d_lut
                )
            self._depth_sync_flag = True

    def apply_visualization(self):
        """Function to perform Visualization
        """
        # Adding eps to avoid log 0
        depth_image_8bit = (
            (255 * np.log(self._depth_image + np.finfo(float).eps)) / math.log(2048)
        ).astype(np.uint8)
        self._depth_image_rgb = cv2.cvtColor(depth_image_8bit, cv2.COLOR_GRAY2BGR)
        if self._gesture != None:
            # Draw gesture type on the image
            gesture_text = f"[{self._frame_num}] Gesture:[{self._gesture}]"
            cv2.putText(
                self._depth_image_rgb,
                gesture_text,
                (40, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
            )

    def fill_publish_ptcld(self):
        """ Function to populate point cloud message
        """
        ptcld_msg = PointCloud2()
        ptcld_msg.header = self._header
        ptcld_msg.width = self._xyz_points.shape[0]
        ptcld_msg.height = 1
        ptcld_msg.fields = self._ptcld_fields
        ptcld_msg.point_step = len(self._ptcld_fields) * 4  # Length of a point in bytes
        ptcld_msg.row_step = ptcld_msg.point_step * ptcld_msg.width
        ptcld_msg.is_bigendian = False
        ptcld_msg.is_dense = False  # Set to False as invalid points exists
        ptcld = self._xyz_points.astype(np.float32)
        ptcld_msg.data = np.asarray(ptcld, np.float32).tobytes()

        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        # Get the transform
        transform = tf_buffer.lookup_transform(
            "map", ptcld_msg.header.frame_id, rospy.Time.now(), rospy.Duration(5.0)
        )
        # Apply the transformation
        ptcld_transformed = do_transform_cloud(ptcld_msg, transform)
        self._output_ptcld_publisher.publish(ptcld_transformed)

    def publish_outputs(self):
        """Function to Publish visualization output 
        """
        bridge = CvBridge()
        # output_depth_image = np.array(self._depth_image).reshape([self._frame_width, self._frame_height]).astype(np.uint16)
        output_image_ros_msg = bridge.cv2_to_imgmsg(
            self._depth_image_rgb, encoding="bgr8", header=self._header
        )
        self._output_image_publisher.publish(output_image_ros_msg)

        if self._enable_point_cloud:
            self.fill_publish_ptcld()

    def get_output(self):
        """ Function waits for the output messages, then performs visualization and publishes output """
        # Set the loop rate
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            while not self._is_camera_info_set:
                rospy.loginfo("Waiting for the camera_info topic..")
                time.sleep(1)
            self._header.frame_id = self._frame_id
            self._header.seq = self._frame_num
            self._header.stamp = rospy.Time.now()
            if self._depth_sync_flag:
                self._depth_sync_flag = False
                self.apply_visualization()
                self.publish_outputs()
                rospy.loginfo(f"Running the host node : {self._frame_num}")
                self._frame_num += 1
            rate.sleep()


if __name__ == "__main__":
    """ Main function """

    # ROS initialization
    rospy.init_node("adi_3dtof_gesture_recognition_host_node")

    input_topic_prefix = rospy.get_param("~input_topic_prefix", "cam1")
    enable_point_cloud = rospy.get_param("~enable_point_cloud", False)

    # Create the node
    adi_3dtof_gesture_recognition_node = ADI3DToFGestureRecognitionHost(
        input_topic_prefix, enable_point_cloud
    )
