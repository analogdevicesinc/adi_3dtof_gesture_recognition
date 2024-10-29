#!/usr/bin/env python
"""Thread for Input processing
"""

import queue
import rospy
import numpy as np
import adi_3dtof_gesture_recognition.hand_segmentation as hand_segmentation

# Queue
_global_input_queue = None
_global_abort_thread = False


def abort():
    """Abort the thread"""
    global _global_abort_thread
    _global_abort_thread = True


def input_thread(sensor_handle, input_args):
    """The input thread function

    Args:
        sensor_handle (object): Input sensor handle
        input_args (list): Parameters for the thread
    """
    frame_width = input_args["frame_width"]
    frame_height = input_args["frame_height"]
    exclusion_border_percentage = input_args["exclusion_border_percentage"]
    max_queue_length = input_args["max_queue_length"]
    ros_loop_rate = input_args["ros_loop_rate"]

    # Set the loop rate
    rate = rospy.Rate(ros_loop_rate)

    # Create the queue
    global _global_input_queue
    _global_input_queue = queue.Queue(max_queue_length)
    hand_segment = hand_segmentation.HandSegmentation(
        frame_width, frame_height, exclusion_border_percentage
    )

    disable_algorithm = input_args["disable_algorithm"]

    # Loop
    while not rospy.is_shutdown():

        if _global_abort_thread:
            break

        # Get the frame from the sensor
        frame_info = sensor_handle.get_frame()

        if frame_info is not None:
            # Run Hand segmentation
            if not disable_algorithm:
                point_cloud = np.asarray(frame_info["xyz_points"])
                (
                    frame_info["hand_depth"],
                    frame_info["hand_ir"],
                    frame_info["hand_pts"],
                    frame_info["processing_roi"],
                    frame_info["hand_segmentation_debug_info"],
                    frame_info["no_hand_found"],
                ) = hand_segment.perform_segmentation(
                    frame_info["depth_image"],
                    frame_info["ir_image"],
                    point_cloud,
                    input_args["frame_width"],
                    input_args["frame_height"],
                )
            else:
                frame_info["hand_depth"] = None
                frame_info["hand_ir"] = None
                frame_info["hand_pts"] = None
                frame_info["processing_roi"] = None
                frame_info["hand_segmentation_debug_info"] = None
                frame_info["no_hand_found"] = None

        # Put it in the queue
        if _global_input_queue.full():
            # If the queue is full, remove the last item
            _global_input_queue.get()
            _global_input_queue.task_done()

        _global_input_queue.put(frame_info)

        # Sleep
        rate.sleep()


def get_frame():
    """Get the first buffer in the queue

    Returns:
        dict : Frame info
    """
    frame = None
    if _global_input_queue is not None:
        frame = _global_input_queue.get()

    return frame
