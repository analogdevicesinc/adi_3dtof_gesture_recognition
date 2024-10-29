#!/usr/bin/env python
"""Thread for output processing
"""

import queue
import rospy
import copy

# Queue
_global_output_queue = None
_global_abort_thread = False


def abort():
    """abort the thread"""
    global _global_abort_thread
    _global_abort_thread = True


def output_thread(output_handle, output_args):
    """Output Thread

    Args:
        output_handle (object): Output handle
    """
    max_queue_length = output_args["max_queue_length"]
    ros_loop_rate = output_args["ros_loop_rate"]
    # Set the loop rate
    rate = rospy.Rate(ros_loop_rate)

    # Create the queue

    global _global_output_queue
    _global_output_queue = queue.Queue(max_queue_length)

    # Loop
    while not rospy.is_shutdown():

        if _global_abort_thread:
            break

        # Get the frame from the output queue
        if not _global_output_queue.empty():
            output_frame_info = _global_output_queue.get()
            _global_output_queue.task_done()

            # Perform output action
            output_handle.publish_frame(output_frame_info, output_args)

        # Sleep
        rate.sleep()

    if _global_abort_thread:
        # Flush the Queue
        while not _global_output_queue.empty():
            # Get the frame from the output queue
            output_frame_info = _global_output_queue.get()
            # Perform output action
            output_handle.publish_frame(output_frame_info, output_args)


def put_frame(frame_info):
    """Insert the current frame into the queue

    Returns:
        dict: Frame Info
    """
    # Put it in the queue
    if _global_output_queue.full():
        # If the queue is full, remove the last item
        _global_output_queue.get()
        _global_output_queue.task_done()

    _global_output_queue.put(copy.deepcopy(frame_info))
