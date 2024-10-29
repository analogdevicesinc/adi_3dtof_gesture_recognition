#!/usr/bin/env python
"""Thread for the process 
"""

import time
import rospy
import adi_3dtof_gesture_recognition.input_thread as input_thread
import adi_3dtof_gesture_recognition.output_thread as output_thread

# Queue
_global_abort_thread = False


def abort():
    """ Abort the thread """
    global _global_abort_thread
    _global_abort_thread = True


def process_thread(process_handle, process_args):
    """Process Thread 

    Args:
        sensor_handle (object): Process handle
    """
    disable_algorithm = process_args["disable_algorithm"]
    ros_loop_rate = process_args["ros_loop_rate"]
    # Set the loop rate
    rate = rospy.Rate(ros_loop_rate)

    frame_num = 0
    abort_loop = False

    # Loop
    while not rospy.is_shutdown():

        if _global_abort_thread:
            break

        # Get the frame from the sensor
        frame_info = None
        num_tries = 0
        # Get the input, let it try more more times, if the inout is not available
        frame_info = input_thread.get_frame()
        while frame_info is None:
            frame_info = input_thread.get_frame()
            num_tries += 1
            if num_tries > 10 and frame_info is None:
                abort_loop = True
                break
            print(f"Trying for input frame, trycount = {num_tries}")
            time.sleep(1)

        if abort_loop:
            break

        # Process
        gesture_id = -1

        if not disable_algorithm:

            # Hand segmentation is done in the input thread, and the output is par of frame_info

            # Detection
            if not frame_info["no_hand_found"]:
                gesture_id = process_handle.detect(
                    frame_info["hand_depth"],
                    frame_info["hand_ir"],
                    frame_info["hand_pts"],
                )

        output_frame_info = {
            "frame_num": frame_info["frame_num"],
            "time_stamp": frame_info["time_stamp"],
            "depth_image": frame_info["depth_image"],
            "ir_image": frame_info["ir_image"],
            "hand_segmentation_debug_info": frame_info["hand_segmentation_debug_info"],
            "output_gesture": gesture_id,
            "processing_roi": frame_info["processing_roi"],
        }

        # Submit the output info to output thread
        output_thread.put_frame(output_frame_info)

        frame_num += 1

        rospy.loginfo(f"Finished processing frame : {frame_num}")

        # Sleep
        rate.sleep()

    # Signal abort to input and output threads
    output_thread.abort()
    input_thread.abort()
