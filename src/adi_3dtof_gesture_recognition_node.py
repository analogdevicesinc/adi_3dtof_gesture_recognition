#!/usr/bin/env python
""" The script to perform Gesture Recognition using depth images. 
"""
import threading
import rospy
import os

from dynamic_reconfigure.server import Server

from adi_3dtof_gesture_recognition.input_factory import InputFactory
from adi_3dtof_gesture_recognition.input_rostopic import InputROSTopic
from adi_3dtof_gesture_recognition.input_file import InputFile

# from adi_3dtof_gesture_recognition.input_network import InputNetwork

from adi_3dtof_gesture_recognition.model_factory import ModelFactory
from adi_3dtof_gesture_recognition.model_depthcnngestnet import ModelDepthCNNGestNet

from adi_3dtof_gesture_recognition.output_factory import OutputFactory
from adi_3dtof_gesture_recognition.output_rostopic import OutputROSTopic

from adi_3dtof_gesture_recognition.input_thread import input_thread
from adi_3dtof_gesture_recognition.process_thread import process_thread
from adi_3dtof_gesture_recognition.output_thread import output_thread
from adi_3dtof_gesture_recognition.cfg import ADI3DToFGestureRecognitionConfig


class ADI3DToFGestureRecognition:
    """ Gesture Recognition class """

    # defining slots
    __slots__ = [
        "_optical_camera_link",
        "_ros_loop_rate",
        "_input_type",
        "_sensor_id",
        "_model_type",
        "_model_file",
        "_output_type",
        "_output_topic_prefix",
        "_input_handle",
        "_model_handle",
        "_output_handle",
        "_frame_width",
        "_frame_height",
        "_input_thread",
        "_process_thread",
        "_output_thread",
        "_disable_algorithm",
        "_exclusion_border_percentage",
        "_ab_threshold",
        "_confidence_threshold",
        "_hand_confidence_threshold",
        "_gesture_confidence_threshold",
        "_config_file_name_of_tof_sdk",
        "_frame_type",
        "_camera_info_k",
        "_camera_info_d",
        "_input_args",
        "_process_args",
        "_output_args",
    ]

    def __init__(
        self,
        optical_camera_link,
        ros_loop_rate,
        in_type,
        sen_id,
        mod_type,
        mod_file,
        out_type,
        out_topic_prefix,
        disable_algorithm,
        exclusion_border_percentage,
        ab_threshold,
        confidence_threshold,
        hand_confidence_threshold,
        gesture_confidence_threshold,
        config_file_name_of_tof_sdk,
        frame_type,
        enable_output_image_compression,
        enable_output_image_flip,
    ):
        """
        Constructor

        Args:
            in_type (str): Input Type (adtf31xx or rostopic or file)
            sen_id (str): Sensor ID(adtf31xx:"" or rostopic:Input topic prefix or file: Input filename)
            mod_type (str): Model type(depthcnngestnet)
            mod_file (str): Path to the weights file
            out_type (str): Output type(rostopic or rostopic_file)
            out_topic_prefix (str): (Output topic prefix)
        """
        self._optical_camera_link = optical_camera_link
        self._ros_loop_rate = ros_loop_rate
        self._input_type = in_type
        self._sensor_id = sen_id
        self._model_type = mod_type
        self._model_file = mod_file
        self._output_type = out_type
        self._output_topic_prefix = out_topic_prefix
        self._disable_algorithm = disable_algorithm
        self._exclusion_border_percentage = exclusion_border_percentage
        self._ab_threshold = ab_threshold
        self._confidence_threshold = confidence_threshold
        self._hand_confidence_threshold = hand_confidence_threshold
        self._gesture_confidence_threshold = gesture_confidence_threshold
        self._config_file_name_of_tof_sdk = config_file_name_of_tof_sdk
        self._frame_type = frame_type
        self._camera_info_k = None
        self._camera_info_d = None

        # Get input handle
        # Register all input handlers
        input_factory = InputFactory()
        if self._input_type == "adtf31xx":
            from adi_3dtof_gesture_recognition.input_adtf31xx import InputADTF31xx

            input_factory.register("adtf31xx", InputADTF31xx)
        input_factory.register("file", InputFile)
        input_factory.register("rostopic", InputROSTopic)
        # input_factory.register("network",InputNetworkADTF31xx)
        try:
            self._input_handle = input_factory.get_input_handler(self._input_type)
        except ValueError as e:
            print(e)
            return

        # Register all model handlers
        model_factory = ModelFactory()
        model_factory.register("depthcnngestnet", ModelDepthCNNGestNet)
        try:
            self._model_handle = model_factory.get_model_handler(self._model_type)
        except ValueError as e:
            print(e)
            return

        # Register all output handlers
        output_factory = OutputFactory()
        output_factory.register("rostopic", OutputROSTopic)
        output_factory.register(
            "rostopic_file", OutputROSTopic
        )  # same handler for both modes

        try:
            self._output_handle = output_factory.get_output_handler(self._output_type)
        except ValueError as e:
            print(e)
            return

        # Open Input
        self._input_handle.open_sensor(
            self._sensor_id, self._frame_type, self._config_file_name_of_tof_sdk
        )

        # Load Model
        self._model_handle.load_model(self._model_file, self._hand_confidence_threshold, self._gesture_confidence_threshold)

        # Configure
        try:
            self._input_handle.configure_sensor()
        except IOError as e:
            print(e)
            return

        self._frame_width = self._input_handle.frame_width
        self._frame_height = self._input_handle.frame_height
        self._camera_info_k = self._input_handle.camera_info_k
        self._camera_info_d = self._input_handle.camera_info_d

        # Open output
        # Build output filename
        # The default name is gesture_recognition_output.avi,
        # but, if the input_type is "fileio", then use the inputfilename to build the output filename as well.
        output_filename = "gesture_recognition_output.avi"
        if self._input_type == "file":
            output_filename = (
                self._sensor_id[0:-4] + "_gr_out.avi"
            )  # The assumption is the input file has ".bin" extension
        self._output_handle.open_sensor(
            self._frame_width,
            self._frame_height,
            self._output_topic_prefix,
            output_filename,
            enable_output_image_compression,
            enable_output_image_flip,
        )

        # Create threads.
        max_input_queue_length = 1
        max_output_queue_length = 1

        if self._input_type == "file":
            max_input_queue_length = 100

        if self._output_type == "rostopic_file":
            max_output_queue_length = 100

        # Input args
        self._input_args = {
            "ros_loop_rate": self._ros_loop_rate,
            "frame_width": self._frame_width,
            "frame_height": self._frame_height,
            "max_queue_length": max_input_queue_length,
            "exclusion_border_percentage": self._exclusion_border_percentage,
            "disable_algorithm": self._disable_algorithm,
        }

        # Process args
        self._process_args = {
            "ros_loop_rate": self._ros_loop_rate,
            "disable_algorithm": self._disable_algorithm,
        }

        # Output args
        self._output_args = {
            "ros_loop_rate": self._ros_loop_rate,
            "optical_camera_link": self._optical_camera_link,
            "max_queue_length": max_output_queue_length,
            "camera_info_k": self._camera_info_k,
            "camera_info_d": self._camera_info_d,
        }

        self._input_thread = threading.Thread(
            target=input_thread, args=(self._input_handle, self._input_args)
        )
        self._process_thread = threading.Thread(
            target=process_thread, args=(self._model_handle, self._process_args)
        )
        self._output_thread = threading.Thread(
            target=output_thread, args=(self._output_handle, self._output_args)
        )

        # Setup Dynamic Reconfigure Server
        srv = Server(
            ADI3DToFGestureRecognitionConfig, self.dynamic_reconfigure_callback
        )

        # Start input and output threads and then process thread.
        # process thread needs both input and output threads to be running
        # for it's operation(get_input() and put_output())
        self._input_thread.start()
        self._output_thread.start()
        self._process_thread.start()

        # Joining the threads.
        self._input_thread.join()
        self._process_thread.join()
        self._output_thread.join()

        # Close the sensor
        print("Close the sensor")
        self._output_handle.close_sensor()

    def dynamic_reconfigure_callback(self, config, level):
        """ Dynamic Reconfigure Callback function 

        Args:
            config (dict): Dictionary of updated parameters 
            level (int): The level value is the result of ORing together all of the level values of the parameters that have changed.

        Returns:
            dict: Dictionary of updated parameters
        """
        rospy.loginfo(
            f"Reconfigure Request: {config.ab_threshold=} {config.confidence_threshold=} {level=}"
        )
        if level & 1:
            rospy.loginfo("Updating ab_threshold")
            self._input_handle.set_AB_invalidation_threshold(self._ab_threshold)
        if level & 2:
            rospy.loginfo("Updating confidence_threshold")
            self._input_handle.set_confidence_threshold(self._confidence_threshold)
        return config


if __name__ == "__main__":
    """ Main function """

    # ROS initialization
    rospy.init_node("adi_3dtof_gesture_recognition_node")

    camera_link = rospy.get_param("~camera_link", "camera_link")
    optical_camera_link = rospy.get_param("~optical_camera_link", "optical_camera_link")
    ros_loop_rate = rospy.get_param("~ros_loop_rate", 10)

    # Get the input type
    # "adtf31xx" - Sensor(OnBoard),
    # "file" - File,
    # "rostopic"- ROS Topic,
    # "network" - Sensor(Network)
    input_type = rospy.get_param("~input_type", "rostopic")
    # Sensor id - Used only if the Input type==1/2/3,
    # 1 - Input file name
    # 2 - ROS Topic prefix (the full depth image and ir image topic names would be cam_prefix/depth_image and cam_prefix/ir_image)
    # 3 - Network ID of the sensor(ex:10.41.0.1)
    sensor_id = rospy.get_param("~sensor_id", "/cam1")
    # Get the output type(
    # "rostopic" - ROS topic,
    # "rostopic_file" - ROS Topic+Fileout)
    output_type = rospy.get_param("~output_type", "rostopic")
    # Get the output filename(Only needed if output_type == 1)
    output_topic_prefix = rospy.get_param("~output_topic_prefix", "/cam1")

    disable_algorithm = rospy.get_param("~disable_algorithm", False)
    # Exclusion border percentage w.r.t image width
    exclusion_border_percentage = rospy.get_param("~exclusion_border_percentage", 0.2)
    model_type = rospy.get_param("~model_type", "depthcnngestnet")
    # Model file
    model_file = rospy.get_param(
        "~model_file", "weights/gesture_recognition_cnn_2_stage_model.tflite"
    )

    #Hand confidence Threshold
    hand_confidence_threshold = rospy.get_param("~hand_confidence_threshold", 0.5)

    #Gesture Confidence Threshold
    gesture_confidence_threshold = rospy.get_param("~gesture_confidence_threshold", 0.5)

    # frame type to be set to device
    frame_type = rospy.get_param("~frame_type", "lr-qnative")
    # ab threshold to be set to device
    ab_threshold = rospy.get_param("~ab_threshold", 10)
    # confidence threshold to be set to devic
    confidence_threshold = rospy.get_param("~confidence_threshold", 10)
    # config file from where device parameter values canbe taken
    config_file_name_of_tof_sdk = rospy.get_param(
        "~config_file_name_of_tof_sdk", "config/config_adsd3500_adsd3100.json"
    )
    # Enable compression for the output image topic
    enable_output_image_compression = rospy.get_param(
        "~enable_output_image_compression", False
    )
    # Enable flip for visualization(only output)
    enable_output_image_flip = rospy.get_param("~enable_output_image_flip", True)

    # Create the node
    adi_3dtof_gesture_recognition_node = ADI3DToFGestureRecognition(
        optical_camera_link,
        ros_loop_rate,
        input_type,
        sensor_id,
        model_type,
        model_file,
        output_type,
        output_topic_prefix,
        disable_algorithm,
        exclusion_border_percentage,
        ab_threshold,
        confidence_threshold,
        hand_confidence_threshold,
        gesture_confidence_threshold,
        config_file_name_of_tof_sdk,
        frame_type,
        enable_output_image_compression,
        enable_output_image_flip,
    )
