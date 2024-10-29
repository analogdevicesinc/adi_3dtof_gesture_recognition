#!/usr/bin/env python

import sys
import time
import copy
import rospy
from math import pi
from collections import deque, Counter
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Int32
from moveit_commander.conversions import pose_to_list
from input_robot_factory import InputRobotFactory
from open_manipulator_robotic_arm import OpenManipulatorRoboticArm


class ADIToFGestureRecognitionRoboticArmControl:
    """
    This class subscribes for topic /output_gesture, and based on the value of gesture, related functionality is allocated to the respective robotic arm.
    """

    # defining slots
    __slots__ = [
        "_robotic_arm",
        "_scene",
        "_robot_name",
        "_input_robot_handle",
        "_display_trajectory_publisher",
        "_planning_frame",
        "_eef_link",
        "_group_names",
        "_previous_gesture",
        "_current_gesture",
        "_previous_action",
        "_current_action",
        "_input_robot_handle",
        "_history_buffer_labels",
        "_history_buffer_len",
        "_thresh_num_entries_in_history_buffer_to_start_action",
    ]

    def __init__(self):
        """
        Constructor of class
        """
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node(
            "adi_3dtof_gesture_recognition_arm_control_node", anonymous=True
        )
        self._robotic_arm = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()

        self._robot_name = rospy.get_param("~param_robot_name", "open_manipulator_x")

        input_robot_factory = InputRobotFactory()
        input_robot_factory.register("open_manipulator_x", OpenManipulatorRoboticArm)

        try:
            self._input_robot_handle = input_robot_factory.get_input_robot_handler(
                self._robot_name
            )
        except ValueError as e:
            print(e)
            return

        self._display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Prints Reference frame of the robot
        self._planning_frame = self._input_robot_handle._group_names_moveit_commander[
            0
        ].get_planning_frame()
        print("============ Reference frame: %s", str(self._planning_frame))

        # Prints end effector link of the robot
        self._eef_link = self._input_robot_handle._group_names_moveit_commander[
            0
        ].get_end_effector_link()
        print("============ End effector: %s", str(self._eef_link))

        # Prints all groups in robot
        self._group_names = self._robotic_arm.get_group_names()
        print("============ Robot Groups:", str(self._robotic_arm.get_group_names()))

        # Prints entire state of the robot
        print("============ Printing robot state")
        print(str(self._robotic_arm.get_current_state()))
        print("")

        self._previous_gesture = -1
        self._current_gesture = -1
        self._previous_action = -1
        self._current_action = -1
        self._history_buffer_len = 5
        self._thresh_num_entries_in_history_buffer_to_start_action = 4
        self._history_buffer_labels = deque(maxlen=self._history_buffer_len)

    def update_buffer_and_calculate_frequency(self, gesture_label):
        """Function to add a number to the buffer and calculate the most frequent number and its count

        Args:
            gesture_label(int): Current gesture label

        Returns:
            list: most_frequent_label, count
        """
        # Add the label to the buffer
        self._history_buffer_labels.append(gesture_label)

        # Calculate the frequency of each label in the buffer
        frequency = Counter(self._history_buffer_labels)

        # Find the label with the highest frequency
        most_frequent_label, count = frequency.most_common(1)[0]

        # Return the most frequent label and its count
        return most_frequent_label, count

    def move_arm_call_back(self, msg):
        """
        This function is call back when /gesture topic is received

        Args:
            msg (Int32): This is value of /gesture topic
        """
        rospy.loginfo("Gesture Topic =  %d", msg.data)

        self._current_gesture = msg.data

        most_frequent_label, most_frequent_label_count = self.update_buffer_and_calculate_frequency(
            self._current_gesture
        )
        if (
            most_frequent_label_count
            < self._thresh_num_entries_in_history_buffer_to_start_action
        ):
            # We do not have enough confidence to start the action, do nothing.
            return

        # We have enough confidence to start the action, set the gesture to label with highest confidence
        self._current_action = most_frequent_label
        current_action = self._current_action

        if self._current_action == self._previous_action:
            rospy.loginfo(
                "Previous action is same as current action, robot is doing nothing."
            )
        elif self._current_action == 1:
            rospy.loginfo("Grab Minion from Bucket 1 and drop it in bucket 2")
            self._input_robot_handle.open_gripper()
            self._input_robot_handle.move_to_pose1_up()
            self._input_robot_handle.move_to_pose1_down()
            self._input_robot_handle.grab_minion()
            self._input_robot_handle.move_to_pose1_up()
            self._input_robot_handle.move_to_pose2_up()
            self._input_robot_handle.move_to_pose2_down()
            self._input_robot_handle.open_gripper()
            self._input_robot_handle.move_to_pose2_up()
        elif self._current_action == 2:
            rospy.loginfo("Grab Minion from Bucket 2 and drop it in bucket 3")
            self._input_robot_handle.move_to_pose2_up()
            self._input_robot_handle.move_to_pose2_down()
            self._input_robot_handle.grab_minion()
            self._input_robot_handle.move_to_pose2_up()
            self._input_robot_handle.move_to_pose3_up()
            self._input_robot_handle.move_to_pose3_down()
            self._input_robot_handle.open_gripper()
            self._input_robot_handle.move_to_pose3_up()
            self._input_robot_handle.move_to_pose2_up()
        elif self._current_action == 3:
            rospy.loginfo("Grab Minion from Bucket 3 and drop it in bucket 1")
            self._input_robot_handle.move_to_pose3_up()
            self._input_robot_handle.move_to_pose3_down()
            self._input_robot_handle.grab_minion()
            self._input_robot_handle.move_to_pose3_up()
            self._input_robot_handle.move_to_pose1_up()
            self._input_robot_handle.move_to_pose1_down()
            self._input_robot_handle.open_gripper()
            self._input_robot_handle.move_to_pose1_up()
            self._input_robot_handle.move_to_pose2_up()
        else:
            rospy.loginfo("### Gesture not recognised, doing nothing.")
            current_action = self._previous_action

        # Set history
        self._previous_action = current_action
        self._history_buffer_labels.clear()


def adi_3dtof_gesture_recognition_arm_control_node():
    """
    This fucntion sets the suscriber topic and callback
    """
    adi_tof_gesture_recognition_arm_control = (
        ADIToFGestureRecognitionRoboticArmControl()
    )
    print(
        "adi_3dtof_gesture_recognition_robotic_arm_control_node node is running......"
    )
    adi_tof_gesture_recognition_arm_control._input_robot_handle.move_to_home()
    rospy.Subscriber(
        "/output_gesture",
        Int32,
        adi_tof_gesture_recognition_arm_control.move_arm_call_back,
        queue_size=1,
    )
    rospy.spin()
    adi_tof_gesture_recognition_arm_control.rate = rospy.Rate(20)


if __name__ == "__main__":
    """
    main function
    """
    adi_3dtof_gesture_recognition_arm_control_node()
