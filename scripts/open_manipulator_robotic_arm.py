#!/usr/bin/env python
"""_summary_
"""

import rospy
import moveit_commander


class OpenManipulatorRoboticArm:
    """
    This class defines the functionalities for OpenManipulator robotic arm
    """

    # defining slots
    __slots__ = [
        "_group_names",
        "_group_names_moveit_commander",
        "_joint_goal_gripper",
        "_joint_goal_arm",
    ]

    def __init__(self):
        self._group_names = ["arm", "gripper"]
        self._group_names_moveit_commander = []

        for group_name in self._group_names:
            self._group_names_moveit_commander.append(
                moveit_commander.MoveGroupCommander(group_name)
            )

    def move_to_home(self):
        """
        This is Function to move open manipulator robot to it's home position.
        """
        rospy.loginfo("### Open manipulator is moving to it's home.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            0
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = 1.570000
        self._joint_goal_gripper[1] = -1.000000
        self._joint_goal_gripper[2] = 0.296706
        self._joint_goal_gripper[3] = 0.698132
        self._group_names_moveit_commander[0].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[0].stop()

    def move_to_home_left(self):
        """
        This is function to move open manipulator robot to left of it's home position.
        """
        rospy.loginfo("### Open manipulator moving to left of its home position.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            0
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = 0.0872665
        self._joint_goal_gripper[1] = -1.0000000
        self._joint_goal_gripper[2] = 0.2967060
        self._joint_goal_gripper[3] = 0.6981320
        self._group_names_moveit_commander[0].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[0].stop()

    def move_to_home_right(self):
        """
        This is function to move open manipulator robot to right of it's home position.
        """
        rospy.loginfo("### Open manipulator moving to right of its home position.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            0
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = -0.0872665
        self._joint_goal_gripper[1] = -1.0000000
        self._joint_goal_gripper[2] = 0.2967060
        self._joint_goal_gripper[3] = 0.6981320
        self._group_names_moveit_commander[0].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[0].stop()

    def move_to_home_head_up(self):
        """
        This is function to move the open manipulator head upwards.
        """
        rospy.loginfo("### Open manipulator is raising its head.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            0
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = 0.000000
        self._joint_goal_gripper[1] = -1.000000
        self._joint_goal_gripper[2] = 0.296706
        self._joint_goal_gripper[3] = 0.610865
        self._group_names_moveit_commander[0].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[0].stop()

    def move_to_home_head_down(self):
        """
        This is fucntion to move the open manipulator head downwards
        """
        rospy.loginfo("### Open manipulator is bowing it's head.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            0
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = 0.000000
        self._joint_goal_gripper[1] = -1.000000
        self._joint_goal_gripper[2] = 0.296706
        self._joint_goal_gripper[3] = 0.785398
        self._group_names_moveit_commander[0].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[0].stop()

    def move_to_zero(self):
        """
        This is function to move to zero location of joints
        """
        rospy.loginfo("### Open manipulator is moving to it's zero position.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            0
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = 0
        self._joint_goal_gripper[1] = 0
        self._joint_goal_gripper[2] = 0
        self._joint_goal_gripper[3] = 0
        self._group_names_moveit_commander[0].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[0].stop()

    def move_to_pose1_down(self):
        """
        This is function to move to pose1 down position
        """
        rospy.loginfo("### Open manipulator is moving to pose1 down.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            0
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = -1.048
        self._joint_goal_gripper[1] = 0.000000
        self._joint_goal_gripper[2] = 0.209440
        self._joint_goal_gripper[3] = 0.785398
        self._group_names_moveit_commander[0].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[0].stop()

    def move_to_pose1_up(self):
        """
        This is function to move to pose1 up position
        """
        rospy.loginfo("### Open manipulator is moving to pose1 up.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            0
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = -1.048
        self._joint_goal_gripper[1] = -0.523599
        self._joint_goal_gripper[2] = 0.209440
        self._joint_goal_gripper[3] = 0.785398
        self._group_names_moveit_commander[0].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[0].stop()

    def move_to_pose2_down(self):
        """
        This is function to move to pose2 down position
        """
        rospy.loginfo("### Open manipulator is moving to pose2 down.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            0
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = 0.000000
        self._joint_goal_gripper[1] = 0.000000
        self._joint_goal_gripper[2] = 0.209440
        self._joint_goal_gripper[3] = 0.785398
        self._group_names_moveit_commander[0].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[0].stop()

    def move_to_pose2_up(self):
        """
        This is function to move to pose2 up position
        """
        rospy.loginfo("### Open manipulator is moving to pose2 up.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            0
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = 0.000000
        self._joint_goal_gripper[1] = -0.523599
        self._joint_goal_gripper[2] = 0.209440
        self._joint_goal_gripper[3] = 0.785398
        self._group_names_moveit_commander[0].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[0].stop()

    def move_to_pose3_down(self):
        """
        This is function to move to pose3 down position
        """
        rospy.loginfo("### Open manipulator is moving to pose3 down.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            0
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = 1.048
        self._joint_goal_gripper[1] = 0.000000
        self._joint_goal_gripper[2] = 0.209440
        self._joint_goal_gripper[3] = 0.785398
        self._group_names_moveit_commander[0].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[0].stop()

    def move_to_pose3_up(self):
        """
        This is function to move to pos3 up position
        """
        rospy.loginfo("### Open manipulator is moving to pose3 up.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            0
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = 1.048
        self._joint_goal_gripper[1] = -0.523599
        self._joint_goal_gripper[2] = 0.209440
        self._joint_goal_gripper[3] = 0.785398
        self._group_names_moveit_commander[0].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[0].stop()

    def open_gripper(self):
        """
        This is function to open the gripper
        """
        rospy.loginfo("### Open manipulator is opening gripper.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            1
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = 0.01
        self._joint_goal_gripper[1] = 0.01
        self._group_names_moveit_commander[1].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[1].stop()

    def close_gripper(self):
        """
        This is function to close the gripper
        """
        rospy.loginfo("### Open manipulator is closing gripper.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            1
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = -0.007
        self._joint_goal_gripper[1] = -0.007
        self._group_names_moveit_commander[1].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement s
        self._group_names_moveit_commander[1].stop()

    def grab_minion(self):
        """
        This is function to grab the minion
        """
        rospy.loginfo("### Open manipulator is grabbing minion.")
        self._joint_goal_gripper = self._group_names_moveit_commander[
            1
        ].get_current_joint_values()
        self._joint_goal_gripper[0] = 0.002
        self._joint_goal_gripper[1] = 0.002
        self._group_names_moveit_commander[1].go(self._joint_goal_gripper, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self._group_names_moveit_commander[1].stop()
