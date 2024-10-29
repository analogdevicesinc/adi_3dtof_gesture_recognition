#!/usr/bin/env python
import rospy


class InputRobotFactory:
    """
    This is the Abstract class in Factory method design pattern
    """

    __slots__ = ["_input_robot_handlers"]

    def __init__(self):
        self._input_robot_handlers = {}

    def register(self, input_robot_type, input_robot_handle):
        """
        This method registers all the subclasses


        Args:
            input_robot_type (string): This is name of robot
            input_robot_handle (class): This is concrete class 
        """
        self._input_robot_handlers[input_robot_type] = input_robot_handle

    def get_input_robot_handler(self, input_robot_type):
        """
        This class returns one of the concrete class based on robot type

        Args:
            input_robot_type (string): This is name of robot

        Raises:
            ValueError: raises an exception if the input to this method is an invalid robot name.

        Returns:
            _type_: base class data type
        """
        if input_robot_type not in self._input_robot_handlers:
            raise ValueError(
                f"Error: No input robot handle is registed for the given input robot type : {input_robot_type}"
            )
        return self._input_robot_handlers[input_robot_type]()
