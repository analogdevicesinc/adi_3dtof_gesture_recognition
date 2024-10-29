#!/usr/bin/env python
import rospy

"""Input factory class
"""


class InputFactory:
    """Factory class for the input """

    __slots__ = ["_input_handlers"]

    def __init__(self):
        self._input_handlers = {}

    def register(self, input_type, input_handle):
        """Register a particular mode

        Args:
            input_type (str): Input mode
            input_handle (object): Handle for the input mode
        """
        self._input_handlers[input_type] = input_handle

    def get_input_handler(self, input_type):
        """ Get the handle corresponding to the chosen input mode

        Args:
            input_type (str): Input mode

        Raises:
            ValueError: When the mode is not registered 

        Returns:
            object : Input handle
        """
        if input_type not in self._input_handlers:
            raise ValueError(
                f"Error: No input handle is registed for the given input type : {input_type}"
            )
        return self._input_handlers[input_type]()
