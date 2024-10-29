#!/usr/bin/env python
"""Output factory abstract class
"""


class OutputFactory:
    """Class defintion for the output factory"""

    __slots__ = ["_output_handlers"]

    def __init__(self):
        self._output_handlers = {}

    def register(self, output_type, output_handle):
        """Register output mode

        Args:
            output_type (str): Output type
            output_handle (object): Output handle
        """
        self._output_handlers[output_type] = output_handle

    def get_output_handler(self, output_type):
        """Get hndle for the given mode

        Args:
            output_type (str): Output type

        Raises:
            ValueError: If the mode is not registered

        Returns:
            object : Output mode
        """
        if output_type not in self._output_handlers:
            raise ValueError(
                f"Error: No output handle is registed for the given output type : {output_type}"
            )
        return self._output_handlers[output_type](output_type)
