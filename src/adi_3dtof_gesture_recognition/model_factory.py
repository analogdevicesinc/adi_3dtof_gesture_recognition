#!/usr/bin/env python
"""Model factory class
"""


class ModelFactory:
    """Class definition for Model Factory class"""

    __slots__ = ["_model_handlers"]

    def __init__(self):
        """Constructor
        """
        self._model_handlers = {}

    def register(self, model_type, model_handle):
        """Register the model

        Args:
            model_type (str): Model type
            model_handle (object): Model handle
        """
        self._model_handlers[model_type] = model_handle

    def get_model_handler(self, model_type):
        """Get the model Handle 

        Args:
            model_type (str): Model type

        Raises:
            ValueError: When the model is not registered

        Returns:
            object: Model handle
        """
        if model_type not in self._model_handlers:
            raise ValueError(
                f"Error: No model handle is registed for the given model type : {model_type}"
            )
        return self._model_handlers[model_type]()
