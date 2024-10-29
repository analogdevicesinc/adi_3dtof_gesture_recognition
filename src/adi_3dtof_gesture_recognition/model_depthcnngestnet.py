#!/usr/bin/env python
"""The DepthCNNGetsNet model definition
"""
import cv2
import tensorflow as tf
from tensorflow.keras.layers.experimental import preprocessing
from tensorflow.keras import layers
from tensorflow.keras.models import Sequential
from tensorflow.keras.preprocessing.image import ImageDataGenerator

# Import the Desired Version of EfficientNet
from tensorflow.keras.applications import EfficientNetB0
import numpy as np


class ModelDepthCNNGestNet:
    """ Class definition for the depthcnngestnet Model
    """

    __slots__ = [
        "_model_path",
        "_model",
        "_model_image_size",
        "_input_dtype",
        "_output_dtype",
        "_input_index",
        "_output_gesture_index",
        "_output_hand_confidence_index",
        "_hand_confidence_threshold",
        "_gesture_confidence_threshold",
    ]

    def __init__(self):
        """Constructor"""
        self._model_path = ""
        self._model = None
        self._model_image_size = 112
        self._input_dtype = np.float32
        self._output_dtype = np.float32
        self._input_index = 0
        self._output_gesture_index = 514
        self._output_hand_confidence_index = 511
        self._hand_confidence_threshold = 0.5
        self._gesture_confidence_threshold = 0.5

    def load_model(self, model_path, hand_confidence_threshold, gesture_confidence_threshold):
        """Load the model file

        Args:
            model_path (str): Path to the model file
        """
        self._model_path = model_path

        # Load
        self._model = self.load_depthcnngestnet_model()

        #setting confidence Thresholds
        self._hand_confidence_threshold = hand_confidence_threshold
        self._gesture_confidence_threshold = gesture_confidence_threshold

    def load_depthcnngestnet_model(self, num_classes=18):
        """Load the model

        Args:
            num_classes (int, optional): Number of classes. Defaults to 18.

        Returns:
            interpreter: Tensorflow Lite Interpreter
        """
        interpreter = tf.lite.Interpreter(model_path=str(self._model_path))

        # Get input and output tensors.
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        # Allocate tensors
        interpreter.allocate_tensors()

        # Setup model parameters
        self._input_dtype = input_details[0]["dtype"]
        self._output_dtype = output_details[0]["dtype"]

        self._input_index = input_details[0]["index"]
        self._output_gesture_index = output_details[1]["index"]
        self._output_hand_confidence_index = output_details[0]["index"]

        return interpreter

    def convert_roi_to_square_and_resize(self, img, interpolation=cv2.INTER_AREA):
        """Convert the Hand ROI to a square ROI

        Args:
            img (NDArray[np.uint16]): Hand ROI 
            interpolation (int, optional): Iterpolation type. Defaults to cv2.INTER_AREA.

        Returns:
            NDArray[np.uint16]: Squared ROI
        """
        h, w = img.shape[:2]
        c = None if len(img.shape) < 3 else img.shape[2]

        if h == w:
            return cv2.resize(
                img, (self._model_image_size, self._model_image_size), interpolation
            )
        if h > w:
            dif = h
        else:
            dif = w
        x_pos = int((dif - w) / 2.0)
        y_pos = int((dif - h) / 2.0)
        if c is None:
            mask = np.zeros((dif, dif), dtype=img.dtype)
            mask[y_pos : y_pos + h, x_pos : x_pos + w] = img[:h, :w]
        else:
            mask = np.zeros((dif, dif, c), dtype=img.dtype)
            mask[y_pos : y_pos + h, x_pos : x_pos + w, :] = img[:h, :w, :]
        image = cv2.resize(
            mask, (self._model_image_size, self._model_image_size), interpolation
        )
        return image

    def detect(self, hand_depth, hand_ir, hand_pts):
        """Detection function

        Args:
            hand_depth (NDArray[np.uint16]): Hand ROI image(Depth)
            hand_ir (NDArray[np.uint16]): Hand ROI image(IR)
            hand_pts (NDArray[np.float64]): Hand ROI image(Point cloud)

        Returns:
            int : Gesture label
        """

        # Normalize depth values.
        max_pixel = np.max(hand_depth)
        min_pixel = np.min(hand_depth[np.nonzero(hand_depth)]) + 1
        pixel_range = max_pixel - min_pixel
        hand_depth[np.nonzero(hand_depth)] -= np.uint16(min_pixel)
        hand_depth = (
            hand_depth / pixel_range
        ) * 255  # to help create a 8-bit image for opencv HSV colorization
        image_8bit = np.clip(a=hand_depth, a_min=0, a_max=255).astype(np.uint8)

        image = self.convert_roi_to_square_and_resize(image_8bit)
        image_rgb = cv2.applyColorMap(image, cv2.COLORMAP_HSV)
        zero_indexes = np.where(image == 0)
        image_rgb[zero_indexes] = 0
        # cv2.imshow("wa", image_rgb);cv2.waitKey(2)
        image_input = np.resize(
            image_rgb, (1, image_rgb.shape[0], image_rgb.shape[1], image_rgb.shape[2])
        ).astype(self._input_dtype)
        image_input = image_input / 255.0  # normalizing the input
        self._model.set_tensor(self._input_index, image_input)
        self._model.invoke()
        prediction = self._model.get_tensor(self._output_gesture_index)[0]
        hand_confidence = self._model.get_tensor(self._output_hand_confidence_index)[0]
        if hand_confidence < self._hand_confidence_threshold:
            gesture_pred_label = -1 #set the gesture label to no hand found
        else:
            gesture_pred_index = int(np.argmax(prediction)) 
            if prediction[gesture_pred_index] < self._gesture_confidence_threshold:
                gesture_pred_label = -1 #Set the gesture label to unknown gesture type if confidence is not high
            else:
                gesture_pred_label = gesture_pred_index + 1 #making gesture indices match real_world labels

        return gesture_pred_label
