#!/usr/bin/env python
"""Point cloud utility functions
"""

import math
import numpy as np
import cv2


def generate_range_to_3d_lut(k_raw, d_raw, image_width, image_height):
    """ Generate the LUT to convert radial to point cloud

    Args:
        k_raw (list): K matrix
        d_raw (list): Distortion coefs
        image_width (int): Frame width
        image_height (int): Frame height

    Returns:
        NDArray[np.float64]: LUT
    """

    _, _ = cv2.getOptimalNewCameraMatrix(
        k_raw, d_raw, (image_width, image_height), 0, (image_width, image_height)
    )

    range_to_3d_lut = np.zeros((image_height, image_width, 3))

    for y in range(image_height):
        for x in range(image_width):
            distorted_pt = np.ones((1, 1, 2)).astype(np.float64)
            distorted_pt[0, 0, 0] = x
            distorted_pt[0, 0, 1] = y
            undistorted_pt = np.zeros((1, 1, 2)).astype(np.float64)

            undistorted_pt = cv2.undistortPoints(distorted_pt, k_raw, d_raw)

            ux = undistorted_pt[0, 0, 0]
            uy = undistorted_pt[0, 0, 1]

            scale_factor = 1 / math.sqrt(1 + (ux * ux) + (uy * uy))

            range_to_3d_lut[y, x, 0] = ux * scale_factor
            range_to_3d_lut[y, x, 1] = uy * scale_factor
            range_to_3d_lut[y, x, 2] = scale_factor

    return range_to_3d_lut


def compute_point_cloud(range_image, range_to_3d_lut):
    """Compute point cloud

    Args:
        range_image (NDArray[np.int]): Depth image
        range_to_3d_lut (NDArray[np.float64]): LUT

    Returns:
        NDArray[np.float64]: Point cloud 
    """
    if (
        range_image.shape[0] != range_to_3d_lut.shape[0]
        and range_image.shape[1] != range_to_3d_lut.shape[1]
    ):
        print("ERROR!!: Lut table size does not match input image")
        return None

    xyz_frame = range_image[:, :, np.newaxis] * range_to_3d_lut / 1000

    xyz_frame = xyz_frame.reshape(range_image.shape[0] * range_image.shape[1], 3)

    return xyz_frame
