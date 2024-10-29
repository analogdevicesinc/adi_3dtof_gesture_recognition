#!/usr/bin/env python
import numpy as np

""" Hand Segmentation
"""


class HandSegmentation:
    """Hand Segmentation class"""

    __slots__ = [
        "_frame_width",
        "_frame_height",
        "_x_coords",
        "_y_coords",
        "_flag11",
        "_processing_roi",
        "_threshold_for_points",
        "_processing_roi_mask",
    ]

    def get_processing_roi(self, frame_width, frame_height, exclusion_border_percentage):
        """Function to get the processing ROI

        Args:
            frame_width (int): Frame width
            frame_height (int): Frame height
            exclusion_border_percentage (float): Border size as a percentage of frame width

        Returns:
            list: The coordinates for the top-left and bottom-right points of the ROI
        """
        # Create a reference bounding box after leaving 'exclusion_border_percentage' percentage pixels on all 4 sides
        if exclusion_border_percentage >= 0.5:
            print(
                f"The neglect pixels value should be lesser than 0.5 but given value is : {exclusion_border_percentage}. Therefore, the default value (0.2) is chosen."
            )
            exclusion_border_percentage = 0.2

        ref_top_left = (
            int(frame_width * exclusion_border_percentage),
            int(frame_height * exclusion_border_percentage),
        )
        ref_bottom_right = (
            frame_width - ref_top_left[0],
            frame_height - ref_top_left[1],
        )
        return [ref_top_left, ref_bottom_right]

    def __init__(self, frame_width, frame_height, exclusion_border_percentage):
        """Constructor

        Args:
            frame_width (int): Frame width
            frame_height (int): Frame height
            exclusion_border_percentage (float): Border size as a percentage of frame width
        """

        self._frame_width = frame_width
        self._frame_height = frame_height
        y_coords, x_coords = np.meshgrid(range(frame_height), range(frame_width), indexing="ij")
        self._x_coords = x_coords.reshape(frame_height * frame_width, 1)
        self._y_coords = y_coords.reshape(frame_height * frame_width, 1)
        self._flag11 = np.full((frame_height * frame_width), False, dtype=bool)
        self._processing_roi = self.get_processing_roi(
            frame_width, frame_height, exclusion_border_percentage
        )

        processing_roi_top_left = self._processing_roi[0]
        processing_roi_bottom_right = self._processing_roi[1]
        self._processing_roi_mask = np.zeros([frame_height, frame_width])
        self._processing_roi_mask[
            processing_roi_top_left[1] : processing_roi_bottom_right[1],
            processing_roi_top_left[0] : processing_roi_bottom_right[0],
        ] = 1
        self._processing_roi_mask = self._processing_roi_mask.reshape(frame_height * frame_width, 1)
        # Number of points needed for processing
        self._threshold_for_points = 2000

    def perform_segmentation(
        self,
        depth_image,
        ir_image,
        pts_orig,
        frame_width,
        frame_height,
        max_hand_dist=0.6,
        dist_closest_pt_th=0.2,
        z_closest_pt_th=0.15,
        dist_extent_th=0.2,
    ):  # in cm
        """
        Performs Hand segmentation, extrcats the hand pixels from the depth image.
        It uses the point cloud in m unit length and extract the hand points (based on the geometric rules
        defined by distance thresholds and the assumption that the hand is the closest object to the scene)
        the returned hand points are in mm unit length

        Args:
            depth_image (NDArray[np.uint16]): Depth image
            ir_image (NDArray[np.uint16]): IR image
            pts_orig (NDArray[np.float64]): _description_
            frame_width (int): Image width
            frame_height (int): Image height
            max_hand_dist (float, optional): Maximum distance to skip the frame. Defaults to 0.6.
            dist_closest_pt_th (float, optional): Distance treshold for the first stage. Defaults to 0.2.
            z_closest_pt_th (float, optional): Distance threshold in z-direction. Defaults to 0.1.
            dist_extent_th (float, optional): Distance threshold for the second stage. Defaults to 0.2.

        Raises:
            ValueError: If thegiven width and height are not equal to the values fgiven during construction time.

        Returns:
            list: List of segmented points/pixels and other debug information
        """

        # special condition when there are no points
        if len(pts_orig) == 0:
            return (None, None, None, self._processing_roi, None, True)

        if frame_width != self._frame_width:
            raise ValueError(
                f"Error: frame_width is not equal to the initial value : passed : {frame_width}, init : {self._frame_width}"
            )

        if frame_height != self._frame_height:
            raise ValueError(
                f"Error: frame_height is not equal to the initial value : passed : {frame_height}, init : {self._frame_height}"
            )

        # Mask the points which are outside of the ROI
        pts_mask = self._processing_roi_mask * pts_orig[:, :3]

        pts_full = np.concatenate((pts_mask, self._y_coords, self._x_coords), axis=1)
        pts = pts_full[pts_full[:, 2] != 0]

        # maximum number of nearest objects extracting to detect the hand
        maximum_iteration_for_hand_extraction = 3
        # minimum number of points needed to be in object to consider it as hand
        hand_detection_confidence_threshold = 100

        i = 0
        while i < maximum_iteration_for_hand_extraction:
            if len(pts) == 0:
                return (None, None, None, self._processing_roi, None, True)

            # flag based on distance from closest point
            closest_pt = pts[np.argmin(pts[:, 2]), :]
            is_skip_frame = closest_pt[2] > max_hand_dist

            if is_skip_frame:
                # No pixels within the processing depth zone, return
                return (None, None, None, self._processing_roi, None, is_skip_frame)

            dists = np.linalg.norm(pts[:, :3] - closest_pt[None, :3], axis=1)
            flag = np.logical_and(
                dists < dist_closest_pt_th,
                pts[:, 2] - closest_pt[None, 2] < z_closest_pt_th,
            )  # 20 cm
            hand_pts_first_stage = pts[flag, :]

            # second filter from top point
            top_pt = hand_pts_first_stage[np.argmin(hand_pts_first_stage[:, 1]), :]
            dists = np.linalg.norm(hand_pts_first_stage[:, :3] - top_pt[None, :3], axis=1)
            flag = dists < dist_extent_th

            # project to mm
            # Get x,y location of the points
            pos_2d = (hand_pts_first_stage[flag, 3:5]).astype(int)
            hand_pts = hand_pts_first_stage[flag, :3] * 1e3

            # Get the 2d positions of the points in the hand, needed for Visualization and ROI
            final_mask = np.full((frame_height, frame_width), False, dtype=bool)
            final_mask[pos_2d[:, 0], pos_2d[:, 1]] = True

            # Find min max points in the final mask image to extract Hand ROI
            true_indices = np.argwhere(final_mask == True)
            top_left = true_indices.min(axis=0)
            bottom_right = true_indices.max(axis=0)

            # Find depth pixels coresponding to he extracted hand
            hand_depth = (depth_image * final_mask)[
                top_left[0] : bottom_right[0], top_left[1] : bottom_right[1]
            ]
            # Find IR pixels coresponding to he extracted hand
            hand_ir = (ir_image * final_mask)[
                top_left[0] : bottom_right[0], top_left[1] : bottom_right[1]
            ]

            final_mask = final_mask.reshape(frame_height * frame_width)

            if hand_depth.size > hand_detection_confidence_threshold:
                break
            i = i + 1

            # Remove smaller objects near to the camera
            mask = np.any(np.all(pts[:, 3:] == pos_2d[:, None], axis=2), axis=0)
            pts = pts[~mask]

        # Build debug packet
        debug_info = {"stage2_mask": final_mask, "hand_roi": [top_left, bottom_right]}

        if hand_pts.shape[0] < self._threshold_for_points:
            # Not many pixels in the processing ROI
            is_skip_frame = True

        return (
            hand_depth,
            hand_ir,
            hand_pts,
            self._processing_roi,
            debug_info,
            is_skip_frame,
        )
