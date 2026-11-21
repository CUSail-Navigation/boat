"""ROS-free geometry for turning a detection box and a depth image into a measurement."""

import numpy as np


def box_depth(depth_image, box, depth_scale, roi_fraction, min_depth, max_depth):
    """
    Median of the valid depths in the central roi_fraction of the box, in meters, or None.

    depth_image is the raw depth frame aligned to the color frame and box is
    (x1, y1, x2, y2) in pixels. Depths outside [min_depth, max_depth] are ignored.
    """
    x1, y1, x2, y2 = box
    margin_x = int((x2 - x1) * (1 - roi_fraction) / 2)
    margin_y = int((y2 - y1) * (1 - roi_fraction) / 2)
    roi = depth_image[
        y1 + margin_y : y2 - margin_y + 1, x1 + margin_x : x2 - margin_x + 1
    ]
    depths = roi[roi > 0] * depth_scale
    depths = depths[(depths >= min_depth) & (depths <= max_depth)]
    if depths.size == 0:
        return None
    return float(np.median(depths))


def box_width(box, depth, fx):
    """Physical width in meters of a box at the given depth, for focal length fx in pixels."""
    x1, _, x2, _ = box
    return (x2 - x1) * depth / fx
