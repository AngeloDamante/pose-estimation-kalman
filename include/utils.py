import numpy as np
import cv2
import csv
from typing import Tuple, List
from scipy.spatial.transform import Rotation as R
from include.definitions import MARKER, TARGETS
from include.parameters import DELTA, DIST_COEFFS, INTRINSIC_MATRIX, MARKER_POINTS


def remap(value: float, x_a: float, x_b: float, y_a: float, y_b: float) -> float:
    """Remap

    Args:
        value (float):
        x_a (float):
        x_b (float):
        y_a (float):
        y_b (float):

    Returns:
        float:
    """
    value = np.clip(value, x_a, x_b)
    mapped_value = ((value - x_a) / (x_b - x_a)) * (y_b - y_a) + y_a
    return mapped_value
