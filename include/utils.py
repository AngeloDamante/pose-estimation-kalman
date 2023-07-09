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

def extract_desired_corners(id_desired, ids, corners) -> Tuple[bool, np.ndarray]:
    """Found and extract desired marker and its corners from list of markers
      Args:
        id_desired (int)
        ids (np.ndarray): Nx1
        corners (tuple(np.ndarray)): Nx4x2
      Returns:
        flag (bool): true if found else otherwise
    """
    if len(ids) != len(corners):
        return False, []
    if not id_desired in ids:
        return False, []

    desired_index = np.where(ids == id_desired)[0][0]
    return True, corners[desired_index][0]
