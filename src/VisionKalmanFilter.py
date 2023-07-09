""" Define VisionKalmanFilter Class to handle aruco marker filtering

    Predict:
        x_prior = F * x + B * u
        P_prior = F * P * F_T + Q

    Update:
        x_posterior = x_prior + K * innovation
        innovation = z - H * x_prior
        P_posterior = (I - K * H) * P_prior
        K = P_prior * H_T * (H * P_prior * H_T + R)

    The main goal is generate a robust reference and its obtained in two ways:
        - reducing acquiring noise
        - reference prediction
"""

__author__ = "Angelone"
__version__ = "1.1.0"
__main__ = "angelo.damante16@gmail.com"

import cv2
import numpy as np
from include.definitions import configure_logger
import logging

# log_level = logging.DEBUG
# logging.basicConfig(level=log_level, format='%(name)s - %(levelname)s - %(message)s')

log_level = logging.DEBUG
logger = configure_logger()

class VisionKalmanFilter:
    def __init__(self, nX: int, nZ: int, min_samples: int = 10, nU: int = 0) -> None:
        """Filter creation

        Args:
            nX (int): number of dynamic params
            nZ (int): number of measure params
            nU (int): number of control params (optional)
            min_samples (int): minimum number of samples to collect initial measurements
        """
        self.nX = nX
        self.nZ = nZ
        self.nU = nU
        self.min_samples = min_samples

        self.num_samples = 0
        self.is_enabled = False
        self.kf = cv2.KalmanFilter(dynamParams=nX, measureParams=nZ, controlParams=nU)

    def set_matrixes(self, F: np.ndarray = None, H: np.ndarray = None, Q: np.ndarray = None, R: np.ndarray = None, B: np.ndarray = None):
        """Set Main Matrixes of KF

        Args:
            F (np.ndarray, optional): Transition Matrix. Defaults to None.
            H (np.ndarray, optional): Measurement Matrix. Defaults to None.
            Q (np.ndarray, optional): Process Noise Covariance Matrix. Defaults to None.
            R (np.ndarray, optional): Measurement Noise Covariance Matrix. Defaults to None.
            B (np.ndarray, optional): Control Matrix. Defaults to None.
        """
        if F is not None:
            self.kf.transitionMatrix = F.astype(np.float32)
        if H is not None:
            self.kf.measurementMatrix = H.astype(np.float32)
        if Q is not None:
            self.kf.processNoiseCov = Q.astype(np.float32)
        if R is not None:
            self.kf.measurementNoiseCov = R.astype(np.float32)
        if B is not None and self.nU > 0:
            self.kf.ControlMatrix = B.astype(np.float32)
