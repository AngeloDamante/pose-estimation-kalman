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
