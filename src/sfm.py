"""State Finite Machine"""

import cv2
import numpy as np
import logging
from typing import Tuple
from include.definitions import MARKER, STATE, configure_logger
from src.VisionKalmanFilter import VisionKalmanFilter
from include.parameters import (
    VISION_KF_SETTINGS,
    MARKER_POINTS,
    DIST_COEFFS,
    INTRINSIC_MATRIX
)
from include.utils import (
    extract_desired_corners,
    find_targets, chose_target,
    compute_state,
    compute_refs,
    save_data
)

log_level = logging.DEBUG
logger = configure_logger()

filename_raw = "data_raw_cv.csv"
filename_filtered = "data_filtered_kf.csv"
fields = ['Frame', 'Tx', 'Ty', 'Tz']


class SFM:
    def __init__(self, d_signed: float, kf_enable: bool = True, max_lost_frame: int = 20) -> None:
        # init params
        self.d_signed: float = d_signed
        self.max_lost_frame: int = max_lost_frame

        # attributes
        self.kf = VisionKalmanFilter(VISION_KF_SETTINGS['NX'], VISION_KF_SETTINGS['NZ'], min_samples=20)
        self.target: MARKER = None
        self.state: STATE = STATE.off

        # output
        self.d_ref = 0.0
        self.theta_ref = 0.0
        self.num_lost_frame = 0
        self.num_frame = 0
        self.data_raw = []
        self.data_filtered = []

        # setting KF
        if kf_enable is True:
            self.kf.start()
            self.kf.enable_vision_mode(VISION_KF_SETTINGS['K1'], VISION_KF_SETTINGS['K2'], VISION_KF_SETTINGS['DT'])

    def get_state(self) -> STATE:
        return self.state

    def get_ref(self) -> Tuple[float, float]:
        return self.theta_ref, self.d_ref

    def publish(self, d, theta):
        self.d_ref = d
        self.theta_ref = theta

    def get_collect_data(self) -> Tuple[list, list]:
        return self.data_raw, self.data_filtered

    #################################################################
    def clean_signal(self, tvec, rvec):
        measurement = compute_state(t_vec=tvec, r_vec=rvec)
        self.kf.update(measurement)
        self.data_raw.append([self.num_frame, tvec[0][0], tvec[1][0], tvec[2][0]])
        if not self.kf.is_ready(): return tvec
        logging.debug(f'[ REFGEN ]: CLEAN SIGNAL for {str(self.target)}')
        state_hat = self.kf.predict()
        tvec = np.array([state_hat[0], state_hat[1], state_hat[2]], np.float32)
        self.data_filtered.append([self.num_frame, tvec[0][0], tvec[1][0], tvec[2][0]])
        return tvec

    def predict_signal(self):
        if not self.kf.is_ready(): return None
        logging.debug(f"[ REFGEN ]: PREDICTION for {str(self.target)}")
        state_hat = self.kf.predict()
        tvec = np.array([state_hat[0], state_hat[1], state_hat[2]], np.float32)
        self.data_filtered.append([self.num_frame, tvec[0][0], tvec[1][0], tvec[2][0]])
        return tvec

        #################################################################

    def check_for_disable(self, ids, corners) -> bool:
        is_detected, _ = extract_desired_corners(id_desired=MARKER.disable_tracker.value, ids=ids, corners=corners)
        if not is_detected: return False
        logging.debug(f'[ REFGEN ]: disable marker detected')
        self.num_lost_frame = 0
        self.state = STATE.off
        return True

    #################################################################
    def off_state(self, ids, corners):
        flag, _ = extract_desired_corners(MARKER.enable_tracker.value, ids, corners)
        self.publish(0.0, 0.0)

        if len(self.data_filtered) > 0:
            logging.debug('[ REFGEN ]: saving raw data')
            save_data(filename_filtered, self.data_filtered, fields)
        if len(self.data_raw) > 0:
            logging.debug('[ REFGEN ]: saving filtered data')
            save_data(filename_raw, self.data_raw, fields)

        if not flag: return
        logging.debug(f'[ REFGEN ]: enable marker detected')
        self.state = STATE.enable

    def enable_state(self, ids, corners):
        # extract markers detected and compute reference marker
        detected_targets, detected_corners = found_targets(ids, corners)
        ref_marker = chose_target(detected_targets, detected_corners)
        logging.debug(f'[ REFGEN ]: chosen marker = {ref_marker}')

        # change state iff reference marker is detected and computed
        if ref_marker is None: return
        self.target = MARKER(ref_marker)
        self.kf.reset()
        self.data_raw.clear()
        self.data_filtered.clear()
        self.state = STATE.follow

    def follow_state(self, ids, corners):
        # detected targets
        flag, corners = extract_desired_corners(id_desired=self.target.value, ids=ids, corners=corners)
        tvec_hat = None
        if not flag:
            logging.debug(f"[ REFGEN ]: {str(self.target)} LOST")
            if self.num_lost_frame < self.max_lost_frame:
                self.num_lost_frame += 1
                tvec_hat = self.predict_signal()
        else:
            logging.debug(f"[ REFGEN ]: FOLLOW {str(self.target)}")
            self.num_lost_frame = 0
            _, rvec, tvec = cv2.solvePnP(objectPoints=MARKER_POINTS, imagePoints=corners, cameraMatrix=INTRINSIC_MATRIX,
                                         distCoeffs=DIST_COEFFS, flags=cv2.SOLVEPNP_ITERATIVE)
            tvec_hat = self.clean_signal(tvec=tvec, rvec=rvec)

        if tvec_hat is None:
            self.publish(0.0, 0.0)
            return

        logging.debug(f'[ REFGEN ]: tvec = {tvec_hat}')
        ref_d, ref_theta = compute_refs(tvec_hat[0][0], tvec_hat[1][0], tvec_hat[2][0])
        if ref_d < self.d_signed:
            logging.debug(f'[ REFGEN ]: target reached')
            self.publish(0.0, 0.0)
            self.state = STATE.enable
        else:
            logging.debug(f'[ REFGEN ]: target following')
            self.publish(ref_d, ref_theta)