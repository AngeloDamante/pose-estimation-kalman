import cv2
from cv2 import aruco
import numpy as np
import logging
from include.parameters import DELTA, DEG_RANGE
from include.definitions import configure_logger
from src.sfm import SFM
from src.controller import Controller
from src.motor import Motor

SPACE_BAR = 32

log_level = logging.DEBUG
logger = configure_logger()

# NODES
refgen = SFM(d_signed=DELTA, kf_enable=True)
control = Controller(deg_range=DEG_RANGE, d_min=DELTA, d_max=2.0, speed_min=0.4, speed_max=1.0)
motor = Motor()


def view_on_screen(img, theta,d,  command, speed, state):
    cv2.putText(img=img, org=(400, 300), text=f'state={state.value}', fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=0.6, color=(200, 50, 0), thickness=2)
    cv2.putText(img=img, org=(400, 330), text=f'cmd={command.value}', fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=0.6, color=(200, 50, 0), thickness=2)
    cv2.putText(img=img, org=(400, 360), text=f'd={str(d)}', fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=0.6, color=(200, 50, 0), thickness=2)
    cv2.putText(img=img, org=(400, 390), text=f'theta={str(theta)}', fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=0.6, color=(200, 50, 0), thickness=2)
    cv2.putText(img=img, org=(400, 420), text=f'speed={str(speed)}', fontFace=cv2.FONT_HERSHEY_DUPLEX, fontScale=0.6, color=(200, 50, 0), thickness=2)


def lifecycle():
    webcam = cv2.VideoCapture(0)
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

    while (webcam.isOpened()):
        logging.info("---------------------------------")
        ret, img = webcam.read()
        if not ret:
            webcam.release()
            raise AssertionError('resource not available')

        logging.debug('[ CAMERA NODE ]')
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        logging.debug('[ ARUCO NODE ]')
        corners, ids, _ = aruco.detectMarkers(frame, dictionary, parameters=aruco.DetectorParameters())
        if len(corners) > 0:
            logging.debug('[ ARUCO ]: see something')
            aruco.drawDetectedMarkers(img, corners)

        logging.debug('[ REFGEN NODE ]')
        if ids is None: ids = []
        refgen.on_iteration(ids=ids, corners=corners)
        theta_ref, d_ref = refgen.get_ref()

        logging.debug('[ CONTROL NODE ]')
        command, speed = control.chose_command(ref_theta=theta_ref, ref_d=d_ref)

        logging.debug('[ MOTOR NODE ]')
        motor.on_iteration(command=command, speed=speed)
        logging.debug(f'[ MOTOR NODE ]: speed = {str(motor.get_speed())} ')

        # debug on screen
        view_on_screen(img, theta_ref, d_ref, command, motor.get_speed(), refgen.get_state())

        # view
        cv2.imshow("out", img)
        button = cv2.waitKey(1)
        if button == SPACE_BAR:
            webcam.release()
            cv2.destroyAllWindows()
            break


if __name__ == '__main__':
    lifecycle()
