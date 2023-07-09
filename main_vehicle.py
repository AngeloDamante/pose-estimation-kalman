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
