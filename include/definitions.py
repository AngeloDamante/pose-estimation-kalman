"""Definitions of type utilities"""

from enum import Enum
import logging

def configure_logger(log_lvl=logging.DEBUG):
    # fmt = '%(levelname)s - %(filename)s - %(module)s - %(lineno)s - %(message)s'
    # fmt = '%(filename)s - %(lineno)s - %(message)s'
    fmt = '%(message)s'
    datefmt = '%I:%M:%S %p'
    hdl = [ logging.FileHandler("out.log", mode='w'), logging.StreamHandler()]
    logger = logging.basicConfig(format = fmt, level=log_lvl, datefmt=datefmt, handlers=hdl)
    return logger

class MARKER(Enum):
    enable_tracker: int = 1
    disable_tracker: int = 0
    target_0: int = 128
    target_1: int = 129


class COMMAND(Enum):
    forward: str = "FORWARD"
    stop: str = "STOP"
    turn_left: str = "TURN_LEFT"
    turn_right: str = "TURN_RIGHT"




TARGETS = [MARKER.target_0, MARKER.target_1]