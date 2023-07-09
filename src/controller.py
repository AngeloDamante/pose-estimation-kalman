"""CONTROL"""

from typing import Tuple
import logging
from include.definitions import COMMAND, configure_logger
from include.utils import remap

# log_level = logging.DEBUG
# logging.basicConfig(level=log_level, format='%(name)s - %(levelname)s - %(message)s')
log_level = logging.DEBUG
logger = configure_logger()


class Controller:
    def __init__(self, deg_range: tuple, d_min: float, d_max: float, speed_min: float, speed_max: float) -> None:
        self.deg_range = deg_range
        self.d_min = d_min
        self.d_max = d_max
        self.speed_min = speed_min
        self.speed_max = speed_max

    def chose_command(self, ref_theta: float, ref_d: float) -> Tuple[COMMAND, float]:
        logging.debug(f'[ CONTROL ]: parsing theta = {ref_theta}, d = {ref_d}')
        if ref_d < self.d_min:
            cmd = COMMAND.stop
        elif self.deg_range[0] < ref_theta < self.deg_range[1]:
            cmd = COMMAND.forward
        elif ref_theta > 0:
            cmd = COMMAND.turn_right
        elif ref_theta < 0:
            cmd = COMMAND.turn_left
        speed = remap(ref_d, self.d_min, self.d_max, self.speed_min, self.speed_max)
        return cmd, speed
