"""Simulate MOTOR"""

from include.definitions import COMMAND, configure_logger
import logging

# log_level = logging.DEBUG
# logging.basicConfig(level=logging.DEBUG, format='%(name)s - %(levelname)s - %(message)s')

log_level = logging.DEBUG
logger = configure_logger()


class Motor:
    def __init__(self) -> None:
        self.speed = 0.0

    def get_speed(self) -> float:
        return self.speed

    def on_iteration(self, command, speed):
        logging.debug(f'[ MOTOR ]: parsing cmd = {command}, speed = {speed}')
        if command.value == COMMAND.forward.value:
            self.speed = speed
            return
        elif command.value == COMMAND.stop.value:
            self.speed = 0.0
            return
        elif command.value == COMMAND.turn_left.value:
            self.speed = speed
            return
        elif command.value == COMMAND.turn_right.value:
            self.speed = speed
            return