# cython: annotation_typing = False
import datetime
import logging
import uuid
from logging import LogRecord
from multiprocessing import Process, Queue

import colorlog
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import rclpy
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
import time


volatile_qos = QoSProfile(
    durability=DurabilityPolicy.VOLATILE,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=2
)


DISABLE_LIST = ["__main__"]


def get_uuid():
    return str(uuid.uuid4())







class SereactLogger(logging.Logger):
    def __init__(self, name: str = "logimat-interface", disable_console_warnings: bool = False, ros_log: bool = True) -> None:
        super().__init__(name)

        # Set level to DEBUG to log all levels
        debugger_level = logging.DEBUG
        self.setLevel(debugger_level)

        # Create a file handler
        # file_handler = logging.FileHandler("sereact_system.log")
        # file_handler.setLevel(debugger_level)

        # Create a console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)

        # Create a color formatter
        console_formatter = colorlog.ColoredFormatter('%(log_color)s%(levelname)s | %(process)s | %(name)s | %(asctime)s | %(message)s',
                                                      log_colors={
                                                          'DEBUG': 'cyan',
                                                          'INFO': 'green',
                                                          'WARNING': 'yellow',
                                                          'ERROR': 'red',
                                                      },
                                                      reset=True,
                                                      style='%',
                                                      datefmt='%Y-%m-%d %H:%M:%S'
                                                      )
        console_handler.setFormatter(console_formatter)
        self.addHandler(console_handler)

        # Suppress warning messages only for the console handler
        if disable_console_warnings:
            console_handler.addFilter(
                lambda record: record.levelno != logging.WARNING)

        # file_formatter = logging.Formatter(
        #     '%(levelname)s | %(process)s | %(name)s | %(asctime)s | %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
        # file_handler.setFormatter(file_formatter)
        # self.addHandler(file_handler)

        if name in DISABLE_LIST:
            for handler in self.handlers:
                if isinstance(handler, logging.StreamHandler):
                    self.removeHandler(handler)