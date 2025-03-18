from dataclasses import dataclass, field
from typing import List, Dict, Union, Optional
import enum
import uuid
import time
from datetime import datetime, timezone
from thermofischer_interface.logger import SereactLogger
logger = SereactLogger(__name__)




class StackLightMode(enum.Enum):
    NORMAL = 1
    MANUAL_PROCESSING = 2
    MAINTENANCE = 3
    FAULT = 5
    WAITING_FOR_RESTART = 6
    EMERGENCY = 7
    CUSTOM = 8

class SystemState(enum.Enum):
    NOT_READY = 0
    NORMAL = 1
    ERROR = 2
    EMERGENCY = 3
    BIN_FULL = 4
    DOOR_OPEN = 5

class GeneralState(enum.Enum):
    ERROR = 1
    STARTING = 2
    OPERATIONAL = 3


def is_valid_uuid(val):
    try:
        uuid_obj = uuid.UUID(val, version=4)
    except ValueError:
        return False
    return True

def generate_uuid() -> str:
    """
    Generate a unique identifier based on the hostname and the current time.

    Returns:
        str: Random unique identifier.
    """
    return str(uuid.uuid4())

def generate_timestamp() -> str:
    """
    Generate a timestamp.
    Returns:
        str: Timestamp.
    """
    return datetime.now(tz=timezone.utc).isoformat()
    

def log_event(logger, level, message):
    if level == "info":
        logger.info(message)
    elif level == "warning":
        logger.warning(message)
    elif level == "error":
        logger.error(message)
    elif level == "critical":
        logger.error(message)
    else:
        logger.info(message)

class TimeIt:
    """A context manager to measure the execution time of a code block.

    Usage:
        with TimeIt("Block 1"):
            # Code block to be measured

    Attributes:
        print_output (bool): Indicates whether the time statistics should be printed.
        last_parent (TimeIt): Reference to the parent TimeIt instance (used for nested blocks).
        level (int): Level of nesting (used for indentation of printed statistics).
    """

    print_output: bool = True
    last_parent: Optional['TimeIt'] = None
    level: int = -1
    class_logger: SereactLogger = SereactLogger(__name__ + '.TimeIt')

    def __init__(self, block_name: str, output_list: Optional[List[Dict[str, Union[str, float]]]] = None):
        """Initialize a TimeIt instance.

        Args:
            block_name (str): The name of the code block being measured.
            output_list (Optional[List[Dict[str, Union[str, float]]]]): A list to store the time statistics.
        """
        self.block_name: str = block_name
        self.start_time: Optional[float] = None
        self.end_time: Optional[float] = None
        self.child_outputs: List[Dict[str, Union[str, float]]] = []
        self.parent: Optional['TimeIt'] = None
        self.output_list: Optional[List[Dict[str,
                                             Union[str, float]]]] = output_list

    def __enter__(self):
        """Start measuring the time when entering the context."""
        self.parent = TimeIt.last_parent
        TimeIt.last_parent = self
        TimeIt.level += 1
        self.start_time = time.perf_counter()

    def print_time_stats(self, data_list: List[Dict[str, Union[str, float]]]):
        """Print the time statistics.

        Args:
            data_list (List[Dict[str, Union[str, float]]]): List containing time statistics for different blocks.
        """
        for entry in data_list:
            self.class_logger.info('%s%s: %0.1fms' % (
                '  ' * entry["level"], entry["name"], entry["duration_ms"]))

    def __exit__(self, exc_type, exc_value, traceback):
        """Stop measuring the time and print statistics if necessary.

        Args:
            exc_type: Type of exception raised (None if no exception).
            exc_value: The exception raised (None if no exception).
            traceback: Traceback information (None if no exception).
        """
        self.end_time = time.perf_counter()
        duration_ms = (self.end_time - self.start_time) * 1000
        block_data = {"name": self.block_name,
                      "duration_ms": duration_ms, "level": TimeIt.level}

        if self.parent:
            self.parent.child_outputs.append(block_data)
            self.parent.child_outputs += self.child_outputs
        else:
            self.child_outputs.insert(0, block_data)
            if self.output_list is not None:
                self.output_list += self.child_outputs
            if TimeIt.print_output:
                self.print_time_stats(self.child_outputs)

        TimeIt.level -= 1
        TimeIt.last_parent = self.parent