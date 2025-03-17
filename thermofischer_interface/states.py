

import enum

class PickingState(enum.Enum):
    IDLE = 0
    PICKING = 1
    PLACING = 2
    HOMING = 3
    HOMING_PICK = 4
    ERROR = 5


class AccessControlState(enum.Enum):
    CLOSED = 1
    REQUEST_OPEN = 2
    OPEN = 3
    REQUEST_CLOSE = 4

class StackLightMode(enum.Enum):
    NORMAL = 1
    MANUAL_PROCESSING = 2
    MAINTENANCE = 3
    FAULT = 5
    WAITING_FOR_RESTART = 6
    EMERGENCY = 7
    CUSTOM = 8

class GeneralState(enum.Enum):
    ERROR = 1
    STARTING = 2
    OPERATIONAL = 3


class EmergencyMode(enum.Enum):
    ERROR = 0
    READY = 1
    EMERGENCY = 2
    NEED_TO_CONFIRM = 3
    CONFIRMED = 4

class SystemState(enum.Enum):
    PAUSED = 0
    READY = 1
    EMERGENCY = 2
    FAULT = 3
    DOOR_OPEN = 4