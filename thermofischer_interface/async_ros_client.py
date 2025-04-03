import rclpy
from rclpy.node import Node
import asyncio
from thermofischer_interface.logger import SereactLogger
from rclpy.callback_groups import ReentrantCallbackGroup
import enum
logger = SereactLogger(__name__, ros_log=False)
from std_msgs.msg import Int32
import enum

class StackLightMode(enum.Enum):
    NORMAL = 1
    MANUAL_PROCESSING = 2
    MAINTENANCE = 3
    FAULT = 5
    WAITING_FOR_RESTART = 6
    EMERGENCY = 7
    CUSTOM = 8
class ThermoFischerStacklightMode(enum.Enum):
    ON= 1
    NOT_ON=2
    UNKNOWN= 3
    OFF= 8

class AsyncRosClient(Node):
    
    def __init__(self, loop) -> None:
        super().__init__("async_ros_client")
        self.loop = loop
        self.server_state = None
        self.current_request = None
        self.last_request = None
        self.topic_callback = None

        self.subscriptions_callbacks = ReentrantCallbackGroup()

        self.system_state = None
        self.stack_light_publisher = self.create_publisher(Int32,  "/stack_light/set", 10)



    async def set_stack_light(self, mode: ThermoFischerStacklightMode):
        stack_light_mode = StackLightMode.MAINTENANCE
        if mode == ThermoFischerStacklightMode.NOT_ON:
            stack_light_mode = StackLightMode.EMERGENCY
        elif mode == ThermoFischerStacklightMode.UNKNOWN:
            stack_light_mode = StackLightMode.WAITING_FOR_RESTART
        elif mode == ThermoFischerStacklightMode.ON:
            stack_light_mode = StackLightMode.NORMAL
        msg = Int32()
        msg.data = stack_light_mode.value
        self.stack_light_publisher.publish(msg)

    async def run(self):
        # like that, it might be heavy on the event loop, not so sure if we want to do that
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01) # TODO I dont know if this is a good or bad timeout
            await asyncio.sleep(0.001)
        self.destroy_node() # will destroy itself when ros is not ok anymore



