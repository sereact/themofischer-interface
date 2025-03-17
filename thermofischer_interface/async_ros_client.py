from turtle import st
import rclpy
from rclpy.node import Node
import asyncio
from thermofischer_interface.image_utils import image_to_base64, base64_to_image
from thermofischer_interface.utils import generate_timestamp
from std_msgs.msg import Bool, Int32, String
from sensor_msgs.msg import Image
from sereact_custom_messaging.msg import SereactErrorCode
import datetime

from thermofischer_interface.system_state_monitor import SystemStateMonitorClient 
from thermofischer_interface.utils import StackLightMode
from thermofischer_interface.states import SystemState
from thermofischer_interface.logger import SereactLogger
from rclpy.callback_groups import ReentrantCallbackGroup
from thermofischer_interface.utils import log_event
import enum
import os
import json
logger = SereactLogger(__name__, ros_log=False)


class ThermoFischerStacklightMode(enum.Enum):
    ON= 1
    NOT_ON=2
    UNKNOWN= 3

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


        # self.stack_light_subscription = self.create_subscription(Int32, self.config["stack_light_set_topic"], self.stack_light_callback, 10, callback_group=self.subscriptions_callbacks)

        # self.create_timer(0.2, self.timer_callback)

    def send_to_ws_ros_image(self, topic, image: Image):
        image = self.cv_bridge.imgmsg_to_cv2(image)
        message = image_to_base64(image)
        self.loop.create_task(self.topic_callback(topic, message))
    
    def is_system_ready(self):
        return self.system_state == SystemState.READY.value

    def stack_light_callback(self, msg):
        self.current_topic_messages["/stack_light/state"] = msg.data

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

    def publish_notification_to_frontend(self, level, msg):
        log_event(logger, level, msg)
        topic = f"/logger/{level.lower()}"
        if self.topic_callback is not None:
            self.loop.create_task(self.topic_callback(topic, msg))

    def add_callback(self, callback):
        self.topic_callback = callback



    async def start(self):
        #call system state monitor to start the system
        await self.system_state_monitor_client.start_system_async()
        pass

    async def stop(self):
        #call system state monitor to stop the system
        await self.system_state_monitor_client.stop_system_async()
        pass
    
    async def pause(self):
        #call system state monitor to pause the system
        await self.system_state_monitor_client.pause_system_async()
        pass

    async def resume(self):
        #call system state monitor to resume the system
        await self.system_state_monitor_client.resume_system_async()
        pass


    async def init(self):
        while self.system_state is None:
            await asyncio.sleep(0.1)
    
    async def run(self):
        # like that, it might be heavy on the event loop, not so sure if we want to do that
        while rclpy.ok():
            # smaller timeout sec might be less heavy on the event loop
            # as this is not awaitable, that mean0s,
            #  we block the whole event loop here, but maybe its also couritine in any case, so I dont really know, but this is the way people do this
            rclpy.spin_once(self, timeout_sec=0.01) # TODO I dont know if this is a good or bad timeout
            await asyncio.sleep(0.001)
        self.destroy_node() # will destroy itself when ros is not ok anymore



