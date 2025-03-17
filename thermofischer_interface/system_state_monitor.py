from rclpy.node import Node 
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup 
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32, Int8, Int32MultiArray, Bool 
import rclpy 
import json 
from industrial_msgs.msg import RobotStatus 
from thermofischer_interface.states import AccessControlState, GeneralState, EmergencyMode, SystemState, StackLightMode 
from sereact_custom_messaging.srv import BoolSrv 
from thermofischer_interface.ros_message import ROBOT_STATUS_QOS
from thermofischer_interface.ros_message import GeneralServiceClient
from thermofischer_interface.capto_api_client import CaptoApiClient
import os
import time
from thermofischer_interface.logger import SereactLogger
logger  = SereactLogger(__name__, ros_log=False)

TIMEOUT = os.environ.get("STATE_TIMEOUT", 30)
CONFIG_FILE = os.environ.get("CONFIG_FILE", "assets/config_files/config.json")

class SystemStateMonitor(Node):
    def __init__(self, config):
        super().__init__("system_state_monitor")

        self.states_callback_group = ReentrantCallbackGroup()
        self.cam_states = {}
        self.cam_subscribers = {}
        self.config = config
        self.robot_state: RobotStatus = None
        self.picking_state = None
        self.emergency_mode = EmergencyMode.ERROR
        self.capto_state = GeneralState.ERROR
        self.gripper_state = GeneralState.ERROR
        self.stack_light_state = StackLightMode.NORMAL

        self.timout_times = {}
        self.system_state_publisher = self.create_publisher(Int32, "/system_state", 10)
        self.error_code_publisher = self.create_publisher(Int32MultiArray, "/error_codes", 10)
        self.stack_light_publisher = self.create_publisher(Int32, self.config["stack_light_topic"], 10)

        self.access_control_state = None
        self.open_door_requested = False
        
        def set_access_control_state(msg):
            self.access_control_state = AccessControlState(msg.data)
        self.access_control_subscriber = self.create_subscription(Int32, self.config["access_control"]["state_topic"], set_access_control_state, 10, callback_group=self.states_callback_group)
        self.emergency_state_topic = None        
        def get_emergency_state(msg):
            self.emergency_state_topic = msg.data
        self.access_control_subscriber = self.create_subscription(Int8, "/emergency/state", get_emergency_state, 10, callback_group=self.states_callback_group)
        
        self.open_door_called = False
        self.capto_client = CaptoApiClient("http://localhost:5000/api")
        self.last_time_state_saved = time.time()
        self.service_callback_group = ReentrantCallbackGroup()

        self.open_door_client = self.create_client(BoolSrv, self.config["access_control"]["open_door_service"], callback_group=self.service_callback_group)

        self.cam_subscriptions_callbacks = ReentrantCallbackGroup()
        self.robot_subscriptions_callbacks = ReentrantCallbackGroup()
        self.emergency_subscriptions_callbacks = ReentrantCallbackGroup()
        self.subscriptions_callbacks = ReentrantCallbackGroup()

        for cam, cam_state_topic in self.config["cams"].items():
            self.cam_states[cam] = None
            self.cam_subscribers[cam] = self.create_subscription(Int32, cam_state_topic, lambda msg, cam=cam: self.set_cam_state(msg, cam=cam), 10, callback_group=self.cam_subscriptions_callbacks)
        

        def set_capto_state(msg):
            self.capto_state = GeneralState(msg.data)
            self.timout_times["capto"] = time.time()
        self.capto_state_subscriber = self.create_subscription(Int32, "/capto/system/state", set_capto_state, 10, callback_group=self.robot_subscriptions_callbacks)

        self.emergency_states = {}

        # The states of the relays (external and internal estop)
       
        time.sleep(1)
        self.run_timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.access_control_timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.02, self.run, callback_group=self.run_timer_callback_group)
        self.create_timer(0.5, self.access_control_monitor, callback_group=self.access_control_timer_callback_group)
        self.door_open_request_called = False

        self.gripper_check_state_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.5, self.gripper_check_state_cb, callback_group=self.gripper_check_state_cb_group)
        self.gripper_state_publish = self.create_publisher(Int32, "/logimat/gripper_state", 10)

        self.requested_to_pause_operation = False
        service_callback = ReentrantCallbackGroup()
        self.pause_system_service = self.create_service(BoolSrv, "/logimat/pause_system", self.pause_system_service_cb, callback_group=service_callback)
        self.resume_system_service = self.create_service(BoolSrv, "/logimat/resume_system", self.resume_system_service_cb, callback_group=service_callback)
        self.start_system_service = self.create_service(BoolSrv, "/logimat/start_system", self.start_system_service_cb, callback_group=service_callback)
        self.stop_system_service = self.create_service(BoolSrv, "/logimat/stop_system", self.stop_system_service_cb, callback_group=service_callback)

        self.system_state = SystemState.PAUSED
        logger.info("System state initialized")

    def activate_air(self, msg):
        if not msg.data:
            self.air_publisher.publish(Bool(data=True))
   
    def main_button_cb(self, msg):
        if self.main_button_state and not msg.data:
            self.should_run = True
        self.main_button_state = msg.data

    def should_run_cb(self, msg):
        self.should_run = msg.data
    
    def resume_system_service_cb(self, request, response):
        if not self.should_run:
            self.should_run = True
        return response
        
    def stop_system_service_cb(self, request, response):
        self.should_run = False
        return response
       
    def start_system_service_cb(self, request, response):
       
        if not self.should_run:
            self.should_run = True
        #logger.error(f" THIS IS THE RESUME CALL: {self.requested_to_pause_operation} | {self.should_run}")
        return response
        
    def pause_system_service_cb(self, request, response):
        #self.should_run = False
        self.should_run = False
        #logger.error(f" THIS IS THE PAUSE CALL: {self.requested_to_pause_operation} | {self.should_run}")
        return response
 
    def set_emergency_state(self, msg, emergency):
        #logger.info(f"EMERGENCZZ is { msg.data} and {emergency}")
        self.emergency_states[emergency] = msg.data

    def set_emergency_button_state(self, msg, emergency_button):
        self.emergency_button_states[emergency_button] = msg.data
    
    def is_emergency(self):
        if any(self.emergency_states.values()):
            return True
        if self.robot_state is not None:
            if self.robot_state.e_stopped.val:
                return True
        return False

    def gripper_check_state_cb(self):
        update = self.gripper_subs(blocking=False)
        for _ in range(5):
            time.sleep(0.2)
            update = self.gripper_subs(blocking=False)
            if update is not None:
                break
        if update is None:
            self.gripper_state = GeneralState.ERROR
        else:
            self.gripper_state = GeneralState.OPERATIONAL
        self.gripper_state_publish.publish(Int32(data=self.gripper_state.value))

    def set_cam_state(self, msg, cam):
        self.cam_states[cam] = GeneralState(msg.data)
        self.timout_times[cam] = time.time()

    def _check_header_timeout(self, header, timeout=20):
        current_time = self.get_clock().now().to_msg()
        if (current_time.sec - header.stamp.sec) > timeout:
            return True
        return False

    def check_robot_state(self):
        if self.robot_state is None:
            return False
        if self._check_header_timeout(self.robot_state.header, 5):
            return False
        if self.robot_state.e_stopped.val:
            return False
        return True
        
    def check_system_state(self):
        error_codes = []
        is_fault = False
        if self.is_emergency():
            self.should_run = False
            return StackLightMode.EMERGENCY, SystemState.EMERGENCY

        if self.access_control_state != AccessControlState.CLOSED:
            if self.access_control_state != AccessControlState.REQUEST_OPEN:
                return StackLightMode.FAULT, SystemState.PAUSED

        if self.capto_state != GeneralState.OPERATIONAL or (time.time() -  self.timout_times["capto"]) > 1:
            logger.info("CAPTO NOT OPERATIONAL")
            is_fault = True

        if self.robot_state is None:
            logger.info("ROBOT STATE IS NONE")
            is_fault=True
        else:
            if self._check_header_timeout(self.robot_state.header, 5):
                is_fault=True

        for cam in self.cam_states.values():
            if cam != GeneralState.OPERATIONAL:
                is_fault=True
                logger.info("CAMERA IS NOT OPERATIONAL")

        if is_fault:
            self.should_run = False
            return StackLightMode.FAULT, SystemState.FAULT
        

        system_state = SystemState.READY
        mode = StackLightMode.NORMAL
        if not self.should_run:
            mode = StackLightMode.WAITING_FOR_RESTART
            system_state = SystemState.PAUSED
        return mode, system_state

    def set_e_stop_reset_light(self, on: bool):
        self.e_stop_reset_light_pub.publish(Bool(data=on))
        self.e_stop_reset_light_on = on
        
    def blink_e_stop_reset_light(self):
        should_blink = True
        for state in self.emergency_button_states:
            if self.emergency_button_states[state]:
                should_blink = False
        if should_blink:
            if time.time() - self.last_time_blink_e_stop_reset_light > 1.0:
                self.set_e_stop_reset_light(not self.e_stop_reset_light_on)
                self.last_time_blink_e_stop_reset_light = time.time()
        else:
            self.set_e_stop_reset_light(False)
        
    def run(self):
        stack_light_state, system_state = self.check_system_state()
        self.save_state(stack_light_state, system_state)
        self.system_state = system_state
        self.stack_light_state = stack_light_state
        self.system_state_publisher.publish(Int32(data=self.system_state.value))
        self.stack_light_publisher.publish(Int32(data=stack_light_state.value))

        if self.is_emergency():
           logger.error(f"THIS IS AN EMERGENCY - {self.system_state}")
        else:
           # logger.error(f"THIS IS NOOOOOOOOOOOOOT  AN EMERGENCY - {self.system_state}")
            self.set_e_stop_reset_light(False)
        
    def save_state(self, stack_light_state, system_state):
        current_time = time.time()
        if current_time - self.last_time_state_saved < 30:
            return
        self.last_time_state_saved = current_time

        state = {
            "stack_light": stack_light_state.name,
            "system_state": system_state.name
        }
        try:
            self.capto_client.insert_state(state)
        except Exception as e:
            print(str(e))

    
    def access_control_monitor(self):
        if self.access_control_state == AccessControlState.REQUEST_OPEN:
            if self.picking_state is not None:
                self.should_run = False
                if self.picking_state == PickingState.IDLE:
                    request = BoolSrv.Request()
                    self.open_door_client.call(request)
                    self.door_open_request_called = True
                    return
            return


class SystemStateMonitorClient(GeneralServiceClient):
    def __init__(self, node, cb_group=None):
        super().__init__(node, cb_group)
        self.create_client(BoolSrv, "/logimat/pause_system", cb_group)
        self.create_client(BoolSrv, "/logimat/resume_system", cb_group)
        self.create_client(BoolSrv, "/logimat/start_system", cb_group)
        self.create_client(BoolSrv, "/logimat/stop_system", cb_group)

    def pause_system(self):
        request = BoolSrv.Request()
        self.call_service("/logimat/pause_system", request)
    
    async def pause_system_async(self):
        request = BoolSrv.Request()
        await self.call_service_async("/logimat/pause_system", request)

    def resume_system(self):
        request = BoolSrv.Request()
        self.call_service("/logimat/resume_system", request)

    async def resume_system_async(self):
        request = BoolSrv.Request()
        await self.call_service_async("/logimat/resume_system", request)   

    def start_system(self):
        logger.info("STAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAART")
        request = BoolSrv.Request()
        self.call_service("/logimat/start_system", request)

    async def start_system_async(self):
        request = BoolSrv.Request()
        await self.call_service_async("/logimat/start_system", request)

    def stop_system(self):
        request = BoolSrv.Request()
        self.call_service("/logimat/stop_system", request)

    async def stop_system_async(self):
        request = BoolSrv.Request()
        await self.call_service_async("/logimat/stop_system", request)


def main():
    rclpy.init()
    config = json.load(open(CONFIG_FILE))
    node = SystemStateMonitor(config)
    executor = MultiThreadedExecutor(8)
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()