import asyncio
import time
from thermofischer_interface.error_codes import JsonRpcError
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

ROBOT_STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
class GeneralServiceClient:

    def __init__(self, node, cb_group=None) -> None:
        self.node = node
        self.clients = {}
        self.cb_group = cb_group

    def create_client(self, service_type, service_name, cb_group=None):
        if cb_group is None:
            cb_group = self.cb_group
        self.clients[service_name] = self.node.create_client(
            service_type, service_name, callback_group=self.cb_group
        )

    def wait_for_service(self, service_name, timeout_sec=1.0):
        return self.clients[service_name].wait_for_service(timeout_sec)

    def call_service(self, service_name, request, sleep_time=0.001):
        future = self.clients[service_name].call_async(request)
        while not future.done():
            time.sleep(sleep_time)
        result = future.result()
        if result.raise_exception:
            raise Exception(result.exception_msg)
        return future.result()

    async def  call_service_async(self, service_name, request, sleep_time=0.001):
        future = self.clients[service_name].call_async(request)
        while not future.done():
            await asyncio.sleep(sleep_time)
        result = future.result()
        if result.raise_exception:
            if hasattr(result, "error_code") and result.error_code!=0:
                raise JsonRpcError(result.error_code)
            raise Exception(result.exception_msg)
        result = future.result()
        return result
