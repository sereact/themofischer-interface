import asyncio
from inspect import stack
import json
import os
import re
from urllib import response

from aiohttp import web
import aiohttp
import aiohttp_cors 
import rclpy
import os
from datetime import datetime
from thermofischer_interface.logger import SereactLogger

from thermofischer_interface.async_ros_client import AsyncRosClient, ThermoFischerStacklightMode
from thermofischer_interface.logger import SereactLogger

import time
logger = SereactLogger(__name__, ros_log=False)

PORT = os.environ.get("SEREACT_PORT", 8080)
CWD = os.path.dirname(os.path.abspath(__file__))
MAIN_LOOP_INTERVAL = int(os.environ.get("SEREACT_MAIN_LOOP_INTERVAL", "1"))

class Server:
    def __init__(self, host='0.0.0.0', port=8082):
        self.host = host
        self.port = port
        self.loop = asyncio.get_event_loop()
        self.setup_app()
        self.ws_clients = set()
        self.ros_client = AsyncRosClient(self.loop)
        self.loop.create_task(self.main_loop())
        print(f"The Web Server has been started ------------ PORT: {self.port}")

    
    async def get_api(self, request):
        path = request.path
        path = path.split("/")[1]
        pass

    async def post_api(self, request):
        pass



    def setup_cors(self):
        """Configure CORS on all routes in the web application."""
        cors = aiohttp_cors.setup(self.app, defaults={
            "*": aiohttp_cors.ResourceOptions(
                allow_credentials=True,
                expose_headers="*",
                allow_headers="*",
            )
        })

        # Configure CORS on all routes.
        for route in list(self.app.router.routes()):
            cors.add(route)



    async def main_loop(self):
        while True:
            # if not self.api.ros_client.is_system_ready():
            #     await self.publish_error_code()
            await asyncio.sleep(MAIN_LOOP_INTERVAL)



    def setup_app(self):
        self.app = web.Application()
        self.app.router.add_post('/lens_response', self.lens_response)
        self.setup_cors()

    async def check_item_state(self, request):
        stack_light_mode = ThermoFischerStacklightMode.UNKNOWN
        response = request.get("response", None)
        if not isinstance(response, dict):
            return stack_light_mode

        bins = response.get("bins", [])
        if not isinstance(bins, list):
            return stack_light_mode
        
        if len(bins) == 0:
            return stack_light_mode
        
        first_bin = bins[0]
        if not isinstance(first_bin, dict):
            return stack_light_mode

        item_state = first_bin.get("extra_schema", {}).get("device_status", None)
        if item_state is None:
            return stack_light_mode

        if item_state == "ON":
            stack_light_mode = ThermoFischerStacklightMode.ON
        elif item_state == "NOT ON":
            stack_light_mode = ThermoFischerStacklightMode.NOT_ON
        return stack_light_mode



    async def lens_response(self, request):
        data = await request.json()
        stack_light_mode = await self.check_item_state(data)
        await self.ros_client.set_stack_light(stack_light_mode)
        logger.info(data)
        
        self.loop.create_task(self.delayed_light_off())
        
        return web.json_response({})
    
    async def delayed_light_off(self):
        """Turn off the light after 3 seconds"""
        await asyncio.sleep(3)
        await self.ros_client.set_stack_light(ThermoFischerStacklightMode.OFF)

    def main(self):
        runner = aiohttp.web.AppRunner(self.app)
        self.loop.run_until_complete(runner.setup())
        site = aiohttp.web.TCPSite(runner, host="0.0.0.0", port=self.port)
        self.loop.run_until_complete(site.start())
        try:
            import uvloop
            uvloop.install()
        except ImportError:
            pass
        self.loop.run_forever()


def main():
    rclpy.init()
    server = Server(port=PORT)
    server.main()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

