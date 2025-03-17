import requests
from typing import Dict
from thermofischer_interface.logger import SereactLogger
import os
import time
from thermofischer_interface.utils import generate_uuid
logger = SereactLogger(__name__)
TIMEOUT = int(os.environ.get("SEREACT_CAPTO_API_TIMEOUT", "120"))

class RobotException(Exception):
    pass

class CaptoApiClient:
    def __init__(self, uri):
        self.uri = uri

    def call_api(self, method, params):
        data = {
            "jsonrpc": "2.0",
            "method": method,
            "params": params,
            "id": 1
        }
        response = requests.post(self.base_url, json=data)
        return response.json()
    
    def _request(self, msg: Dict) -> Dict:
        res = requests.post(self.uri, json=msg, timeout=TIMEOUT)
        return res.json()

    def request(self, method: str, params: Dict = {}) -> Dict:
        """
        Make a request to the Sereact API

        Args:
            method (str): Method to call
            params (Dict, optional): Parameters to pass to the method. Defaults to {}.
        """
        logger.info(f"Requesting method: {method} with params: {params}")
        start_time = time.time()
        request_result = self._request({
            "jsonrpc": "2.0",
            "method": method,
            "params": params,
            "id": 1,
        })
        if request_result.get("error", None) is not None:
            logger.error(f"{request_result['error']['message']}. {request_result['error']['data']['title']}. {request_result['error']['data']['description']}")
            raise RobotException(request_result["error"]["data"]["description"])
        else:
            result = request_result["result"]
            logger.info(f"Result: {result} in {time.time() - start_time} seconds")
            #if result.get("success", False) is False:
            #logger.error(f"Failure in handling method: {method}")
            return result

    def grasp(self, 
              no_items: int = 1, 
              speed: int=50, 
              pick_area: str="pick_1", 
              place_area: str="place_1", 
              pick_compartment: Dict= None,  
              pack_compartment: Dict=None, 
              metadata: Dict={}, 
              check_obj_fits: bool= True, 
              check_bin_full: bool = False, 
              max_retries: int = 0,
              additional_inside_height_pick: float = 0.25,
              additional_inside_height_place: float = 0.2,
              overwrite_grasp_prio: dict = None,
              threshold_height: float=1.0,
              threshold_percentage: float=0.05, 
              next_grasp_uuid = None) -> Dict:
        """
        Grasp items from pick area and place them in place area

        Args:
            no_items (int, optional): Number of items to grasp. Defaults to 1.
            speed (int, optional): Speed of the robot. Defaults to 50.
            pick_area (str, optional): Pick area. Defaults to "pick_1".
            place_area (str, optional): Place area. Defaults to "place_1".
            check_obj_fits (str, optional): Check if the object fits

        """
        kwargs = {
            "no_items": no_items,
            "speed": speed,
            "pick_area": pick_area,
            "place_area": place_area,
            "pick_compartment": pick_compartment,
            "pack_compartment": pack_compartment,
            "check_obj_fits": check_obj_fits,
            "check_bin_full": check_bin_full,
            "max_retries": max_retries,
            "metadata": metadata,
            "additional_inside_height_pick": additional_inside_height_pick,
            "additional_inside_height_place": additional_inside_height_place,
            "soft_place_back_if_not_fit": True,
            "overwrite_grasp_prio": overwrite_grasp_prio,
            "threshold_percentage": threshold_percentage,
            "threshold_height": threshold_height,
            "only_pick_fit_object": True,
            "next_grasp_uuid": next_grasp_uuid
        }
        result = self.request("robot.grasp", kwargs)["payload"]["picks"][0]
        return result

    def pick_only(self, 
              no_items: int = 1, 
              speed: int=10, 
              pick_area: str="pick_1", 
              pick_compartment: Dict= None,  
              metadata: Dict={}, 
              max_retries: int = 0,
              additional_inside_height_pick: float = 0.25,
              check_obj_fits: bool= True,
              soft_place_back_if_not_fit: bool = True,
              only_pick_fit_object: bool = True,
              overwrite_grasp_prio: dict = None,
              place_area="place_1",
              next_grasp_uuid: str = None,
              robot_cycle_id: str = generate_uuid(),
              ) -> Dict:

        kwargs = {
            "no_items": no_items,
            "speed": speed,
            "pick_area": pick_area,
            "pick_compartment": pick_compartment,
            "max_retries": max_retries,
            "metadata": metadata,
            "place_area": place_area,
            "additional_inside_height_pick": additional_inside_height_pick,
            "overwrite_grasp_prio": overwrite_grasp_prio,
            "check_obj_fits": check_obj_fits,
            "only_pick_fit_object": only_pick_fit_object,
            "soft_place_back_if_not_fit": soft_place_back_if_not_fit,
            "next_grasp_uuid": next_grasp_uuid,
            "robot_cycle_id": robot_cycle_id,
            "max_object_size": {
                "width": 0.25,
                "height": 0.25,
                "depth": 0.25
            }
        }
        result = self.request("robot.grasp_only", kwargs)["payload"]
        return result


    def grasp_parallel(self, 
              no_items: int = 1, 
              pick_area: str="pick_1", 
              pick_compartment: Dict= None,  
              metadata: Dict={}, 
              additional_inside_height_pick: float = 0.25,
              overwrite_grasp_prio: dict = None) -> Dict:

        kwargs = {
            "no_items": no_items,
            "pick_area": pick_area,
            "pick_compartment": pick_compartment,
            "metadata": metadata,
            "additional_inside_height_pick": additional_inside_height_pick,
            "overwrite_grasp_prio": overwrite_grasp_prio,
        }
        result = self.request("vision.grasps_parallel", kwargs)["payload"]
        return result

    def place_only(self, 
              speed: int=10, 
              place_area: str="place_1", 
              metadata: Dict={}, 
              max_retries: int = 0,
              additional_inside_height_place: float = 0.2,
              overwrite_grasp_prio: dict = None,
              robot_cycle_id: str = generate_uuid()
              ) -> Dict:

        kwargs = {
            "speed": speed,
            "place_area": place_area,
            "max_retries": max_retries,
            "metadata": metadata,
            "additional_inside_height_place": additional_inside_height_place,
            "overwrite_grasp_prio": overwrite_grasp_prio,
            "get_next_grasp": True,
            "robot_cycle_id": robot_cycle_id
        }
        result = self.request("robot.place_only", kwargs)["payload"]
        return result
    
    def scan_only(self, speed: int= 10, scanner_keys: list[str]=None, possible_barcodes: list[str]=[], cut_to: int=13):
        # TODO implement this shit
        kwargs = {
            "speed": speed,
            "scanner_keys": scanner_keys,
            "possible_barcodes": possible_barcodes,
            "no_path_on_grid": True,
            "cut_to": cut_to
        }
        result = self.request("robot.scan_only", kwargs)["payload"]
        return result
    

    def check_item_size(self, area):
        kwargs = {
            "area": area,
            "check_for_item_outside_floor": True
        }
        result = self.request("vision.object_sizes_recompute_height_from_bin", kwargs)["payload"]
        return result
        

    def homing(self, speed=None) -> None:
        """
        Home the robot
        """
        return self.request("robot.homing", {"speed": speed})
    
    def stop(self) -> None:
        """
        Stop the robot
        """
        return self.request("robot.stop")
    
    def hard_stop(self) -> None:
        """
        Hard stop the robot
        """
        return self.request("robot.hard_stop")
    
    def count_items(self, area: str = "pick_1", next_grasp_uuid: str = None) -> int:
        """
        Count items in area

        Args:
            area (str, optional): Area to count items in. Defaults to "pick_1".
        """
        return self.request("vision.count_items", {"area": area, "next_grasp_uuid": next_grasp_uuid})["payload"]
    
    def check_empty(self, area: str = "pick_1") -> bool:
        return self.count_items(area) == 0
    
    def get_barcode(self, area: str = "place_1") -> str:
        """
        Get barcode from area

        Args:
            area (str, optional): Area to get barcode from. Defaults to "pick_1".
        """
        return self.request("vision.get_barcode", {"area": area})["payload"]

    def check_bin_full(self, area="pick_1",
                threshold_height: float=1.0,
              threshold_percentage: float=0.05):
        """
        Check if bin is full
        """
        return self.request(method = "vision.bin_full", params= {"area": area, "threshold_height": threshold_height,
              "threshold_percentage": threshold_percentage})["payload"]
    
    def get_barcode(self, area="pick_1"):
        """
        Get barcode from area
        """
        return self.request("vision.get_barcode", {"area": area})["payload"]

    def upload_blob(self, json_data, area=None):
        json_data["customer"] = "bol"
        kwargs = {}
        kwargs["json_data"] = json_data
        if area is not None:
            kwargs["area"] = area

        return self.request("data.upload_json", kwargs)
    
    def insert_timeseries(self, collection, metadata, timestamp=None):
        kwargs = {
            "collection": collection,
            "metadata": metadata,
            "timestamp": timestamp
        }
        return self.request("data.insert_timeseries", kwargs)
    
    def insert_state(self, metadata):
        return self.insert_timeseries("states", metadata=metadata)

    def get_object_sizes(self, area="pick_1"):
        return self.request("vision.get_object_sizes", {"area": area})["payload"]
    
    def clear_failed_grasps(self):
        return self.request("robot.clear_failed_grasps")["payload"]
