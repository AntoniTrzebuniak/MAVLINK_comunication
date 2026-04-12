import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import threading
import time
from Application.Services.MatekService import MatekService
from Application.Services.CameraService import CameraService 
from Application.configuration.config_loader import cfg
from Application.Logger.log_module import get_logger
from Application.Services.MissionService import MissionService


drone = MatekService(device = cfg.mav.device, baud = cfg.mav.baud)
camera = CameraService(drone=drone)
planner = MissionService(drone) 
logger = get_logger(__name__)
drone.set_mission_current_rate(10)


camera_thread = None

Mapping_started_flag = False
i=0
while True:
    print(f"iteration {i}")
    i+=1
    curr_wp = drone.get_mission_status()
    print(f"Current waypoint: {curr_wp}")
    

    if curr_wp == 5 and not Mapping_started_flag:
        Mapping_started_flag = True
        logger.info(f"Reached waypoint {curr_wp}, starting mapping")
        camera_thread = threading.Thread(
                target=camera.MappingListener, 
                daemon=True
                )
        camera_thread.start()

    if curr_wp >= 96:
        logger.info(f"Reached waypoint {curr_wp}, stopping mapping")
        camera.stop_event.set()     # zamykanie wątku
        camera_thread = None
        Mapping_started_flag = False
    

'''
def droppoint_pipeline():
    target1 = {"command": "WAYPOINT", "lat": 50.2844310, "lon": 19.7209096, "isBottle": True}
    drop_point1 = planner.calc_drop_coords(target1)


    new_mission = drone.append_waypoints(lap2)
    drone.set_mode("AUTO")
    if new_mission:
        logger.info(f"Added {len(lap2)} waypoints successfully: \n{lap2}")
    else:
        logger.error("Failed to add waypoints for target 1.")

'''

