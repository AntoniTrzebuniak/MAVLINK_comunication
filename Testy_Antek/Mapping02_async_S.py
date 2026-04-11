import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import threading
import time
from Application.Services.MatekService import MatekService
from Application.Services.CameraService import CameraService 
from Application.configuration.config_loader import cfg
from Application.Logger.log_module import get_logger


drone = MatekService(device="tcp:192.168.0.186:5763")
camera = CameraService(drone=drone)
logger = get_logger(__name__)

camera_thread = None

Mapping_started_flag = False
i=0
while True:
    print(f"iteration {i}")
    i+=1
    curr_wp = drone.get_mission_status()
    print(f"Current waypoint: {curr_wp}")
    

    if curr_wp == 3 and not Mapping_started_flag:
        Mapping_started_flag = True
        logger.info(f"Reached waypoint {curr_wp}, starting mapping")
        camera_thread = threading.Thread(
                target=camera.Testing_function, 
                daemon=True
                )
        '''camera_thread = threading.Thread(
                target=camera.image_capture_listener, 
                daemon=True
                )'''
        camera_thread.start()

    if curr_wp >= 116:
        logger.info(f"Reached waypoint {curr_wp}, stopping mapping")
        camera.stop_event.set()     # zamykanie wątku
        camera_thread = None



