 # Initialize connection\n
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Initialize connection
from Application.Services.MatekService import MatekService
from Application.Services.CameraService import CameraService 
from Application.configuration.config_loader import cfg
from Application.Logger.log_module import get_logger

drone = MatekService(device="tcp:192.168.0.109:5763")
#camera = CameraService(drone=drone)
logger = get_logger(__name__)

###### usunąc przed lotem
import csv
import time
from datetime import datetime
PHOTOS_DIR = cfg.dirs.photos_dir
PHOTOS_MISSION_DIR = PHOTOS_DIR / f"mission_{datetime.now().strftime('Mission_%Y-%m-%d_%H-%M')}"
os.makedirs(PHOTOS_MISSION_DIR, exist_ok=True)
LOG_FILE = PHOTOS_MISSION_DIR / 'photos_position.csv'

drone.set_mission_current_rate(10)


Mapping_started_flag = False
while True:
    curr_wp = drone.get_mission_status()
    print(f"Current waypoint: {curr_wp}")

    if curr_wp == 5 and not Mapping_started_flag:
        Mapping_started_flag = True
        logger.info(f"Reached waypoint {curr_wp}, starting mapping")

        #log_file = open(camera.LOG_FILE, 'a', newline='')
        log_file = open(LOG_FILE, 'a', newline='')
        while True:
            msg = drone.master.recv_match(type='CAMERA_FEEDBACK', blocking=True)   # 'TERRAIN_REPORT'
            if msg:
                print(f"Received CAMERA_FEEDBACK message: img_idx={msg.img_idx}")
                CameraService.image_capture_test(log_file, msg)









