from Application.configuration.config_loader import cfg     # mandatory
from Application.Logger.log_module import get_logger       
from Application.Services.MatekService import MatekService
from Application.Services.MissioinService import MissionService
import time


drone  = MatekService(device=cfg.mav.device, baud=cfg.mav.baud)
logger = get_logger(__name__)
planner = MissionService(drone)

mission = drone.get_mission()


drone.append_waypoints(mission[1:])

#print(drone.get_mission_status())
drop_wp = {"lat": 50.2847326, "lon": 19.7204590, "isBottle": True}

while True:
    curr_wp, total_wp = drone.get_mission_status()
    
    print(f"Current WP: {curr_wp}")
    print(f"Total WPs: {total_wp}")
    if curr_wp == 7:
        planner.add_drop_waypoints()
    time.sleep(1)
    
    