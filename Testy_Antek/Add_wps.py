 # Initialize connection\n
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Initialize connection
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService
from Application.configuration.config_loader import cfg
from Application.Logger.log_module import get_logger

"""
from Application.calc_drop_translation import core_math
from Application import configuration
import math
"""


drone = MatekService(device="tcp:192.168.0.111:5763")
planner = MissionService(drone) 
print("Inicjalizacja programu")
logger = get_logger(__name__)

waypoints = drone.get_mission()

target = {"command": "WAYPOINT", "lat": 50.2843899, "lon": 19.7213602, "alt": 20, "acr": 0}
drop_point = planner.calc_drop_coords(target)

#Założenie : waypointy zostają dodane w mission planerze. Wykonane zostaje jedno kółko w celu zidentyfikowania celu. Następnie 
# misja zostaje skopiowana i dodana na koniec obecnej misji wraz z sekwencją zrzutu.

#yaw = math.radians(cfg.drops.bottle.drop_course)

flag1 = False
while True:
    idx = drone.get_mission_status()
    logger.info(f"Current WP: {idx}")
    if idx == 5 and not flag1:
        flag1 = True
        container = planner.calc_drop_waypoints(
            drop_point=drop_point,
            yaw=10,
            container=[]
        )

        new_mission = drone.add_drop_sequence(container)
        drone.append_waypoints(new_mission)
        drone.set_mode("AUTO")

s