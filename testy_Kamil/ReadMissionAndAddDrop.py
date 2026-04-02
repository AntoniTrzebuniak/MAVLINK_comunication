 # Initialize connection\n
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Initialize connection
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService
"""
from Application.calc_drop_translation import core_math
from Application import configuration
import math
"""


resolution = [1920, 1080]
drone = MatekService(device="tcp:172.20.10.2:5763")
drone_mission = MissionService(drone,resolution) 
print("Inicjalizacja programu")


waypoints = drone.get_mission()
#target = {"command": "WAYPOINT", "lat": -33.0000, "lon": 149.165230, "isBottle": True}

drop_point = {"command": "WAYPOINT", "lat": -35.3243695, "lon": 149.1483307, "alt": 20, "acr": 0}





#Założenie : waypointy zostają dodane w mission planerze. Wykonane zostaje jedno kółko w celu zidentyfikowania celu. Następnie 
# misja zostaje skopiowana i dodana na koniec obecnej misji wraz z sekwencją zrzutu.

#yaw = math.radians(cfg.drops.bottle.drop_course)

container = drone_mission.calc_drop_waypoints(
    drop_point=drop_point,
    yaw=10,
    container=[]
)
new_mission = drone.add_drop_sequence(container)
drone.append_waypoints(new_mission)
drone.set_mode("AUTO")

