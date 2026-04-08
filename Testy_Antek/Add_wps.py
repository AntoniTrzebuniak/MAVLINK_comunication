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

drone = MatekService(device="tcp:192.168.0.109:5763")
planner = MissionService(drone) 
print("Inicjalizacja programu")
logger = get_logger(__name__)
waypoints = drone.get_mission()
poligon = planner.load_Poly(cfg.dirs.zones_dir / "Sloszowa_target_detection_1.poly")
if poligon is None:
    logger.error("Nie można wczytać poligonu. Sprawdź ścieżkę i format pliku.")

# ++++++++++++++ 1 cel ++++++++++++++
target1 = {"command": "WAYPOINT", "lat": 50.2844310, "lon": 19.7209096, "isBottle": True}
drop_point1 = planner.calc_drop_coords(target1)
lap2 = []
lap2.append({"command": "WAYPOINT", "lat": 50.2853702, "lon": 19.7223687, "alt": 80, "acr": 15})
lap2.append({"command": "WAYPOINT", "lat": 50.2822749, "lon": 19.7217625, "alt": 60, "acr": 15})
lap2.append({"command": "WAYPOINT", "lat": 50.2825903, "lon": 19.7204214, "alt": 60, "acr": 15})
b1 = planner.isinPolygon(target1['lat'], target1['lon'], poligon)
if b1 == False:
    logger.warning("cel poza strefą")

# ++++++++++++++ 2 cel ++++++++++++++
target2 = {"command": "WAYPOINT", "lat": 50.2847943, "lon": 19.7201586, "isBottle": False}
drop_point2 = planner.calc_drop_coords(target2)

lap3 = []
lap3.append({"command": "WAYPOINT", "lat": 50.2853702, "lon": 19.7223687, "alt": 60, "acr": 15})
lap3.append({"command": "WAYPOINT", "lat": 50.2822749, "lon": 19.7217625, "alt": 60, "acr": 15})
lap3.append({"command": "WAYPOINT", "lat": 50.2825903, "lon": 19.7204214, "alt": 60, "acr": 15})
b1 = planner.isinPolygon(target2['lat'], target2['lon'], poligon)
if b1 == False:
    logger.warning("cel poza strefą")


flag1 = False
flag2 = False

while True:
    idx = drone.get_mission_status()
    logger.info(f"Current WP: {idx}")

    if idx == 5 and not flag1:
        flag1 = True

        lap2 = planner.calc_drop_waypoints(
            drop_point=drop_point1,
            yaw=10,
            container=lap2
        )
        lap2.append({"command": "WAYPOINT", "lat": 50.2855004, "lon": 19.7208774, "alt": 60, "acr": 15})
        new_mission = drone.append_waypoints(lap2)
        drone.set_mode("AUTO")
        if new_mission:
            logger.info(f"Added {len(lap2)} waypoints successfully: \n{lap2}")
        else:
            logger.error("Failed to add waypoints for target 1.")


    if idx == 13 and not flag2:
        flag2 = True

        lap3 = planner.calc_drop_waypoints(
            drop_point=drop_point2,
            yaw=10,
            container=lap3
        )
        lap3.append({"command": "WAYPOINT", "lat": 50.2855004, "lon": 19.7208774, "alt": 60, "acr": 15})
        new_mission = drone.append_waypoints(lap3)
        drone.set_mode("AUTO")
        if new_mission:
            logger.info(f"Added {len(lap3)} waypoints successfully: \n{lap3}")
        else:
            logger.error("Failed to add waypoints for target 2.")

