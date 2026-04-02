# Initialize connection\n
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Initialize connection
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService



resolution = [1920, 1080]
drone = MatekService(device="tcp:172.20.10.2:5763")
drone_mission = MissionService(drone,resolution) 
print("Inicjalizacja programu")

drone.get_mission()
print("Current mode: ", drone.get_current_mode())
print("current waypoint: ", drone.get_mission_status())
print("Attitude: ", drone.get_attitude())
