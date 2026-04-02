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


waypoints = drone.get_mission()
droppoint = [
    {"command": "WAYPOINT", "lat": -35.363800, "lon": 100.165800, "alt": 20, "acr": 0},
]


#Założenie : waypointy zostają dodane w mission planerze. Wykonane zostaje jedno kółko w celu zidentyfikowania celu. Następnie 
# misja zostaje skopiowana i dodana na koniec obecnej misji wraz z sekwencją zrzutu.
container = drone_mission.calc_drop_waypoints(droppoint, yaw=10, container=[], isRed=True)
new_mission = drone.add_drop_sequence(container)
drone.append_waypoints(new_mission)

