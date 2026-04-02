



 



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



print(drone.get_current_mode())

waypoints = [
    {"command": "WAYPOINT", "lat": -35.363261, "lon": -100.165230, "alt": 20, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.363500, "lon": -110.165500, "alt": 25, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.363500, "lon": -110.165500, "alt": 25, "acr": 0},
]
append = [
    {"command": "WAYPOINT", "lat": -35.363800, "lon": 100.165800, "alt": 20, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.364000, "lon": 110.166000, "alt": 25, "acr": 0},

]
prepend = [
    {"command": "WAYPOINT", "lat": -35.364200, "lon": 120.166200, "alt": 20, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.364400, "lon": 130.166400, "alt": 25, "acr": 0},
]

droppoint = {"command": "WAYPOINT", "lat": -33.0000, "lon": 149.165230, "alt": 20, "acr": 0}

#prosta implementacja dodania zrzutu do obecnej misji

mission = drone.get_mission()
print("Mission read successfully")
cur_waypoint = drone.get_mission_status()



container = drone_mission.calc_drop_waypoints(droppoint, yaw=10, container=[], isRed=True)
new_mission = drone.add_drop_sequence(container)
drone.append_waypoints(new_mission)
#drone.set_current_waypoint(cur_waypoint)
#drone.set_mode("AUTO")





