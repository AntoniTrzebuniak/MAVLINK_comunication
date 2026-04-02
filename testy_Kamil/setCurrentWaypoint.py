 # Initialize connection\n
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Initialize connection
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService




index = int(sys.argv[1])
resolution = [1920, 1080]
drone = MatekService(device="tcp:172.20.10.2:5763")
drone_mission = MissionService(drone,resolution) 
print("Inicjalizacja programu")


print(drone.set_current_waypoint(index))
if drone.set_current_waypoint(index)==False:
    print("Nie można ustawić aktualnego waypointa. Ponawiam próbę.")
    drone.set_current_waypoint(index)
    