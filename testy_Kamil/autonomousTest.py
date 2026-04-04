 # Initialize connection\n
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# Initialize connection
from Application.Services.MatekService import MatekService
from Application.Services.MissionService import MissionService



drone = MatekService(device="tcp:172.20.10.2:5763")
drone_mission = MissionService(drone) 
print("Inicjalizacja programu")


waypoints = [

    {"command": "TAKEOFF", "alt": 100.0},
    {"command": "WAYPOINT", "lat": -35.35762620, "lon": 149.16008950, "alt": 100.0, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.35545620, "lon": 149.15519710, "alt": 100.0, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.35517620, "lon": 149.14987560, "alt": 100.0, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.35748620, "lon": 149.14558410, "alt": 100.0, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.36042610, "lon": 149.14369580, "alt": 100.0, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.36287600, "lon": 149.14403920, "alt": 100.0, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.36595570, "lon": 149.14403920, "alt": 100.0, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.36847540, "lon": 149.14506910, "alt": 100.0, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.37155490, "lon": 149.16875840, "alt": 100.0, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.37043510, "lon": 149.17442320, "alt": 100.0, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.36861540, "lon": 149.17922970, "alt": 100.0, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.35559620, "lon": 149.16386600, "alt": 100.0, "acr": 0}
]

droppoint = {"command": 16, "lat": -35.36511580, "lon": 149.17099000, "alt": 100.0, "acr": 0}

try:
        # Upload mission\n
        if drone.set_waypoints(waypoints):
            print("Mission uploaded successfully")
            # Start mission\n
            #if drone.start_mission():
               # print("Mission started - monitoring progress...")
                # Monitor for 60 seconds\n
                #for _ in range(30):
                   # drone.monitor_mission()
                   # time.sleep(2)
        else:
            print("Failed to upload mission")
except KeyboardInterrupt:
    print("\nMission monitoring stopped by user")
except Exception as e:
    print(f"Error: {e}")
    



mission = drone.get_mission()







