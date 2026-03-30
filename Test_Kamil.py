 # Initialize connection\n
from Application.Services.MatekService import MatekService



drone = MatekService(device="tcp:192.168.0.186:5763")
print("Inicjalizacja programu")


# DZIAŁA
#drone.is_armed()
#drone.arm()
#drone.get_current_mode()
#drone.set_mode()


waypoints = [
    {"command": "WAYPOINT", "lat": -35.363261, "lon": -100.165230, "alt": 20, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.363500, "lon": -110.165500, "alt": 25, "acr": 0},
    {"command": "SET_SERVO", "channel": 5, "pwm": 1900},
]
append = [
    {"command": "WAYPOINT", "lat": -35.363800, "lon": 100.165800, "alt": 20, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.364000, "lon": 110.166000, "alt": 25, "acr": 0},
    {"command": "SET_SERVO", "channel": 5, "pwm": 1900},
]
prepend = [
    {"command": "WAYPOINT", "lat": -35.364200, "lon": 120.166200, "alt": 20, "acr": 0},
    {"command": "WAYPOINT", "lat": -35.364400, "lon": 130.166400, "alt": 25, "acr": 0},
]

drone.get_mission()
drone.set_waypoints(waypoints)
print("Mission uploaded successfully")

drone.get_mission()
print("get_mission() wykonane")

drone.append_waypoints(append)
print("append_waypoint() wykonane")
drone.get_mission()

drone.prepend_waypoints(prepend)
print("prepend_waypoint() wykonane")
drone.get_mission()

drone.set_current_waypoint(5)
print("set_current_waypoint() wykonane")

drone.add_drop_waypoint({"command": "WAYPOINT", "lat": -25.364600, "lon": 140.166600, "alt": 20, "acr": 0})
print("add_drop_waypoint() wykonane")

drone.get_mission()
