from MissionService import MissionService
from PixHawk.communication import PixHawkService
from LiveDetectionModes import LiveDetection
import math

ld = LiveDetection()
drone = PixHawkService()
resolution = ld.getResolution()
mission = MissionService(drone, (1280, 720))

while True:
    f,c,is_red = ld.detect_shape()
    i=1
    if f == 0:
       i+=1
    else:
        print(f"Detected shape with {f} sides at {c}") 
        if mission.process_target(c, is_red):
            print("dodano cel")
        i+=1
    if not i % 30:
        seq, total = drone.get_mission_status()
        if seq >= total-1:
            break


yaw_EAST = math.radians(96)
yaw_WEST = math.radians(96-180)
mission.calc_drop_waypoints(mission.TRG_CANDIDATES[0], yaw_EAST, mission.PATH1)
mission.calc_drop_waypoints(mission.TRG_CANDIDATES[0], yaw_WEST, mission.PATH2)
#tutaj funkcja której njie ma

mission.prepare_wps(mission.PATH1, mission.PATH2)
mission.calc_drop_waypoints(mission.NEW_MISSION)

    