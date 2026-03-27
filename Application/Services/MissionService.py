from __future__ import annotations
from Application.Services.MatekService import MatekService
import numpy as np
from pymavlink import mavutil, mavextra
from typing import Tuple, List, Dict, Optional
import cv2

class MissionService:
    """
    Args:
        drone: MatekService object instance
        resolution: (width, height)
    """
    def __init__(self, drone: MatekService, resolution: tuple):
        self.logger = get_logger(__name__)
        self.image_width, self.image_height = resolution
        self.GEOFENCE = [
            (40.7352710, 30.0884038),
            (40.7339580, 30.0882322),
            (40.7341816, 30.0856787),
            (40.7354823, 30.0858986)
        ]
        self.POLES = [(),()]
        self.TRG_CANDIDATES=[] #{"lat": lat, "lon": lon, "count": 1, "isRed": isRed}
        self.NEW_MISSION=[{"lat": 40.7342403, "lon": 30.0897108, "alt": 55.75, "acr": 16},
                            {"lat": 40.0, "lon": 0.0, "alt": 40.0, "acr": 22},
                            {"lat": 40.7347669, "lon": 30.0849867, "alt": 40.0, "acr": 16},
                            {"lat": 40.7353604, "lon": 30.0851315, "alt": 40.0, "acr": 16},
                            {"lat": 40.7350189, "lon": 30.0889564, "alt": 40.0, "acr": 16},
                            {"lat": 40.7344295, "lon": 30.0888866, "alt": 40.0, "acr": 16},
                            {"lat": 40.7347669, "lon": 30.0849867, "alt": 40.0, "acr": 16},
                            {"lat": 40.7353604, "lon": 30.0851315, "alt": 40.0, "acr": 16},
                            {"lat": 40.7350189, "lon": 30.0889564, "alt": 40.0, "acr": 16},
                            {"lat": 40.7344295, "lon": 30.0888866, "alt": 40.0, "acr": 16},
                            {"lat": 40.7347669, "lon": 30.0849867, "alt": 40.0, "acr": 16},
                            {"lat": 40.7353604, "lon": 30.0851315, "alt": 40.0, "acr": 16},
                            {"lat": 40.7350189, "lon": 30.0889564, "alt": 40.0, "acr": 16},
                            {"lat": 40.7344295, "lon": 30.0888866, "alt": 40.0, "acr": 16},
                            {"lat": 40.7347669, "lon": 30.0849867, "alt": 40.0, "acr": 16},
                            {"lat": 40.7353604, "lon": 30.0851315, "alt": 40.0, "acr": 16},
                            {"lat": 40.7350189, "lon": 30.0889564, "alt": 40.0, "acr": 16},
                            {"lat": 40.7344295, "lon": 30.0888866, "alt": 40.0, "acr": 16}]
        self.drone = drone
        self.PATH1 = []
        self.PATH2 = []

    def setResolution(self, resolution):
        w,h = resolution
        self.image_width = w
        self.image_height = h
        

    K = np.array([
            [1773.9244383143994, 0.00000000e+00, 1152],
            [0.00000000e+00, 1733.1547280645818, 648],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ], dtype=np.float32)
    dist = np.array([[-0.1359832, 0.36278877, 0.02114295, 0.02506856, -0.21990155]], dtype=np.float32)

    
    def rot_matrix(self, roll, pitch, yaw):
        Rx = np.array([[ np.cos(roll), 0, -np.sin(roll)],
                    [0, 1, 0],
                    [np.sin(roll), 0, np.cos(roll)]])
        Ry = np.array([[1, 0, 0],
                    [0, np.cos(pitch), np.sin(pitch)],
                    [0, -np.sin(pitch),  np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), np.sin(yaw), 0],
                    [-np.sin(yaw),  np.cos(yaw), 0],
                    [0, 0, 1]])
        return Rz @ Ry @ Rx

    def calculate_target_cords(self, pixel: tuple)->tuple:
        """
        Oblicza współrzędne geograficzne punktu na ziemi
        odpowiadającego pikselowi (u,v) z obrazu kamery
        zamontowanej pionowo w dół (nadir).
        Args:
            pixel: (width, height)
        Returns:
            (lat, lon)
        """
        # pobierz rozdzielczość obrazu
        
        u, v = pixel

        # dane z autopilota
        msg_gps = self.drone.get_current_coordinates()
        if msg_gps is None:
            return None
        lat_uav, lon_uav, alt_uav = msg_gps
        msg_att = self.drone.get_attitude()
        if msg_att is None:
            return None
        roll, pitch, yaw = msg_att

        # sprawdzamy, czy piksel jest w zakresie obrazu
        if not (0 <= u < self.image_width and 0 <= v < self.image_height):
            print(f"Piksel ({u},{v}) poza zakresem ({self.image_width}x{self.image_height})")
            return None
        
        # korekta dystorsji
        undistorted = cv2.undistortPoints(
            np.array([[u, v]], dtype=np.float32),
            cameraMatrix=MissionService.K,
            distCoeffs=MissionService.dist
        )
        x_u, y_u = undistorted[0,0]
        
        # promień w układzie kamery
        ray_cam = np.array([x_u, -y_u, 1.0])
        ray_cam /= np.linalg.norm(ray_cam)

        R_world_body = self.rot_matrix(roll, pitch, yaw)
        ray_world = R_world_body @ ray_cam
        dz = ray_world[2]

        # Sprawdź czy promień nie jest równoległy do ziemi lub skierowany ponad horyzont
        if abs(dz) < 1e-6 or dz <= 0:
            print("Ray is parallel to ground or above horizon, cannot compute intersection.")
            return None
        
        # ---- przecięcie z ziemią ----
        h_g = 0.0
        t = (alt_uav - h_g) / dz
        hit_local = ray_world * t   # [north, east, down]

        # ---- GPS offset ----
        lat_t, lon_t = mavextra.gps_offset(
            lat_uav, lon_uav,
            hit_local[0],  # north
            hit_local[1]   # east
        )
        print("detected point", lat_t, lon_t)
        return lat_t, lon_t

    def isinrect(self, lat, lon) -> bool:
        # Define corners as a list of (lat, lon) tuples in order
        num = len(self.GEOFENCE)
        inside = False

        for i in range(num):
            j = (i + 1) % num
            xi, yi = self.GEOFENCE[i][1], self.GEOFENCE[i][0]
            xj, yj = self.GEOFENCE[j][1], self.GEOFENCE[j][0]
            if ((yi > lat) != (yj > lat)) and \
            (lon < (xj - xi) * (lat - yi) / (yj - yi + 1e-12) + xi):
                inside = not inside
        return inside
    
    def is_target(self, lat, lon):
        """
        Returns True if (lat, lon) is within 5 meters of any target in self.TRG_CANDIDATES.
        Also, increment counter if true.
        self.TRG_CANDIDATES should be a list of (lat, lon, count, isRed) dicts.
        """
        for target in self.TRG_CANDIDATES:
            t_lat, t_lon = target
            distance_lat = abs(lat-t_lat)
            distance_lon = abs(lon-t_lon)

            if distance_lat*111_320 <= 5:
                if distance_lon*85000 <=5:
                    target["count"]+=1
                    return True
        return False
    
    def insert_target(self, lat, lon, isRed: bool):
        self.TRG_CANDIDATES.append({"lat": lat, "lon": lon, "count": 1, "isRed": isRed})

    def process_target(self, pixel, isRed)-> bool:
        res = self.calculate_target_cords(pixel)
        if res is None:
            return False
        lat, lon = res
        if not self.isinrect(lat, lon):
            return False
        if not self.is_target(lat, lon):
            self.insert_target(lat, lon, isRed)
        return True
        
    def select_targets(self):
        """
        Wybiera dwa słowniki o największych wartościach 'count' z self.TRG_CANDIDATES,
        usuwa resztę z listy i zwraca listę tych dwóch słowników.
        Jeśli kandydatów jest mniej niż dwóch, zwraca tyle ile jest.
        """
        if self.TRG_CANDIDATES==[]:
            return None
        # Sortuj malejąco po 'count'
        sorted_candidates = sorted(self.TRG_CANDIDATES, key=lambda t: t["count"], reverse=True)
        # Usuń drugiego kandydata jeśli oba mają ten sam kolor, powtarzaj aż będą różne lub zostanie jeden
        while len(sorted_candidates) > 1 and (sorted_candidates[0]["isRed"] == sorted_candidates[1]["isRed"]):
            sorted_candidates.pop(1)
        top_two = sorted_candidates[:2]
        # Zastąp listę tylko tymi dwoma
        self.TRG_CANDIDATES = top_two


    def calc_drop_waypoints(self, trg_dict: dict, yaw: float, container: list, alt=40):
        """
        Oblicza 3 waypointy leżące na linii o zadanym kierunku (yaw, w radianach), 

        Args:
            yaw: kierunek w radianach (0 = północ, pi/2 = wschód)
        Returns: 
            lista: [(lat1, lon1), (lat2, lon2), (lat3, lon3)]
        """
        dist_and_acr = [(80, 40), (62, 5), (40, 40)]  # metry przed targetem

        # Przesunięcie "przed target" to minus yaw (czyli -yaw)
        for d, acr in dist_and_acr:
            # offset w metrach
            north = -d * np.cos(yaw)
            east = -d * np.sin(yaw)
            # użyj mavextra.gps_offset do przeliczenia na lat/lon
            wp_lat, wp_lon = mavextra.gps_offset(trg_dict["lat"], trg_dict["lon"], north, east)
            container.append({"lat": wp_lat, "lon": wp_lon, "alt": alt, "acr": acr, "cmd": "NAV"})
        if trg_dict["isRed"]:
            container.insert(2, {"pwm": PixHawkService.PWM_DROP_SERVO, "ch": PixHawkService.RED_CH, "cmd": PixHawkService.SET_SERVO_CMD})
        else:
            container.insert(2, {"pwm": PixHawkService.PWM_DROP_SERVO, "ch": PixHawkService.BLUE_CH, "cmd": PixHawkService.SET_SERVO_CMD})
        return container

    def prepare_wps(self, path1, path2):
        self.NEW_MISSION[12:12], self.NEW_MISSION[20:20] = path1, path2


if __name__ == "__main__":
    a=1
    mission = MissionService(a, (1280, 720))
    ll = [{"lat": 0, "lon": 0, "count": 1, "isRed": True },
          {"lat": 0, "lon": 0, "count": 1, "isRed": True },
          {"lat": 0, "lon": 0, "count": 1, "isRed": True },
          {"lat": 0, "lon": 0, "count": 1, "isRed": True }]
    
    pp = [{"lat": 1, "lon": 1, "count": 2, "isRed": True },
          {"lat": 1, "lon": 1, "count": 2, "isRed": True },
          {"lat": 1, "lon": 1, "count": 2, "isRed": True },
          {"lat": 1, "lon": 1, "count": 2, "isRed": True }]

    for i in mission.NEW_MISSION:
        print(i)

    mission.prepare_wps(ll, pp)
    for i in mission.NEW_MISSION:
        print(i)


