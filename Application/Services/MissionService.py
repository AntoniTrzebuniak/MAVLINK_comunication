from __future__ import annotations
from Application.Services.MatekService import MatekService
from Application.configuration.config_loader import cfg     # mandatory
from Application.Logger.log_module import get_logger
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
        self.image_width, self.image_height = cfg.camera.resolution

        self.TRG_CANDIDATES=[] #{"lat": lat, "lon": lon, "count": 1, "isBottle": isBottle}

        self.drone = drone

    K = cfg.camera.K
    dist = cfg.camera.distortion

    
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

    def project_target_cords(self, pixel: tuple, lat_uav, lon_uav, alt_uav,roll, pitch, yaw)->tuple:
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

    def isinPolygon(lat, lon, Polygon: List[Tuple[float, float]]) -> bool:
        '''
           Polygon: List of (lat, lon) tuples defining the vertices of the polygon in order!!!
        '''
        num = len(Polygon)
        inside = False
        for i in range(num):
            j = (i + 1) % num
            xi, yi = Polygon[i][1], Polygon[i][0] # xi = lon, yi = lat
            xj, yj = Polygon[j][1], Polygon[j][0]
            # Algorytm Ray Casting
            if ((yi > lat) != (yj > lat)) and \
            (lon < (xj - xi) * (lat - yi) / (yj - yi + 1e-12) + xi):
                inside = not inside
        return inside
    
    def is_target(self, lat, lon, threshold=5.0):
        """
            Sprawdza, czy (lat, lon) znajduje się w promieniu threshold_meters od celów.
            Modyfikuje self.TRG_CANDIDATES, zwiększając 'count' dla celów within threshold.
        """
        METERS_PER_DEGREE = 111319.5
        cos_lat = math.cos(math.radians(lat))
        meters_per_lon = METERS_PER_DEGREE * cos_lat
        
        for target in self.TRG_CANDIDATES:
            dy = (lat - target["lat"]) * METERS_PER_DEGREE
            dx = (lon - target["lon"]) * meters_per_lon
            if (dx * dx + dy * dy) <= threshold**2:
                return True
        return False
    
    def insert_target(self, lat, lon, isBottle: bool):
        self.TRG_CANDIDATES.append({"lat": lat, "lon": lon, "count": 1, "isBottle": isBottle})

    def process_target(self, pixel, isBottle: bool)-> bool:
        msg_gps = self.drone.get_current_coordinates()
        if msg_gps is None:
            return None
        lat_uav, lon_uav, alt_uav = msg_gps
        msg_att = self.drone.get_attitude()
        if msg_att is None:
            return None
        roll, pitch, yaw = msg_att

        res = self.project_target_cords(pixel, lat_uav, lon_uav, alt_uav, roll, pitch, yaw)
        if res is None:
            return False
        lat, lon = res
        if not self.isinPolygon(lat, lon, self.GEOFENCE):
            return False
        if not self.is_target(lat, lon):
            self.insert_target(lat, lon, isBottle)
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
        # Usuń drugiego kandydata jeśli oba są takie samme, powtarzaj aż będą różne lub zostanie jeden
        while len(sorted_candidates) > 1 and (sorted_candidates[0]["isBottle"] == sorted_candidates[1]["isBottle"]):
            sorted_candidates.pop(1)
        top_two = sorted_candidates[:2]
        # Zastąp listę tylko tymi dwoma
        self.TRG_CANDIDATES = top_two
        return top_two


    def calc_drop_waypoints(self, trg_dict: dict, container: list) -> List[dict]: 
        #TODO do zmiany, należy uwzględniać wiatri nie weim czy 3 wp są wystarczające
        """
        Oblicza 3 waypointy leżące na linii o zadanym kierunku (yaw, w radianach), 

        Args:
            yaw: kierunek w stopniach (0 = północ, pi/2 = wschód)
        Returns: 
            lista: [(lat1, lon1), (lat2, lon2), (lat3, lon3)]
        """
        cfg.drops.bottle.x_translation
        cfg.drops.bottle.y_translation
        cfg.drops.bottle.drop_course
        cfg.drops.beacon.x_translation
        cfg.drops.beacon.y_translation
        cfg.drops.beacon.drop_course
        cfg.drops.altitude
        
        dist_and_acr = [(80, 40), (62, 5), (40, 40)]  # (metry przed targetem, ACR)

        # Przesunięcie "przed target" to minus yaw (czyli -yaw)
        for d, acr in dist_and_acr:
            # offset w metrach
            north = -d * np.cos(yaw)
            east = -d * np.sin(yaw)
            # użyj mavextra.gps_offset do przeliczenia na lat/lon
            wp_lat, wp_lon = mavextra.gps_offset(trg_dict["lat"], trg_dict["lon"], north, east)
            container.append({"lat": wp_lat, "lon": wp_lon, "alt": alt, "acr": acr, "cmd": "NAV"})
        if trg_dict["isBottle"]:
            container.insert(2, {"pwm": MatekService.PWM_DROP_SERVO, "ch": MatekService.RED_CH, "cmd": MatekService.SET_SERVO_CMD})
        else:
            container.insert(2, {"pwm": MatekService.PWM_DROP_SERVO, "ch": MatekService.BLUE_CH, "cmd": MatekService.SET_SERVO_CMD})
        return container

    @staticmethod
    def load_Poly(filename) -> list[tuple[float, float]]:
        '''
        Loads polygon vertices from a .poly file
        
        Returns:
            List of (lat, lon) tuples
        '''
        polygon = []
        with open(filename, 'r') as f:
            for line in f:
                lat_str, lon_str = line.strip().split(' ')
                polygon.append((float(lat_str), float(lon_str)))
        return polygon


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