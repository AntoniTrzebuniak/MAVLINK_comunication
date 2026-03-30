from PixHawkService_Kamil import PixHawkService
import numpy as np
from pymavlink import mavutil, mavextra
from typing import Tuple, List, Dict, Optional
import cv2
import core_math

class MissionService:
    """
    Args:
        drone: PixHawkService object instance
        resolution: (width, height)
    """
    def __init__(self, drone: PixHawkService, resolution: tuple):
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

    def calculate_target_cords(self, pixel: tuple)->tuple | None:
        """
        Oblicza współrzędne geograficzne punktu na ziemi
        odpowiadającego pikselowi (u,v) z obrazu kamery
        zamontowanej pionowo w dół (nadir).
        Args:
            pixel: (width, height)
        Returns:
            (lat, lon) or None 
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
            t_lat = target["lat"]
            t_lon = target["lon"]
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

    def calc_release_point(
        self, 
        target: dict, 
        yaw: float, 
        speed: float, 
        altitude_agl: float, 
        solver,  # ShootingSolver instance
        env # simulation environment instance
    ) -> dict:
        """
        Wyznacza punkt GPS, który wyliczyła symulacja by trafić w cel.
        
        Args:
            target: Słownik z kluczami 'lat', 'lon', 'isRed' reprezentujący cel
            yaw: Kierunek podejścia w radianach (0 = North, pi/2 = East)
            speed: Prędkość samolotu w m/s
            altitude_agl: Wysokość nad celem w metrach (AGL)
            solver: Instancja klasy ShootingSolver, która ma metodę calculate_release_point()
        Returns:
            Słownik z kluczami 'lat', 'lon', 'isRed', 'original_target_lat', 'original_target_lon'
            reprezentujący punkt GPS do zrzutu oraz informacje o celu.
        """
        # skłądowe prędkości w lokalnym ukladzie odniesienia
        vx = speed * np.cos(yaw)
        vy = speed * np.sin(yaw)
        vz = 0.0  # zakładamy lot poziomy
        approach_velocity = np.array([vx, vy, vz])

        # liczymy od punku (0,0) i potem zamienimy displacement na GPS
        target_local = np.array([0.0, 0.0])
        
        offset_meters = solver.calculate_release_point(
            target_position=target_local,
            approach_altitude=altitude_agl,
            approach_velocity=approach_velocity,
            env=env
        )
        
        # offset_meters to wektor [north_offset, east_offset] w metrach, który mówi jak daleko od celu (w lokalnym układzie) powinien być punkt zrzutu        
        # konvertujemy na GPS
        release_lat, release_lon = mavextra.gps_offset(
            target["lat"], 
            target["lon"], 
            offset_meters[0], 
            offset_meters[1]  
        )
        
        # Zwracamy punkt zrzutu wraz z informacją o celu (kolor, oryginalne współrzędne) 
        return {
            "lat": release_lat,
            "lon": release_lon,
            "isRed": target["isRed"],
            "original_target_lat": target["lat"], 
            "original_target_lon": target["lon"]
        }


    def calc_drop_waypoints(self, drop_point: dict, yaw: float, container: list, alt=40.0):
    
        """
        drop_point['lat'], drop_point['lon'] = punkt, w którym ma otworzyć się serwo
        yaw = kierunek nalotu w radianach
        """

        dist_and_acr = [
            (80, 40),   
            (40, 15),   
            (0, 5),     
            (-40, 20)   
        ]

        for d, acr in dist_and_acr:
            north = -d * np.cos(yaw)
            east = -d * np.sin(yaw)

            wp_lat, wp_lon = mavextra.gps_offset(
                drop_point["lat"],
                drop_point["lon"],
                north,
                east
            )

            container.append({
                "command": "WAYPOINT",
                "lat": wp_lat,
                "lon": wp_lon,
                "alt": alt,
                "acr": acr
            })

            if d == 0:
                if drop_point["isRed"]:
                    container.append({
                        "command": "SET_SERVO",
                        "channel": PixHawkService.RED_CH,
                        "pwm": PixHawkService.PWM_DROP_SERVO
                    })
                else:
                    container.append({
                        "command": "SET_SERVO",
                        "channel": PixHawkService.BLUE_CH,
                        "pwm": PixHawkService.PWM_DROP_SERVO
                    })

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


    # Przykładowe użycie solvera
    # definicja funckji oporu aerodynamicznego
    drag_area_function =core_math.create_drag_area_model(
        cd_payload=1.0, area_payload=0.0104,
        cd_parachute=0.8, area_parachute=0.159,
        deploy_time=1.5, opening_duration=0.5
    )

    # środowisko, czyli wiatr
    env = core_math.SimulationEnvironment(
        air_density=1.225,
        wind_model=core_math.create_shear_wind(np.array([5.0, 5.0, 0.0]), 1 / 7)
    )

    # solver
    solver = core_math.ShootingSolver(
        mass=0.158,
        drag_area_func=drag_area_function,
        release_latency=0.5,
    )

    # parametry podejścia, może da się nie hardkodować
    flight_speed = 23.0
    flight_altitude_agl = 60.0
    approach_yaw = np.radians(45)  

    aiming_paths = []
    # testowe, żeby zobaczyć czy działa
    mission.TRG_CANDIDATES = [
        {"lat": 40.0, "lon": 30.0, "count": 16, "isRed": True},
        {"lat": 41.0, "lon": 31.0, "count": 16, "isRed": False}
    ]
    # jak użyliśmy już select_targets() to w self.TRG_CANDIDATES powinny być tylko 2 cele, więc iterujemy po nich i generujemy ścieżki
    for target in mission.TRG_CANDIDATES:
        
        # używamy solvera do wyliczenia punktu zrzutu dla tego celu
        release_point = mission.calc_release_point(
            target=target,
            yaw=approach_yaw,
            speed=flight_speed,
            altitude_agl=flight_altitude_agl,
            solver=solver,
            env=env
        )
        
        print(f"Target at: {target['lat']}, {target['lon']}")
        print(f"Releasing at: {release_point['lat']}, {release_point['lon']}")
        
        # używamy calc_drop_waypoints() gdzie drop_point to release_point, który obliczylismy
        path = mission.calc_drop_waypoints(
            drop_point=release_point, 
            yaw=approach_yaw, 
            container=[], 
            alt=flight_altitude_agl
        )
        
        aiming_paths.append(path)
        
        # nie wiem czy robimy kółko czy lecimy z przeciwnej strony, też zależy jak te cele są ułożone
        approach_yaw = -approach_yaw
