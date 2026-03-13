"""
t_lat, t_lon = 40.73738208763941, 30.067738977233393
lat, lon = 40.73733529346655, 30.067730517308714

distance_lat = abs(lat-t_lat)
distance_lon = abs(lon-t_lon)
print(distance_lat*111320, distance_lon*85000)
if distance_lat*111_320 <= 5:
    if distance_lon*85000 <=5:
        print("gowno")
"""
import numpy as np
from pymavlink import mavextra

def calc_drop_waypoints(target: tuple, yaw: float):
        """
        Oblicza 3 waypointy leżące na linii o zadanym kierunku (yaw, w radianach),
        w odległościach 80m, 60m i 40m PRZED targetem (czyli "pod wiatr" względem yaw).
        :param target: (lat, lon) w stopniach
        :param yaw: kierunek w radianach (0 = północ, pi/2 = wschód)
        :return: lista [(lat1, lon1), (lat2, lon2), (lat3, lon3)]
        """
        lat, lon = target
        distances = [100, 60, 40]  # metry przed targetem
        waypoints = []

        # Przesunięcie "przed target" to minus yaw (czyli -yaw)
        for d in distances:
            # offset w metrach
            north = -d * np.cos(yaw)
            east = -d * np.sin(yaw)
            # użyj mavextra.gps_offset do przeliczenia na lat/lon
            wp_lat, wp_lon = mavextra.gps_offset(lat, lon, north, east)
            waypoints.append((wp_lat, wp_lon))
        
        return waypoints

print(calc_drop_waypoints((40.73641380938534, 30.07330877131571), 0))