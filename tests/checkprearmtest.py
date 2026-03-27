from pymavlink import mavutil, mavextra
import time
from typing import Any, Tuple, List, Dict, Optional
import numpy as np
#from pyproj import Transformer, CRS
#import cv2
from math import radians

class MatekService:
    """
    Example usage:
    # Initialize connection\n
    drone = MatekService()

    waypoints = [
        {"lat": -35.363261, "lon": 149.165230, "alt": 20},
        {"lat": -35.363500, "lon": 149.165500, "alt": 25},
        {"lat": -35.363800, "lon": 149.165800, "alt": 20}
    ]

    try:
        # Upload mission\n
        if drone.set_waypoints(waypoints):
            print("Mission uploaded successfully")
            # Start mission\n
            if drone.start_mission():
                print("Mission started - monitoring progress...")
                # Monitor for 60 seconds\n
                for _ in range(30):
                    drone.monitor_mission()
                    time.sleep(2)
        else:
            print("Failed to upload mission")
    except KeyboardInterrupt:
        print("\nMission monitoring stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        drone.disarm()
        drone.close()
    """

    def __init__(self, device: str = "/dev/ttyAMA2", baud: int = 115200):
        """
        Connects to Matek via MAVLink.

        :param device: string to device (eg. '/dev/ttyAMA0')
        :param baud: baud rate (default 57600)
        """

        try:
            self.master = mavutil.mavlink_connection(device, baud=baud)
            self.master.wait_heartbeat()
            print(f"Connected to system {self.master.target_system}, component {self.master.target_component}")
        except Exception as e:
            print(f"Connection failed: {e}")
            raise

    
    def check_prearm_status(self) -> bool:
        """
        Checks GPS, EKF, and system status before arming.

        :return: True if drone is ready for arming, False otherwise
        """
        checks_passed = True

        # GPS check
        gps = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        print(gps)
        if not gps:
            print("GPS not found")
        if gps.fix_type < 3:
            print("GPS not ready (need 3D fix)")
            checks_passed = False
        else:
            print(f"GPS ready (fix type: {gps.fix_type}, satellites: {gps.satellites_visible})")

        # System status
        sys_status = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        print(sys_status)
        if sys_status:
            # Check individual sensor health bits
            sensors = sys_status.onboard_control_sensors_health
            if sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO:
                print("Gyroscope healthy")
            else:
                print("Gyroscope not healthy")
                checks_passed = False

            if sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL:
                print("Accelerometer healthy")
            else:
                print("Accelerometer not healthy")
                checks_passed = False
        else:
            print("No system status received")
            checks_passed = False

        # EKF check (if available)
        ekf = self.master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=2)
        print(ekf)
        if ekf:
            if ekf.flags & 0x01:  # EKF solution OK bit
                print("EKF stable")
            else:
                print("EKF not stable")
                checks_passed = False
        else:
            print("EKF status not available (may be normal)")

        # Battery check
        battery = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
        print(battery)
        if battery and hasattr(battery, 'voltage_battery'):
            voltage = battery.voltage_battery / 1000.0  # Convert from mV to V
            if voltage > 10.5:  # Minimum safe voltage for most setups
                print(f"Battery voltage OK: {voltage:.1f}V")
            else:
                print(f"Battery voltage low: {voltage:.1f}V")
                checks_passed = False

        if checks_passed:
            print("All pre-arm checks passed")
        else:
            print("Some pre-arm checks failed")

        return checks_passed
    def get_current_coordinates(self, timeout=2) -> Optional[Tuple[float, float, float]]:
        """
        Reads current GPS position (lat, lon, alt).

        :return: Tuple of (latitude, longitude, altitude) or None if no GPS data
        """
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if not msg:
            print("No GPS data received.")
            return None
        return msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0

    

drone = MatekService()
print("init")
print(drone.check_prearm_status())
print(drone.get_current_coordinates())
