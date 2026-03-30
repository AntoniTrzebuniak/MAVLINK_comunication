from __future__ import annotations
from pymavlink import mavutil, mavextra
import time
from typing import Any, Tuple, List, Dict, Optional
import numpy as np
#from pyproj import Transformer, CRS
#import cv2
from math import radians

from Application.configuration.config_loader import cfg     # Mandatory
from Application.Logger.log_module import get_logger


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

    def __init__(self, device: str = cfg.mav.device, baud: int = cfg.mav.baud):
        """
        Connects to Matek via MAVLink.

        :param cfg: optional Config object loaded from config.toml
        :param device: device path e.g. '/dev/ttyAMA0' (overrides cfg)
        :param baud: baud rate (overrides cfg)
        """
        self.logger = get_logger(__name__)
        self.device = device
        self.baud = baud

        try:
            self.master = mavutil.mavlink_connection(self.device, baud=self.baud)
            self.master.wait_heartbeat()
            self.logger.info("Connected to system %s component %s", self.master.target_system, self.master.target_component)
        except Exception as e:
            self.logger.exception("Connection failed: %s", e)
            raise

    def get_current_coordinates(self, timeout=2) -> Optional[Tuple[float, float, float]]:
        """
        Reads current GPS position (lat, lon, alt).

        :return: Tuple of (latitude, longitude, altitude) or None if no GPS data
        """
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if not msg:
            self.logger.warning("No GPS data received.")
            return None
        return msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0

    def is_armed(self) -> bool:
        """
        Checks if the drone is armed (returns True/False).

        :return: True if armed, False if disarmed or no heartbeat
        """
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            return (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        return False

    def get_current_mode(self) -> str:
        """
        Gets the current flight mode.

        :return: current flight mode or 'UNKNOWN'
        """
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            # Convert mode number to mode name
            mode_mapping = self.master.mode_mapping()
            for mode_name, mode_id in mode_mapping.items():
                if mode_id == msg.custom_mode:
                    return mode_name
        return "UNKNOWN"

    def get_mission(self) -> List[Dict[str, Any]]:
        """
        Downloads mission as list of raw MAVLink mission items.

        :return: List of dicts with seq, command, frame, current,
        autocontinue, param1..param7
        """
        mission = []
        self.master.mav.mission_request_list_send(self.master.target_system, self.master.target_component)
        msg = self.master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
        if not msg:
            self.logger.warning("No mission count received")
            return mission

        count = msg.count
        self.logger.debug(f"Mission has {count} items")

        for i in range(count):
            #self.master.mav.mission_request_send(self.master.target_system, self.master.target_component, i)
            #item = self.master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)

            self.master.mav.mission_request_int_send(self.master.target_system, self.master.target_component, i, 0)
            item = self.master.recv_match(type='MISSION_ITEM_INT', blocking=True, timeout=5)
            if item:
                mission.append({
                    "seq": item.seq,
                    "command": item.command,
                    "frame": item.frame,
                    "current": item.current,
                    "autocontinue": item.autocontinue,

                    "param1": item.param1,
                    "param2": item.param2,
                    "param3": item.param3,
                    "param4": item.param4,
                    "param5": item.x,
                    "param6": item.y,
                    "param7": item.z,
                })


                self.logger.debug(f"""
                --- MISSION ITEM {item.seq} ---
                command: {item.command}
                frame: {item.frame}
                current: {item.current}
                autocontinue: {item.autocontinue}

                param1: {item.param1}
                param2: {item.param2}
                param3: {item.param3}
                param4: {item.param4}
                param5(x): {item.x}
                param6(y): {item.y}
                param7(z): {item.z}
                ----------------------------
                """)

        return mission

    def set_waypoints(self, waypoints: list[dict[str, Any]]) -> bool:           

	    # komentarz : ogólnie to funkcja działa, ale pierwszy punkt jest ignorowany z jakiegoś powodu!!
		# ardupilot pierwszy punkt traktuje jako Home, dodałem implementację w której automatycznie punkt
		# zerowy zostaje dodany, nie bierze on udziału w misji

        
        home = {"command": 16, "frame": 0, "current": 1, "autocontinue": 1,
            "param1": 0, "param2": 0, "param3": 0, "param4": 0,
            "param5": 0, "param6": 0, "param7": 0}

        
        command_map = {"WAYPOINT": 16, "SET_SERVO": 183}

        all_waypoints = [home] + waypoints
    
        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            len(all_waypoints),
            0
        )

        for i, wp in enumerate(all_waypoints):
            # Czekamy na żądanie przesłania waypointa
            req = self.master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=5)
            if not req:
                self.logger.warning(f"No mission request received for waypoint {i}")
                return False

            self.logger.info(f"Got request type: {req.get_type()}, seq: {req.seq}")
            # Obsługa dwóch typów danych wejściowych : 
            # jeśli param5 istnieje, dane pochodzą z mateka
            if "param5" in wp:
                cmd = wp["command"]
                self.master.mav.mission_item_int_send(
                    self.master.target_system,
                    self.master.target_component,
                    i,
                    wp["frame"],
                    cmd,
                    wp["current"],
                    wp["autocontinue"],
                    wp["param1"],
                    wp["param2"],
                    wp["param3"],
                    wp["param4"],
                    int(wp["param5"]),
                    int(wp["param6"]),
                    wp["param7"]
            )
            # jeśli param5 nie istnieje, dane pochodzą z kodu i należy je przetłumaczyć na format MAVLink
            else:
                cmd = command_map.get(wp["command"], wp["command"])

                if cmd == 16:  # WAYPOINT
                    self.master.mav.mission_item_int_send(
                        self.master.target_system,
                        self.master.target_component,
                        i,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                        1 if i == 0 else 0,
                        1,
                        0,
                        wp["acr"],
                        0, 0,
                        int(wp["lat"] * 1e7),
                        int(wp["lon"] * 1e7),
                        wp["alt"]
                    )
                elif cmd == 183:  # SET_SERVO
                    self.master.mav.mission_item_int_send(
                        self.master.target_system,
                        self.master.target_component,
                        i,
                        mavutil.mavlink.MAV_FRAME_MISSION,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0, 1,
                        wp["channel"],  # param1 = channel
                        wp["pwm"],       # param2 = PWM
                        0, 0, 0, 0, 0    # param3-7 (unused)
                    )
                # jeśli planujemy ręcznie dodawać inne typy komend do misji, należy je zdefiniować tutaj ręcznie
            

                else:
                    self.logger.error(f"Unknown command in waypoint {i}: {wp['command']}")
                    return False
        ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if ack:
            self.logger.info(f"Mission ACK: {ack.type}")

        return True
             
    def append_waypoints(self, new_wps: List[Dict[str, Any]]) -> bool:
        """
        Appends new waypoints to existing mission.

        :param new_wps: List of waypoint dicts to append
        :return: True if mission was updated successfully, False otherwise
        """
        mission = self.get_mission()

        # pomijamy punkt zerowy
        if mission:
            mission = mission[1:]
        mission.extend(new_wps)
        self.logger.info(f"Appending {len(new_wps)} waypoints (new total {len(mission)})")
        return self.set_waypoints(mission)

    def prepend_waypoints(self, new_wps: List[Dict[str, Any]]) -> bool:
        """
        Prepends waypoints at the beginning of mission.

        :param new_wps: List of waypoint dicts to prepend
        :return: True if mission was updated successfully, False otherwise
        """
        mission = self.get_mission()
        # pomijamy punkt zerowy
        if mission:
            mission = mission[1:]
        mission = new_wps + mission
        self.logger.info(f"Prepended {len(new_wps)} waypoints, new total {len(mission)}")
        return self.set_waypoints(mission)

    def set_current_waypoint(self, index: int) -> bool:
        """
        Sets current waypoint index (0-based)
        """
        real_index = index + 1  # przesunięcie przez index home w funkcji set_waypoints

        self.master.mav.mission_set_current_send(
            self.master.target_system,
            self.master.target_component,
            real_index
        )
        # Wait for acknowledgment
        ack = self.master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=5)

        if ack and ack.seq == real_index:
            self.logger.info(f"Successfully set current waypoint to index {index}")
            return True
        else:
            self.logger.error(f"Failed to set current waypoint to index {index}")
            return False

    def arm(self) -> bool:
        """
        Arms the drone (if not armed).

        :return: True if drone is armed successfully, False otherwise
        """
        if self.is_armed():
            self.logger.info("Drone already armed")
            return True

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm (1=arm, 0=disarm)
            0, 0, 0, 0, 0, 0  # unused parameters
        )

        # Wait for acknowledgment
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.logger.debug("Arm command accepted")
            # Wait a moment and check if actually armed
            time.sleep(1)
            if self.is_armed():
                self.logger.info("Drone successfully armed")
                return True
            else:
                self.logger.warning("Arm command sent but drone not armed")
                return False
        else:
            self.logger.error(f"Arm command failed: {ack.result if ack else 'timeout'}")
            return False

    def disarm(self) -> bool:
        """
        Disarms the drone (if armed).

        :return: True if drone is disarmed successfully, False otherwise
        """
        if not self.is_armed():
            self.logger.info("Drone already disarmed")
            return True

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # disarm (1=arm, 0=disarm)
            0, 0, 0, 0, 0, 0  # unused parameters
        )

        # Wait for acknowledgment
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Disarm command accepted")
            time.sleep(1)
            if not self.is_armed():
                self.logger.info("Drone successfully disarmed")
                return True
            else:
                self.logger.warning("Disarm command sent but drone still armed")
                return False
        else:
            self.logger.error(f"Disarm command failed: {ack.result if ack else 'timeout'}")
            return False

    def set_mode(self, mode_name: str) -> bool:
        """
        Sets the flight mode (e.g., 'AUTO', 'GUIDED').

        :param mode_name: Flight mode name to set
        :return: True if mode was set successfully, False otherwise
        """
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            self.logger.error(f"Unknown mode: {mode_name}")
            available_modes = list(self.master.mode_mapping().keys())
            self.logger.info(f"Available modes: {available_modes}")
            return False

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

        # Verify mode change
        time.sleep(0.5)
        current_mode = self.get_current_mode()
        if current_mode == mode_name:
            self.logger.info(f"Successfully set mode to {mode_name}")
            return True
        else:
            self.logger.warning(f"Mode change failed. Current mode: {current_mode}")
            return False




# dotąd sprawdzałem czy działa i naprawiałem
    def check_prearm_status(self) -> bool:
        """
        Checks GPS, EKF, and system status before arming.

        :return: True if drone is ready for arming, False otherwise
        """
        checks_passed = True

        # GPS check
        gps = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if not gps or gps.fix_type < 3:
            self.logger.warning("GPS not ready (need 3D fix)")
            checks_passed = False
        else:
            self.logger.info(f"GPS ready (fix type: {gps.fix_type}, satellites: {gps.satellites_visible})")

        # System status
        sys_status = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if sys_status:
            # Check individual sensor health bits
            sensors = sys_status.onboard_control_sensors_health
            if sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO:
                self.logger.info("Gyroscope healthy")
            else:
                self.logger.warning("Gyroscope not healthy")
                checks_passed = False

            if sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL:
                self.logger.info("Accelerometer healthy")
            else:
                self.logger.warning("Accelerometer not healthy")
                checks_passed = False
        else:
            self.logger.error("No system status received")
            checks_passed = False

        # EKF check (if available)
        ekf = self.master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=2)
        if ekf:
            if ekf.flags & 0x01:  # EKF solution OK bit
                self.logger.info("EKF stable")
            else:
                self.logger.warning("EKF not stable")
                checks_passed = False
        else:
            self.logger.info("EKF status not available (may be normal)")

        # Battery check
        battery = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
        if battery and hasattr(battery, 'voltage_battery'):
            voltage = battery.voltage_battery / 1000.0  # Convert from mV to V
            if voltage > 10.5:  # Minimum safe voltage for most setups
                self.logger.info(f"Battery voltage OK: {voltage:.1f}V")
            else:
                self.logger.warning(f"Battery voltage low: {voltage:.1f}V")
                checks_passed = False

        if checks_passed:
            self.logger.info("All pre-arm checks passed")
        else:
            self.logger.warning("Some pre-arm checks failed")

        return checks_passed

    def start_mission(self, start_index: int = 0) -> bool:
        """
        Runs pre-arm checks, arms drone, sets AUTO mode,
        and starts the mission from the given waypoint index.

        :param start_index: Waypoint index to start from (0-based)
        :return: True if mission started successfully, False otherwise
        """
        self.logger.info("Starting mission sequence...")

        if not self.check_prearm_status():
            self.logger.warning("Pre-arm checks failed - mission not started")
            return False

        # Check if mission exists
        mission = self.get_mission()
        if not mission:
            self.logger.error("No mission loaded")
            return False

        if start_index >= len(mission):
            self.logger.error(f"Start index {start_index} exceeds mission length {len(mission)}")
            return False

        # Arm the drone
        if not self.arm():
            self.logger.error("Failed to arm drone")
            return False

        # Set AUTO mode
        if not self.set_mode("AUTO"):
            self.logger.error("Failed to set AUTO mode")
            return False

        # Set starting waypoint
        if not self.set_current_waypoint(start_index):
            self.logger.error(f"Failed to set starting waypoint {start_index}")
            return False

        self.logger.info(f"Mission started successfully from waypoint {start_index}")
        return True

    def emergency_stop(self) -> None:
        """
        Emergency stop - disarm immediately.

        Forces immediate disarm without waiting for acknowledgment.
        """
        self.logger.info("EMERGENCY STOP - Disarming drone")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # disarm
            21196,  # force disarm magic number
            0, 0, 0, 0, 0
        )

    def get_mission_status(self) -> Optional[Dict[str, int]]:
        """
        Gets current mission progress.

        :return: Dict with current_waypoint and total_waypoints, or None if no data
        """
        current = self.master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=3)
        if current:
            return current.seq, len(self.get_mission())-1  # pomijamy waypoint home
        return None

    def wait_for_command_ack(self, command: int, timeout: int = 5) -> bool:
        """
        Waits for command acknowledgment.

        :param command: MAVLink command ID to wait for
        :param timeout: Timeout in seconds
        :return: True if command was acknowledged successfully, False otherwise
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            ack = self.master.recv_match(type='COMMAND_ACK', blocking=False)
            if ack and ack.command == command:
                return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            time.sleep(0.1)
        return False

    def monitor_mission(self, update_interval: int = 2) -> Dict:
        """
        Monitors mission progress in real-time.
        Call this in a loop to track mission status.

        :param update_interval: Update interval in seconds (not used in current implementation)
        :return: Dict with coordinates, mission_status, mode, and armed status
        """
        coords = self.get_current_coordinates()
        curr_wp, total_wp = self.get_mission_status()
        mode = self.get_current_mode()
        armed = self.is_armed()

        if coords and curr_wp is not None:
            self.logger.info(f"Position: {coords[0]:.6f}, {coords[1]:.6f}, Alt: {coords[2]:.1f}m")
            self.logger.info(
                f"Mission: {curr_wp}/{total_wp} | Mode: {mode} | Armed: {armed}")

        return {
            "coordinates": coords,
            "mission_status": (curr_wp, total_wp),
            "mode": mode,
            "armed": armed
        }

    def close(self) -> None:
        """
        Closes the MAVLink connection.

        Should be called when finished to properly cleanup resources.
        """
        if hasattr(self, 'master'):
            self.master.close()
            print("MAVLink connection closed")
    
    def add_drop_waypoint(self, new_waypoint):
        print("START")
        mission = self.get_mission()

        if mission:
            mission = mission[1:] 

        best_idx = -1
        best_distance = 10**100

        nav_mission = []

        for idx, item in enumerate(mission):
            if item["command"] == 16:
                nav_mission.append({
                    "lat": item["param5"] / 1e7,
                    "lon": item["param6"] / 1e7,
                    "alt": item["param7"],
                    "idx": idx
                })

        if not nav_mission:
            return [new_waypoint]

        for i in range(1, len(nav_mission) + 1):
            i = i % len(nav_mission)
            future_waypoint = nav_mission[i]
            prev_waypoint = nav_mission[i - 1]

            new_lat = new_waypoint["lat"]
            new_lon = new_waypoint["lon"]
            prev_lat = prev_waypoint["lat"]
            prev_lon = prev_waypoint["lon"]

            distance = (new_lat - prev_lat) ** 2 + (new_lon - prev_lon) ** 2

            print(f"previous wp: {prev_waypoint}\nfuture wp: {future_waypoint}\ndistance = {distance}")
            print("\n\n\n")

            if distance < best_distance:
                best_distance = distance
                best_idx = i

        if best_idx == 0:
            mission = mission + [new_waypoint]
        else:
            real_idx = nav_mission[best_idx]["idx"]
            mission = mission[:real_idx] + [new_waypoint] + mission[real_idx:]

        return self.set_waypoints(mission)
    
    def get_attitude(self, timeout=1):
        msg_att = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=timeout)
        if msg_att is None:
            self.logger.warning("No attitude data received")
            return None
        roll  = msg_att.roll    # rad +prawo -lewo
        pitch = msg_att.pitch   # rad +góra  -dół
        yaw   = msg_att.yaw     # rad +prawo -lewo
        return roll, pitch, yaw
    
    def set_servo(self, channel: int, pwm: int) -> bool:
        """
        Sets a servo output using.
        Works on ArduPilot (channels usually start at 1 = AUX1).

        :param channel: Servo channel number (e.g. 5 = AUX1)
        :param pwm: PWM value in microseconds (typically 1000–2000)
        :return: True if command was acknowledged, False otherwise
        """
        self.logger.info(f"Setting servo channel {channel} to {pwm}µs")

        # Send command
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,          # confirmation
            channel,    # param1: servo number
            pwm,        # param2: PWM microseconds
            0, 0, 0, 0, 0  # unused params
        )

        # Wait for ACK
        return self.wait_for_command_ack(mavutil.mavlink.MAV_CMD_DO_SET_SERVO, timeout=3)
        
    def close(self) -> None:
        """
        Closes the MAVLink connection.

        Should be called when finished to properly cleanup resources.
        """
        if hasattr(self, 'master'):
            self.master.close()
            self.logger.info("MAVLink connection closed")

    def sandbox(self) -> List[Dict[str, float]]:
        mission = []

        # Zapytaj o misję
        self.master.mav.mission_request_list_send(self.master.target_system, self.master.target_component)

        msg = self.master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
        if not msg:
            print("No mission count received")
            return mission

        count = msg.count
        print(f"Mission has {count} waypoints")

        for i in range(count):
            self.master.mav.mission_request_send(self.master.target_system, self.master.target_component, i)

            while True:
                item = self.master.recv_match(blocking=True, timeout=5)
                if not item:
                    print(f"Timeout waiting for waypoint {i}")
                    return mission

                if item.get_type() in ["MISSION_ITEM", "MISSION_ITEM_INT"]:
                    mission.append({
                        "seq": item.seq,
                        "lat": item.x, #/ 1e7 if hasattr(item, "x") else item.x,
                        "lon": item.y, #/ 1e7 if hasattr(item, "y") else item.y,
                        "alt": item.z,
                        "command": item.command
                    })
                    break  # przechodzimy do kolejnego waypointa
        mission.pop(0)
        # Odbierz ACK na końcu
        ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if ack:
            print(f"Mission download ACK: {ack.type}")

        return mission

'''
if __name__ == "_main__":      # zamierzone użycie klasy
    drone = MatekService()
    mission = MissionService(drone)
    #TODO getPixel która zdobędzie piksel od image_processing
    # pytanie kiedy zdjęcie było zrobione bo jak img processing ~0.5s to snapshot parametrów opóźniony
    pixel = mission.getPixel()
    is_accepted = mission.accept_target(pixel)
    
    if is_accepted:
        print("Target cords was added to list")
    #kontynujemy wykryanie celów
    # ...
    yaw = 90 # tutaj ustawiamy z której strony nalatujemy nad cel, do ustalenia
    aiming_Paths = []
    for target in mission.TARGETS:
        path = mission.calc_drop_waypoints(target, yaw)
        aiming_wps = mission.prepare_wps(path, alt=40)
        aiming_Paths.append(path)
        yaw = -yaw
    for new_wps in aiming_Paths:
        
        if not drone.append_waypoints(new_wps):
            print("couldnt update mission")
    
    #TODO jak wysłać sygnał do szczerba o zrzuceniu rzutki?


    
def rot_matrix(roll, pitch, yaw):
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



if __name__ == "__main__":

    lat_uav, lon_uav = 40.736313, 30.073209        # [deg]
    alt_uav = 100.0          # [m] nad ziemią
    print("UAV:", lat_uav, lon_uav, "alt:", alt_uav)


    roll  = radians(90)              # [rad]
    pitch = radians(0)              # [rad]
    yaw   = radians(0)
    
    pixels= [(0,0), (0, 1295), (2303,0), (2303, 1295)]


    for pixel in pixels:
        # rozdzielczość kamery
        
        width, height = 2304, 1296

        # intrinsic (z kalibracji)
        #K = np.array([
        #    [1.37057373e+03, 0.00000000e+00, 1.08538067e+03],
         #   [0.00000000e+00, 1.38363295e+03, 7.10619956e+02],
          #  [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        #], dtype=np.float32)

        # obliczone przez chata
        K = np.array([
            [1773.9244383143994, 0.00000000e+00, 1152],
            [0.00000000e+00, 1733.1547280645818, 648],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ], dtype=np.float32)

        dist = np.array([[-0.1359832, 0.36278877, 0.02114295, 0.02506856, -0.21990155]], dtype=np.float32)

        # ---- undistort piksel ----
        u,v = pixel
        undistorted = cv2.undistortPoints(
            np.array([[u, v]], dtype=np.float32),
            cameraMatrix=K,
            distCoeffs=dist
        )
        x_u, y_u = undistorted[0,0]
        # ---- promień w kamerze ----
        ray_cam = np.array([x_u, -y_u, 1.0])
        ray_cam /= np.linalg.norm(ray_cam)


        R_world_body = rot_matrix(roll, pitch, yaw)
        ray_world = R_world_body @ ray_cam

        dz = ray_world[2]
        if abs(dz) < 1e-6 or dz <= 0:
            print("Ray is parallel to ground or above horizon, cannot compute intersection.")
        
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

        print( lat_t, lon_t)
        #ISin = isinrect(lat_t, lon_t)
'''