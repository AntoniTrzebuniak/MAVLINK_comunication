import os
import csv
import time
from datetime import datetime
from pymavlink import mavutil
from picamera2 import Picamera2
from Application.configuration.config_loader import cfg     # mandatory
from Application.Logger.log_module import get_logger       
from Application.Services.MatekService import MatekService


class CameraService:
    def __init__(self, drone: MatekService):
        self.logger = get_logger(__name__)
        self.PHOTOS_DIR = cfg.dirs.photos_dir
        self.PHOTOS_MISSION_DIR = self.PHOTOS_DIR / f"mission_{datetime.now().strftime('Mission_%Y-%m-%d_%H-%M')}"
        os.makedirs(self.PHOTOS_MISSION_DIR, exist_ok=True)
        self.LOG_FILE = self.PHOTOS_MISSION_DIR / 'photos_position.csv'
        self.drone = drone

        self.picam = Picamera2()
        config = self.picam.create_still_configuration(main={"size": (2304, 1296)})
        config["main"]["quality"] = 100
        self.picam.configure(config)
        self.picam.start()
        self.picam.set_controls({"AfMode": 2, "ExposureTime": 10000})
        
        with open(self.LOG_FILE, 'w', newline='') as f:
            csv.writer(f).writerow(['Filename', 'Index', 'Lat', 'Lon', 'Alt', 'Roll', 'Pitch', 'Yaw'])

    def image_capture_listener(self):
        """
        Nasłuchuje komunikatów CAMERA_FEEDBACK i reaguje na nie, wykonując zdjęcia i zapisując dane.
        Ten kod powinien być uruchomiony w osobnym wątku lub procesie, aby nie blokować głównej logiki drona.
        """
        while True:
            msg = self.drone.master.recv_match(type='CAMERA_FEEDBACK', blocking=True)   # 'TERRAIN_REPORT'
            
            if msg:
                ts = datetime.now().strftime("%H-%M-%S_%f")
                img_idx = msg.img_idx
                filename = f"IMG_{img_idx:04d}_{ts}.jpg"
                self.picam.capture_file(self.PHOTOS_MISSION_DIR/filename)
                self.logger.info(f"Zrobiono zdjęcie: {filename} (img_idx={img_idx})")
                lat = msg.lat / 1e7
                lon = msg.lng / 1e7
                alt = msg.alt_msl        # Wysokość n.p.m. (lub relatywna, zależnie od ustawień)
                r, p, y = msg.roll, msg.pitch, msg.yaw
                
                # 2. Zapis precyzyjnych danych z komunikatu
                with open(self.LOG_FILE, 'a', newline='') as f:
                    csv.writer(f).writerow([filename, img_idx, lat, lon, alt, r, p, y])
                
