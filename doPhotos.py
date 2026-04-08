from picamera2 import Picamera2
from Application.configuration.config_loader import cfg     # mandatory
import os
import time
from datetime import datetime

PHOTOS_DIR = cfg.dirs.photos_dir
PHOTOS_MISSION_DIR = PHOTOS_DIR / f"mission_{datetime.now().strftime('Mission_%Y-%m-%d_%H-%M')}"
os.makedirs(PHOTOS_MISSION_DIR, exist_ok=True)
filename = f"IMG_1_{time.time()}.jpg"


picam = Picamera2()
config = picam.create_still_configuration(main={"size": (2304, 1296)})
config["main"]["quality"] = 100

picam.configure(config)

picam.start()
picam.set_controls({
    "AfMode": 2,          # 0: off (z LensPosition na 0 będzie nieskończoność), 1: Single-shot, 2: Auto - tryb ostrości
    "AfMetering": 0,        # 0: Matrix , 1: Center-weighted, 2: spot - tryb pomiaru ostrości
    "ExposureTime": 10000,   # 1000us = 1/1000s (idealne dla drona)
    "AfRange": 1   
    #"AnalogueGain": 2.0     # Odpowiednik ISO (zwiększ, jeśli zdjęcia są za ciemne)
})

time.sleep(1)

i=0
t = time.time()

while i<50:
    timestamp = datetime.now().strftime("%m-%d_%H:%M:%S:%f")[:-4]
    filename = f"IMG_{i:03d}_{timestamp}.jpg"
    picam.capture_file(PHOTOS_MISSION_DIR/filename)
    time.sleep(0.5)
    i+=1
    print(f"Zrobiono zdjęcie {i}/50")

print(f"Zrobiono 50 zdjęć w {time.time() - t} sekund")