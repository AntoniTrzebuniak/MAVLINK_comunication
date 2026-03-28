import threading
import time
from Application.Services.MatekService import MatekService
from Application.Services.CameraService import CameraService # Załóżmy, że tam zapisałeś klasę
from Application.configuration.config_loader import cfg

drone = MatekService(device=cfg.mav.device, baud=cfg.mav.baud)
camera = CameraService(drone=drone)


camera_thread = threading.Thread(
    target=camera.image_capture_listener, 
    daemon=True
)
camera_thread.start()

print("System gotowy. Oczekiwanie na misję/wyzwalanie kamery...")

try:
    while True:
        # Tutaj może być inna logika, np. sprawdzanie statusu baterii
        time.sleep(1)
except KeyboardInterrupt:
    print("Zamykanie aplikacji...")