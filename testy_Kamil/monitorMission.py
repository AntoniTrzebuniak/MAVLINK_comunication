import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
 
import time
import math
 
from Application.Services.MatekService import MatekService
from Application.configuration.config_loader import cfg
from Application.Logger.log_module import get_logger
from Application.Services.MissionService import MissionService

drone = MatekService(device=cfg.mav.device, baud=cfg.mav.baud)    
drone_mission = MissionService(drone) 

logger = get_logger(__name__)
logger.info("Inicjalizacja programu")

 
  
while True:

    drone.monitor_all()
    time.sleep(1)
 
