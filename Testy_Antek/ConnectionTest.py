import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from Application.Services.MatekService import MatekService
from Application.configuration.config_loader import cfg

#print(cfg.mav.device)
#print(cfg.mav.baud)
#drone = MatekService(device = cfg.mav.device, baud = cfg.mav.baud)

drone = MatekService(device = "tcp:192.168.0.109:5763")
print("Connected to drone")
