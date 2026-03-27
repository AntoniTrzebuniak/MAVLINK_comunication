from Application.Services.MatekService import MatekService
from Application.configuration.config_loader import cfg

print(cfg.mav.device)
print(cfg.mav.baud)
drone = MatekService(device = cfg.mav.device, baud = cfg.mav.baud)

