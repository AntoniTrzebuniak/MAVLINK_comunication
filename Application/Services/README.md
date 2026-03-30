# Services

Holds hardware service abstractions for MAVLink and camera operation.

## Files
- `MatekService.py`: MAVLink communication and drone control interface.
- `CameraService.py`: handles camera capture and trigger logic.
- `MissionService.py`: mission planning and execution logic (mission lifecycle orchestrator).

## Usage
- Initialize `MatekService` with `cfg.mav.device` and `cfg.mav.baud`.
- Pass `MatekService` instance to `CameraService` for image capture in mission loops.
