# Application Module

Contains the core application logic and support modules.

## Structure
- `calc_drop_translation/`: drop trajectory simulation and math core.
- `configuration/`: config loader and config.toml definitions.
- `Logger/`: logging utilities.
- `media/`: directories for logs/media outputs.
- `Services/`: MAVLink and camera service wrappers.

## Usage
1. Configure `Application/configuration/config.toml`.
2. Use `Application.Services.MatekService` to connect to MAVLink.
3. Use `Application.Services.CameraService` for image capture and camera control.
4. `calc_drop_translation.run_preflight_simulation` computes drop offsets (read-only approach, no direct `update_translations`).
