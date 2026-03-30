# configuration

Contains configuration parsing and environment initialization.

## Files
- `config_loader.py`: initializes `Config`, loads `config.toml`, maps dataclasses, and executes preflight simulation.
- `config.toml`: runtime settings for MAVLink, drop physics, camera, and paths.

## Usage
- Edit `config.toml` to set device/baud, drop parameters or directories.
- It auto-recomputes offsets at startup via `run_preflight_simulation`.
