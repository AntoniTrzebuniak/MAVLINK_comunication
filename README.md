# Aligatorpy

Autonomous drone mission system for payload delivery and surveillance using MAVLink protocol.

## Overview

This project implements a complete drone mission control system with:
- MAVLink communication for drone control
- Camera capture and image processing
- Trajectory simulation for accurate payload drops
- Mission planning and execution
- Raspberry Pi deployment configuration

## Project Structure

- `Application/`: Core application modules
  - `calc_drop_translation/`: Physics simulation for drop trajectories
  - `configuration/`: Configuration loading and management
  - `Logger/`: Logging utilities
  - `media/`: Output directories for logs, photos, videos
  - `Services/`: MAVLink and camera service interfaces

- `Raspberry_configuration/`: Raspberry Pi setup and configuration files
- `tests/`: Test scripts for validation
- `*.py`: Main entry points and utilities

## Quick Start

### Prerequisites
- Python 3.8+
- Raspberry Pi with camera module
- MAVLink-compatible drone (e.g., Matek systems)

### Setup
1. Clone the repository
2. Configure Raspberry Pi using `Raspberry_configuration/Pipeline.md`
3. Edit `Application/configuration/config.toml` for your hardware
4. Install dependencies: `pip install -r Raspberry_configuration/config_files/requirements.txt`

### Running
```bash
# Test camera and MAVLink connection
python3 Camera_test.py

# Run main mission
python3 main.py
```

## Configuration

Edit `Application/configuration/config.toml` to set:
- MAVLink device and baud rate
- Drop physics parameters (mass, drag coefficients)
- Camera resolution
- Mission directories

The system automatically runs pre-flight simulations to compute drop offsets.

## Testing

Run tests from the `tests/` directory:
```bash
python3 tests/test.py
```

## Deployment

Use the Raspberry Pi configuration files to set up the target hardware:
- Copy boot configs from `Raspberry_configuration/config_files/`
- Enable UART and camera interfaces
- Install required packages

## Contributing

1. Follow the existing code structure
2. Add tests for new features
3. Update READMEs in relevant folders
4. Ensure configuration changes are backward compatible

## License

[Add license information here]