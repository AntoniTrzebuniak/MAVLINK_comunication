# config_files

Low-level Raspberry Pi configuration fragments.

### TODO 
    - config files needs to be updated

## Files
- `99-tty-raw.rules`: serial device permissions.
- `cmdline.txt` and `config.txt`: boot-time kernel/camera configuration.
- `requirements.txt`, `apt_packages.txt`, `manual_packages.txt`: package install lists.

## Usage
- Copy to `/boot` or `/etc` as documented in `Pipeline.md`.
- Use as source for provisioning scripts and image builds.
