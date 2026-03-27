import tomllib
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class MAVLinkConfig:
    device: str
    baud: int
    mav_version: int

@dataclass(frozen=True)
class ItemConfig:
    mass: int
    cd: float
    drop_course: int

@dataclass(frozen=True)
class DropsConfig:
    cruise_speed: int
    wind_speed: int
    wind_direction: int
    beacon: ItemConfig
    bottle: ItemConfig

@dataclass(frozen=True)
class CameraConfig:
    resolution: tuple[int, int]

@dataclass(frozen=True)
class DirsConfig:
    base_dir: Path
    logs_dir: Path


class Config:
    def __init__(self, file_name: str = "config.toml"):
        # 1. Automatyczne wyliczanie ROOT_DIR (zakładając, że loader jest w /src/ lub głównym)
        self.ROOT_DIR = Path(__file__).resolve().parent

        config_path = self.ROOT_DIR / file_name

        with open(config_path, "rb") as f:
            data = tomllib.load(f)

        self.debug_enabled = data.get("system", {}).get("debug_mode", False)

        # 2. Mapowanie sekcji MAVLink
        mav_data = data.get("mavlink", {})
        self.mav = MAVLinkConfig(
            device=mav_data.get("device", "/dev/ttyAMA2"),
            baud=mav_data.get("baud", 115200),
            mav_version=mav_data.get("mav_version", 2)
        )

        # 3. Mapowanie sekcji Drops
        drops_data = data.get("drops", {})
        beacon_data = drops_data.get("beacon", {})
        bottle_data = drops_data.get("bottle", {})
        
        self.drops = DropsConfig(
            cruise_speed=drops_data.get("cruise_speed", 20),
            wind_speed=drops_data.get("wind_speed", 0),
            wind_direction=drops_data.get("wind_direction", 0),
            beacon=ItemConfig(
                mass=beacon_data.get("mass", 155),
                cd=beacon_data.get("cd", 0.27),
                drop_course=beacon_data.get("drop_course", 0)
            ),
            bottle=ItemConfig(
                mass=bottle_data.get("mass", 255),
                cd=bottle_data.get("cd", 0.3),
                drop_course=bottle_data.get("drop_course", 0)
            )
        )

        # 4. Mapowanie kamery (konwersja listy z TOML na tuple)
        camera_data = data.get("camera", {})
        self.camera = CameraConfig(
            resolution=tuple(camera_data.get("resolution", [1920, 1080]))
        )

        # 5. Dynamiczne ścieżki
        dirs_data = data.get("dirs", {})
        logs_dir_name = dirs_data.get("logs_dir", "logs")
        self.dirs = DirsConfig(
            base_dir=self.ROOT_DIR,
            logs_dir=self.ROOT_DIR / logs_dir_name
        )

        self.dirs.logs_dir.mkdir(parents=True, exist_ok=True)


# Inicjalizacja
try:
    cfg = Config()
except FileNotFoundError as exc:
    raise FileNotFoundError("Błąd: Nie znaleziono pliku config.toml!") from exc
