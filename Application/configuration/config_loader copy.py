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
    root_dir: Path
    logs_dir: Path
    config_dir: Path
    videos_dir: Path
    photos_dir: Path
    zones_dir: Path

@dataclass(frozen=True)
class ZonesPaths:
    search_zone_path: list[tuple[float, float]]

class Config:
    def __init__(self, file_name: str = "config.toml"):
        # 1. Ustalanie ścieżek bazowych
        self.ROOT_DIR = Path(__file__).resolve().parent.parent
        self.CONFIG_DIR = self.ROOT_DIR / "configuration"
        config_path = self.CONFIG_DIR / file_name

        try:
            with open(config_path, "rb") as f:
                data = tomllib.load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"KRYTYCZNY BŁĄD: Brak pliku konfiguracji w {config_path}")

        try:
            # System / Debug
            self.debug_enabled = data["system"]["debug_mode"]

            # 2. MAVLink (Strict)
            mav = data["mavlink"]
            self.mav = MAVLinkConfig(
                device=mav["device"],
                baud=mav["baud"],
                mav_version=mav["mav_version"]
            )

            # 3. Drops (Strict)
            drops = data["drops"]
            
            # Pomocnicza funkcja wewnętrzna dla przedmiotów (też strict)
            def parse_item(item_key):
                item = drops[item_key]
                return ItemConfig(
                    mass=item["mass"],
                    cd=item["cd"],
                    drop_course=item["drop_course"]
                )

            self.drops = DropsConfig(
                cruise_speed=drops["cruise_speed"],
                wind_speed=drops["wind_speed"],
                wind_direction=drops["wind_direction"],
                beacon=parse_item("beacon"),
                bottle=parse_item("bottle")
            )

            # 4. Kamera (Strict)
            cam = data["camera"]
            self.camera = CameraConfig(
                resolution=tuple(cam["resolution"])
            )

            # 5. Dirs (Strict)
            dirs = data["dirs"]
            self.dirs = DirsConfig(
                root_dir=self.ROOT_DIR,
                config_dir=self.CONFIG_DIR,
                logs_dir=self.ROOT_DIR / dirs["logs_dir"],
                videos_dir = self.ROOT_DIR / dirs["videos_dir"],
                photos_dir = self.ROOT_DIR / dirs["photos_dir"],
                zones_dir = self.ROOT_DIR / dirs["zones_dir"]
            )

            # Inicjalizacja środowiska
            self.dirs.logs_dir.mkdir(parents=True, exist_ok=True)
            self.dirs.videos_dir.mkdir(parents=True, exist_ok=True)
            self.dirs.photos_dir.mkdir(parents=True, exist_ok=True)
            zonesdata = data["zones"]
            self.zones = ZonesPaths(
                search_zone_path= zonesdata["search_zone_path"]
            )

        except KeyError as e:
            raise KeyError(f"BŁĄD KONFIGURACJI: W pliku {file_name} brakuje wymaganego klucza: {e}")
        except TypeError as e:
            raise TypeError(f"BŁĄD TYPU: Niepoprawny format danych w {file_name}: {e}")

# Przykład użycia
try:
    cfg = Config()
except Exception as e:
    print(f"Start drona przerwany: {e}")
    exit(1) # Zatrzymujemy program, bo bez konfigu lot jest niebezpieczny