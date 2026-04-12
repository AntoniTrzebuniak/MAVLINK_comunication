import sys
import os
from pathlib import Path

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from Application.Services.ImageMosaicService import ImageMosaicService
from Application.configuration.config_loader import cfg


def get_latest_photo_mission_dir() -> Path:
    photos_root = cfg.dirs.photos_dir
    if not photos_root.exists() or not photos_root.is_dir():
        raise FileNotFoundError(f"Brak katalogu zdjęć: {photos_root}")

    mission_dirs = [d for d in photos_root.iterdir() if d.is_dir()]
    if not mission_dirs:
        raise FileNotFoundError(f"Brak katalogów misji w: {photos_root}")

    latest_dir = max(mission_dirs, key=lambda d: d.stat().st_mtime)
    return latest_dir


if __name__ == "__main__":
    mission_dir = get_latest_photo_mission_dir()
    print(f"Wybrano najnowszy katalog misji: {mission_dir}")

    service = ImageMosaicService(mission_dir)
    output_path = service.build_mosaic()
    print(f"Mozaika zapisana jako: {output_path}")


