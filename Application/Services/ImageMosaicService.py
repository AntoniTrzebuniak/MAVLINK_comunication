import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

try:
    import cv2
except ImportError as exc:
    raise ImportError(
        "OpenCV jest wymagane do sklejania zdjęć. Zainstaluj je: pip install opencv-python"
    ) from exc

from Application.configuration.config_loader import cfg


@dataclass
class PhotoPosition:
    filename: str
    index: int
    lat: float
    lon: float
    alt: float
    roll: float
    pitch: float
    yaw: float


class ImageMosaicService:
    """Szybki moduł do sklejania zdjęć misji w mapę.

    Algorytm wykorzystuje informacje z pliku CSV i przybliżoną transformację
    geograficzną: zdjęcia są rozmieszczane w płaszczyźnie na podstawie lat/lon,
    a ich skala jest obliczana z wysokości oraz ogniskowej kamery.
    """

    def __init__(
        self,
        mission_dir: Path,
        csv_filename: str = "photos_position.csv",
        output_filename: str = "mosaic.jpg",
        border_px: int = 200,
        max_output_dim: int = 14000,
    ):
        self.mission_dir = Path(mission_dir)
        self.csv_file = self.mission_dir / csv_filename
        self.output_file = self.mission_dir / output_filename
        self.border_px = border_px
        self.max_output_dim = max_output_dim
        self.camera_focal_px = float(cfg.camera.K[0, 0]) if hasattr(cfg, "camera") else None

    def read_positions(self) -> list[PhotoPosition]:
        if not self.csv_file.exists():
            raise FileNotFoundError(f"Brak pliku CSV: {self.csv_file}")

        with open(self.csv_file, newline="", encoding="utf-8") as fp:
            reader = csv.DictReader(fp)
            rows = []
            for row in reader:
                if not row.get("Filename"):
                    continue
                rows.append(
                    PhotoPosition(
                        filename=row["Filename"].strip(),
                        index=int(row.get("Index", 0)),
                        lat=float(row.get("Lat", 0.0)),
                        lon=float(row.get("Lon", 0.0)),
                        alt=float(row.get("Alt", 0.0)),
                        roll=float(row.get("Roll", 0.0)),
                        pitch=float(row.get("Pitch", 0.0)),
                        yaw=float(row.get("Yaw", 0.0)),
                    )
                )
        if not rows:
            raise ValueError(f"Plik CSV nie zawiera pozycji: {self.csv_file}")
        rows.sort(key=lambda item: item.index)
        return rows

    @staticmethod
    def geo_to_meters(lat: float, lon: float, origin_lat: float, origin_lon: float) -> tuple[float, float]:
        """Prosta transformacja geograficzna: stopnie -> metry."""
        avg_lat = np.deg2rad(origin_lat)
        dx = (lon - origin_lon) * 111_320 * np.cos(avg_lat)
        dy = (lat - origin_lat) * 110_574
        return dx, dy

    def meters_per_pixel(self, altitude: float) -> float:
        """Skala obrazu w metrach na piksel przy przybliżeniu pinhole."""
        if self.camera_focal_px and self.camera_focal_px > 0:
            return altitude / self.camera_focal_px
        return 0.1

    def build_mosaic(self, annotate: bool = True) -> Path:
        positions = self.read_positions()
        origin = positions[0]
        centers_px = []
        images = []

        for item in positions:
            image_path = self.mission_dir / item.filename
            if not image_path.exists():
                continue
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if image is None:
                continue

            mpp = self.meters_per_pixel(item.alt)
            if mpp <= 0:
                raise ValueError("Nieprawidłowa skala metryczna dla obrazu")

            dx, dy = self.geo_to_meters(item.lat, item.lon, origin.lat, origin.lon)
            pixel_scale = 1.0 / mpp
            center_x = dx * pixel_scale
            center_y = -dy * pixel_scale

            images.append((item, image, center_x, center_y))
            centers_px.append((center_x, center_y, image.shape[1], image.shape[0]))

        if not images:
            raise ValueError("Brak dostępnych plików JPG do sklejania")

        min_x = min(cx - w / 2 for cx, _, w, _ in centers_px) - self.border_px
        max_x = max(cx + w / 2 for cx, _, w, _ in centers_px) + self.border_px
        min_y = min(cy - h / 2 for _, cy, _, h in centers_px) - self.border_px
        max_y = max(cy + h / 2 for _, cy, _, h in centers_px) + self.border_px

        width = int(np.ceil(max_x - min_x))
        height = int(np.ceil(max_y - min_y))
        if width <= 0 or height <= 0:
            raise ValueError("Nieprawidłowy rozmiar mozaiki")

        if max(width, height) > self.max_output_dim:
            scale = self.max_output_dim / max(width, height)
            width = int(width * scale)
            height = int(height * scale)
        else:
            scale = 1.0

        offset_x = -min_x
        offset_y = -min_y
        mosaic = np.zeros((height, width, 3), dtype=np.float32)
        weights = np.zeros((height, width), dtype=np.float32)

        for item, image, center_x, center_y in images:
            h_img, w_img = image.shape[:2]
            if scale != 1.0:
                image = cv2.resize(image, (int(w_img * scale), int(h_img * scale)), interpolation=cv2.INTER_AREA)
                h_img, w_img = image.shape[:2]

            x0 = int(round((center_x + offset_x) * scale - w_img / 2))
            y0 = int(round((center_y + offset_y) * scale - h_img / 2))
            x1 = x0 + w_img
            y1 = y0 + h_img

            if x1 <= 0 or y1 <= 0 or x0 >= width or y0 >= height:
                continue

            x0_clip = max(x0, 0)
            y0_clip = max(y0, 0)
            x1_clip = min(x1, width)
            y1_clip = min(y1, height)
            img_x0 = x0_clip - x0
            img_y0 = y0_clip - y0
            img_x1 = img_x0 + (x1_clip - x0_clip)
            img_y1 = img_y0 + (y1_clip - y0_clip)

            region = mosaic[y0_clip:y1_clip, x0_clip:x1_clip]
            alpha = np.ones((y1_clip - y0_clip, x1_clip - x0_clip), dtype=np.float32)
            mosaic[y0_clip:y1_clip, x0_clip:x1_clip] = region + image[img_y0:img_y1, img_x0:img_x1].astype(np.float32) * alpha[:, :, None]
            weights[y0_clip:y1_clip, x0_clip:x1_clip] += alpha

        mask = weights > 0
        if not np.any(mask):
            raise ValueError("Mosaic build failed: no pixels were written")

        mosaic[mask] /= weights[mask][:, None]
        mosaic_uint8 = np.clip(mosaic, 0, 255).astype(np.uint8)

        if annotate:
            for item, _, center_x, center_y in images:
                px = int(round((center_x + offset_x) * scale))
                py = int(round((center_y + offset_y) * scale))
                if 0 <= px < width and 0 <= py < height:
                    cv2.circle(mosaic_uint8, (px, py), 10, (0, 200, 255), thickness=2)
                    cv2.putText(
                        mosaic_uint8,
                        f"{item.index}",
                        (px + 12, py - 12),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (255, 255, 255),
                        thickness=2,
                        lineType=cv2.LINE_AA,
                    )

        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.output_file), mosaic_uint8)
        return self.output_file

    @classmethod
    def stitch_mission_dir(cls, mission_dir: Path, **kwargs: Any) -> Path:
        service = cls(mission_dir, **kwargs)
        return service.build_mosaic()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Sklej zdjęcia misji w jednolitą mozaikę.")
    parser.add_argument("mission_dir", help="Ścieżka do folderu z JPG i photos_position.csv")
    parser.add_argument("--output", default="mosaic.jpg", help="Nazwa pliku wyjściowego")
    args = parser.parse_args()

    mosaic = ImageMosaicService(Path(args.mission_dir), output_filename=args.output).build_mosaic()
    print(f"Mozaika zapisana w: {mosaic}")
