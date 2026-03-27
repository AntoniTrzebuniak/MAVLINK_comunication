from __future__ import annotations
import logging
import sys
from datetime import datetime
from pathlib import Path
from Application.configuration.config_loader import cfg     # mandatory


def get_logger(name: str) -> logging.Logger:
    """
    Konfiguruje logger korzystając z ustawień w config.toml.

    Usage example:
        self.logger = get_logger(__name__)
    """
    logger = logging.getLogger(name)
    
    # Jeśli logger ma już handlery, nie dodawaj ich ponownie (ważne przy re-inicjalizacji)
    if logger.handlers:
        return logger

    # Ustawiamy bazowy poziom na DEBUG, żeby handlery mogły same decydować co odfiltrować
    logger.setLevel(logging.DEBUG)

    # Formatowanie: dodajemy milisekundy (ważne przy analizie lotu!)
    formatter = logging.Formatter(
        "%(asctime)s.%(msecs)03d | %(name)-15s | %(levelname)-8s | %(message)s",
        datefmt="%H:%M:%S"
    )

    # 1. FILE HANDLER (Zawsze loguje wszystko do pliku do późniejszej analizy)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file_name = f"flight_{timestamp}.log"
    log_file_path = cfg.dirs.logs_dir / log_file_name
    file_handler = logging.FileHandler(log_file_path, mode="a", encoding="utf-8")
    file_handler.setFormatter(formatter)
    file_handler.setLevel(logging.DEBUG) 

    # 2. STREAM HANDLER (Konsola - tutaj sterujemy czy chcemy widzieć DEBUG)
    stream_handler = logging.StreamHandler(sys.stdout)
    stream_handler.setFormatter(formatter)
    
    # Decyzja o poziomie na podstawie konfiguracji
    # Załóżmy, że w config.toml masz sekcję [debug] enabled = true/false
    # Jeśli cfg.debug_enabled jest False, konsola pokaże tylko INFO i wyżej
    if hasattr(cfg, 'debug_enabled') and cfg.debug_enabled:
        stream_handler.setLevel(logging.DEBUG)
    else:
        stream_handler.setLevel(logging.INFO)

    logger.addHandler(file_handler)
    logger.addHandler(stream_handler)

    return logger