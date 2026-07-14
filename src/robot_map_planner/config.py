from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path


@dataclass(frozen=True)
class Settings:
    data_dir: Path
    import_roots: tuple[Path, ...]
    host: str
    port: int
    nav_bridge_url: str = "http://127.0.0.1:28180"
    nav_request_timeout: float = 5.0
    nav_pose_timeout: float = 5.0
    nav_waypoint_timeout: float = 300.0
    nav_poll_interval: float = 0.5
    nav_start_tolerance: float = 0.75

    @classmethod
    def from_env(cls) -> "Settings":
        data_dir = Path(os.environ.get("RMP_DATA_DIR", "./data")).expanduser().resolve()
        roots_value = os.environ.get("RMP_IMPORT_ROOTS", "/data/imports")
        roots = tuple(Path(item).expanduser().resolve() for item in roots_value.split(os.pathsep) if item.strip())
        return cls(
            data_dir=data_dir,
            import_roots=roots,
            host=os.environ.get("RMP_HOST", "0.0.0.0"),
            port=int(os.environ.get("RMP_PORT", "28200")),
            nav_bridge_url=os.environ.get("RMP_NAV_BRIDGE_URL", "http://127.0.0.1:28180"),
            nav_request_timeout=float(os.environ.get("RMP_NAV_REQUEST_TIMEOUT", "5.0")),
            nav_pose_timeout=float(os.environ.get("RMP_NAV_POSE_TIMEOUT", "5.0")),
            nav_waypoint_timeout=float(os.environ.get("RMP_NAV_WAYPOINT_TIMEOUT", "300.0")),
            nav_poll_interval=float(os.environ.get("RMP_NAV_POLL_INTERVAL", "0.5")),
            nav_start_tolerance=float(os.environ.get("RMP_NAV_START_TOLERANCE", "0.75")),
        )
