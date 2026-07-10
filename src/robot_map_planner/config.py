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
        )
