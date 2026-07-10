from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture()
def ascii_pcd(tmp_path: Path) -> Path:
    path = tmp_path / "map.pcd"
    points = [
        (0, 0, 0), (0, 0, 0.5), (0, 4, 0), (0, 4, 0.5),
        (4, 0, 0), (4, 0, 0.5), (4, 4, 0), (4, 4, 0.5),
        (2, 0, 0), (2, 4, 0), (0, 2, 0), (4, 2, 0),
        (2, 2, 0.5),
    ]
    header = (
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        f"COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n"
    )
    path.write_text(header + "".join(f"{x} {y} {z}\n" for x, y, z in points), encoding="ascii")
    return path
