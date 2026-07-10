from __future__ import annotations

import json
from pathlib import Path

from fastapi.testclient import TestClient

from robot_map_planner.api import create_app
from robot_map_planner.config import Settings


def test_api_workflow(tmp_path: Path, ascii_pcd: Path) -> None:
    app = create_app(Settings(tmp_path / "data", (tmp_path,), "127.0.0.1", 28200))
    with TestClient(app) as client:
        health = client.get("/healthz")
        assert health.status_code == 200
        with ascii_pcd.open("rb") as stream:
            imported = client.post(
                "/api/v1/maps/import",
                data={"name": "api-map", "config_json": json.dumps({"resolution": 0.25, "hard_clearance": 0.0, "inflation_radius": 0.25})},
                files={"file": ("map.pcd", stream, "application/octet-stream")},
            )
        assert imported.status_code == 201, imported.text
        map_data = imported.json()
        draft = client.post(f"/api/v1/maps/{map_data['id']}/drafts")
        assert draft.status_code == 201
        patched = client.patch(
            f"/api/v1/drafts/{draft.json()['id']}",
            json={"revision": 0, "operation": "set_cells", "mode": "occupied", "cells": [[5, 5]]},
        )
        assert patched.status_code == 200
        draft_grid = client.get(f"/api/v1/drafts/{draft.json()['id']}/grid/final")
        assert draft_grid.status_code == 200
        assert "X-RMP-Meta" in draft_grid.headers
        conflict = client.patch(
            f"/api/v1/drafts/{draft.json()['id']}",
            json={"revision": 0, "operation": "set_cells", "mode": "free", "cells": [[5, 5]]},
        )
        assert conflict.status_code == 409
        assert conflict.json()["error"]["code"] == "VERSION_CONFLICT"
        published = client.post(f"/api/v1/drafts/{draft.json()['id']}/publish")
        assert published.status_code == 200
        version_id = published.json()["active_version_id"]
        grid = client.get(f"/api/v1/versions/{version_id}/grid/costmap")
        assert grid.status_code == 200
        assert "X-RMP-Meta" in grid.headers
        invalid_threshold = client.post(
            f"/api/v1/versions/{version_id}/plan",
            json={"start": [1.0, 1.0], "goal": [2.0, 2.0], "max_traversable_cost": 253},
        )
        assert invalid_threshold.status_code == 422
