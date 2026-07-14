from __future__ import annotations

import json
import logging
from pathlib import Path

from fastapi.testclient import TestClient

from robot_map_planner.api import create_app
from robot_map_planner.config import Settings


def test_import_form_defaults_match_number_steps(tmp_path: Path) -> None:
    app = create_app(Settings(tmp_path / "data", (tmp_path,), "127.0.0.1", 28200))
    with TestClient(app) as client:
        html = client.get("/").text

    assert 'id="resolution" type="number" value="0.10" min="0.01" step="0.01"' in html
    assert 'id="cost-scaling" type="number" value="5.0" min="0.5" step="0.5"' in html


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


def test_import_rejects_invalid_costmap_parameters_with_context(
    tmp_path: Path, ascii_pcd: Path, caplog
) -> None:
    app = create_app(Settings(tmp_path / "data", (tmp_path,), "127.0.0.1", 28200))
    with TestClient(app) as client, caplog.at_level(logging.WARNING):
        with ascii_pcd.open("rb") as stream:
            response = client.post(
                "/api/v1/maps/import",
                data={
                    "name": "invalid-costmap",
                    "config_json": json.dumps({"hard_clearance": 0.25, "inflation_radius": 0.0}),
                },
                files={"file": ("map.pcd", stream, "application/octet-stream")},
            )

    assert response.status_code == 400
    assert response.json() == {
        "error": {
            "code": "INVALID_CONFIG",
            "message": "inflation_radius must be greater than or equal to hard_clearance (got 0.0 < 0.25)",
        }
    }
    assert "Rejected map import" in caplog.text
    assert "API request rejected" in caplog.text
