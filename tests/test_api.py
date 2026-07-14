from __future__ import annotations

import json
import logging
import math
from pathlib import Path

from fastapi.testclient import TestClient
import pytest

from robot_map_planner.api import create_app
from robot_map_planner.config import Settings
from robot_map_planner.navigation import NavBridgeError


class FakeNavigationClient:
    def __init__(self) -> None:
        self.points = None

    def current_pose(self):
        return {"x": 1.25, "y": -0.5, "z": 0.0, "ox": 0.0, "oy": 0.0, "oz": 0.0, "ow": 1.0, "localized": True}

    def execution_status(self):
        return {"status": "idle", "current_waypoint": 0, "total_waypoints": 0}

    def start_path(self, points):
        self.points = points
        return {"status": "queued", "current_waypoint": 0, "total_waypoints": len(points) - 1}


def test_import_form_defaults_match_number_steps(tmp_path: Path) -> None:
    app = create_app(Settings(tmp_path / "data", (tmp_path,), "127.0.0.1", 28200))
    with TestClient(app) as client:
        html = client.get("/").text
        app_js = client.get("/static/app.js").text
        styles = client.get("/static/styles.css").text

    assert 'id="resolution" type="number" value="0.10" min="0.01" step="0.01"' in html
    assert 'id="cost-scaling" type="number" value="5.0" min="0.5" step="0.5"' in html
    assert '<fieldset id="config-fields" disabled>' in html
    assert 'id="select-pcd-source"' in html
    assert 'id="config-form"' in html
    assert 'id="brush-cursor" class="brush-cursor"' in html
    assert 'id="start-yaw" type="number" value="0" step="1"' in html
    assert 'id="goal-yaw" type="number" value="0" step="1"' in html
    assert 'id="export-path" disabled' in html
    assert 'id="follow-path" class="warning" disabled' in html
    assert 'id="point-spacing"' not in html
    assert "state.brush*2*scale" in app_js
    assert "state.editMode==='boundary'||state.editTool!=='brush'" in app_js
    assert "window.confirm" in app_js
    assert "method:'DELETE'" in app_js
    assert "data-delete-map" in app_js
    assert "formatTimestamp(map.created_at)" in app_js
    assert "按参数创建新地图" in app_js
    assert "原地图保持不变" in app_js
    assert "直接覆盖当前地图" not in app_js
    assert "start_yaw:state.startYaw" in app_js
    assert "goal_yaw:state.goalYaw" in app_js
    assert "JSON.stringify(state.path" in app_js
    assert "quaternionYaw" in app_js
    assert "state.path.slice(1,-1)" in app_js
    assert "prominent:true" in app_js
    assert "/api/v1/navigation/pose" in app_js
    assert "/api/v1/navigation/follow-path" in app_js
    assert "error.code=body?.error?.code" in app_js
    assert "机器人将按照当前规划路径实际移动" in app_js
    assert ".brush-cursor{" in styles
    assert ".danger{" in styles


def test_navigation_api_uses_injected_client(tmp_path: Path) -> None:
    navigation = FakeNavigationClient()
    app = create_app(Settings(tmp_path / "data", (tmp_path,), "127.0.0.1", 28200), navigation)
    points = [
        {"x": 1.25, "y": -0.5, "z": 0, "ox": 0, "oy": 0, "oz": 0, "ow": 1, "mode": 1},
        {"x": 2.0, "y": -0.5, "z": 0, "ox": 0, "oy": 0, "oz": 0, "ow": 1, "mode": 1},
    ]
    with TestClient(app) as client:
        pose = client.get("/api/v1/navigation/pose")
        execution = client.get("/api/v1/navigation/execution")
        follow = client.post(
            "/api/v1/navigation/follow-path",
            json={"version_id": "ver_test", "points": points},
        )

    assert pose.status_code == 200
    assert pose.json()["localized"] is True
    assert execution.json()["status"] == "idle"
    assert follow.status_code == 202
    assert follow.json()["status"] == "queued"
    assert navigation.points == points


def test_navigation_api_preserves_bridge_error_code(tmp_path: Path) -> None:
    navigation = FakeNavigationClient()
    navigation.current_pose = lambda: (_ for _ in ()).throw(
        NavBridgeError("NAV_BRIDGE_OFFLINE", "NavBridge is offline", status_code=503)
    )
    app = create_app(Settings(tmp_path / "data", (tmp_path,), "127.0.0.1", 28200), navigation)
    with TestClient(app) as client:
        response = client.get("/api/v1/navigation/pose")

    assert response.status_code == 503
    assert response.json()["error"]["code"] == "NAV_BRIDGE_OFFLINE"


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
        planned = client.post(
            f"/api/v1/versions/{map_data['active_version_id']}/plan",
            json={
                "start": [1.0, 1.0],
                "goal": [3.0, 1.0],
                "start_yaw": math.pi / 2.0,
                "goal_yaw": -math.pi / 2.0,
            },
        )
        assert planned.status_code == 200, planned.text
        path = planned.json()["points"]
        assert len(path) == 2
        assert all(set(point) == {"x", "y", "z", "ox", "oy", "oz", "ow", "mode"} for point in path)
        assert path[0]["oz"] == pytest.approx(math.sqrt(0.5))
        assert path[-1]["oz"] == pytest.approx(-math.sqrt(0.5))
        assert all(point["mode"] == 1 for point in path)
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


def test_recompile_map_creates_copy_and_preserves_original(
    tmp_path: Path, ascii_pcd: Path, caplog
) -> None:
    app = create_app(Settings(tmp_path / "data", (tmp_path,), "127.0.0.1", 28200))
    with TestClient(app) as client:
        with ascii_pcd.open("rb") as stream:
            imported = client.post(
                "/api/v1/maps/import",
                data={"name": "original"},
                files={"file": ("map.pcd", stream, "application/octet-stream")},
            ).json()
        draft = client.post(f"/api/v1/maps/{imported['id']}/drafts").json()
        original_version_id = imported["active_version_id"]
        with caplog.at_level(logging.INFO):
            response = client.post(
                f"/api/v1/maps/{imported['id']}/recompile",
                json={
                    "name": "recompiled",
                    "resolution": 0.20,
                    "obstacle_min_height": 0.10,
                    "obstacle_max_height": 1.50,
                    "min_points_per_cell": 1,
                    "hard_clearance": 0.20,
                    "inflation_radius": 0.40,
                    "cost_scaling": 4.0,
                },
            )
            maps = client.get("/api/v1/maps").json()
            preserved_draft = client.get(f"/api/v1/drafts/{draft['id']}")
            preserved_version = client.get(f"/api/v1/versions/{original_version_id}/grid/costmap")
            invalid_recompile = client.post(
                f"/api/v1/maps/{imported['id']}/recompile",
                json={"name": "invalid", "hard_clearance": 0.50, "inflation_radius": 0.25},
            )
            preserved_original = client.get(f"/api/v1/maps/{imported['id']}").json()

    assert response.status_code == 201, response.text
    recompiled = response.json()
    assert recompiled["id"] != imported["id"]
    assert recompiled["name"] == "recompiled"
    assert recompiled["source_sha256"] == imported["source_sha256"]
    assert recompiled["build_config"]["resolution"] == 0.20
    assert recompiled["cost_config"]["inflation_radius"] == 0.40
    assert recompiled["active_version_id"] != original_version_id
    assert len(recompiled["versions"]) == 1
    assert recompiled["created_at"] != imported["created_at"]
    assert len(maps) == 2
    assert preserved_draft.status_code == 200
    assert preserved_version.status_code == 200
    assert invalid_recompile.status_code == 400
    assert preserved_original["active_version_id"] == original_version_id
    assert preserved_original["name"] == "original"
    assert len(preserved_original["versions"]) == 1
    assert sorted(path.name for path in (tmp_path / "data" / "maps").iterdir()) == sorted(
        [imported["id"], recompiled["id"]]
    )
    assert "Creating recompiled map copy" in caplog.text
    assert "Created recompiled map copy" in caplog.text
    assert f"source_map_id={imported['id']}" in caplog.text
    assert f"new_map_id={recompiled['id']}" in caplog.text


def test_delete_map_removes_metadata_and_files(tmp_path: Path, ascii_pcd: Path, caplog) -> None:
    app = create_app(Settings(tmp_path / "data", (tmp_path,), "127.0.0.1", 28200))
    with TestClient(app) as client:
        with ascii_pcd.open("rb") as stream:
            imported = client.post(
                "/api/v1/maps/import",
                data={"name": "delete-me"},
                files={"file": ("map.pcd", stream, "application/octet-stream")},
            ).json()
        draft = client.post(f"/api/v1/maps/{imported['id']}/drafts").json()
        map_dir = tmp_path / "data" / "maps" / imported["id"]
        assert map_dir.is_dir()
        with caplog.at_level(logging.INFO):
            response = client.delete(f"/api/v1/maps/{imported['id']}")
            maps = client.get("/api/v1/maps").json()
            deleted_map = client.get(f"/api/v1/maps/{imported['id']}")
            deleted_draft = client.get(f"/api/v1/drafts/{draft['id']}")

    assert response.status_code == 200
    assert response.json() == {"deleted": True, "map_id": imported["id"]}
    assert maps == []
    assert deleted_map.status_code == 404
    assert deleted_draft.status_code == 404
    assert not map_dir.exists()
    assert "Deleted map" in caplog.text
    assert "versions=1 drafts=1" in caplog.text
