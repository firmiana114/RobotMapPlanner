from __future__ import annotations

from pathlib import Path

import pytest

from robot_map_planner.errors import PlannerError
from robot_map_planner.storage import MapStore


def configs() -> tuple[dict, dict]:
    return (
        {"resolution": 0.25, "obstacle_min_height": 0.2, "obstacle_max_height": 1.0, "min_points_per_cell": 1},
        {"hard_clearance": 0.0, "inflation_radius": 0.25, "cost_scaling": 3.0},
    )


def test_import_edit_publish_and_plan(tmp_path: Path, ascii_pcd: Path) -> None:
    store = MapStore(tmp_path / "data")
    build, cost = configs()
    imported = store.import_map(ascii_pcd, name="test", build_config=build, cost_config=cost)
    assert imported["source"]["declared_points"] == 13
    assert imported["active_version_id"]

    draft = store.create_draft(imported["id"])
    updated = store.patch_draft(
        draft["id"],
        0,
        {"operation": "set_cells", "mode": "occupied", "cells": [[8, 8], [8, 9]]},
    )
    assert updated["revision"] == 1
    with pytest.raises(PlannerError) as stale:
        store.patch_draft(draft["id"], 0, {"operation": "set_cells", "mode": "free", "cells": [[8, 8]]})
    assert stale.value.code == "VERSION_CONFLICT"
    store.move_history(draft["id"], "undo")
    store.move_history(draft["id"], "redo")
    validation = store.validate_draft(draft["id"])
    assert validation["traversable_cells"] > 0
    assert validation["cost_rebuild_mode"] == "incremental"
    _, _, final, incremental, _ = store.compile_draft(draft["id"])
    from robot_map_planner import _core

    full = bytes(_core.build_costmap(final, imported["meta"], imported["cost_config"]))
    assert incremental == full
    published = store.publish_draft(draft["id"])
    assert len(published["versions"]) == 2


def test_plan_reports_invalid_goal(tmp_path: Path, ascii_pcd: Path) -> None:
    store = MapStore(tmp_path / "data")
    build, cost = configs()
    imported = store.import_map(ascii_pcd, name="test", build_config=build, cost_config=cost)
    with pytest.raises(PlannerError) as error:
        store.plan(imported["active_version_id"], {"start": [1.0, 1.0], "goal": [99.0, 99.0]})
    assert error.value.code == "GOAL_OUTSIDE"
