from __future__ import annotations

import ast
import time

import pytest

from robot_map_planner.navigation import NavBridgeClient, NavBridgeError, trajectory_metrics


POSE = {"x": 1.0, "y": 2.0, "z": 0.0, "ox": 0.0, "oy": 0.0, "oz": 0.0, "ow": 1.0}


def test_current_pose_requires_healthy_bridge() -> None:
    calls = []

    def transport(method, path, form):
        calls.append((method, path, form))
        return {"ok": True}

    client = NavBridgeClient("http://bridge", transport=transport, pose_reader=lambda timeout: POSE)
    pose = client.current_pose()

    assert pose["localized"] is True
    assert pose["x"] == 1.0
    assert calls == [("GET", "/health", None)]


def test_current_pose_reports_offline_bridge() -> None:
    client = NavBridgeClient(
        "http://bridge",
        transport=lambda method, path, form: {"ok": False},
        pose_reader=lambda timeout: POSE,
    )
    with pytest.raises(NavBridgeError, match="health check") as captured:
        client.current_pose()
    assert captured.value.code == "NAV_BRIDGE_OFFLINE"


def test_current_pose_uses_nav_bridge_http_endpoint_by_default() -> None:
    calls = []

    def transport(method, path, form):
        calls.append((method, path, form))
        if path == "/health":
            return {"ok": True}
        return {**POSE, "localized": True, "age_seconds": 0.2}

    client = NavBridgeClient("http://bridge", transport=transport)

    pose = client.current_pose()

    assert pose["x"] == 1.0
    assert calls == [("GET", "/health", None), ("GET", "/current_pose", None)]


def test_current_pose_reports_nav_bridge_not_localized() -> None:
    def transport(method, path, form):
        if path == "/health":
            return {"ok": True}
        return {"localized": False, "message": "current pose is stale"}

    client = NavBridgeClient("http://bridge", transport=transport)

    with pytest.raises(NavBridgeError, match="stale") as captured:
        client.current_pose()
    assert captured.value.code == "ROBOT_NOT_LOCALIZED"


def test_path_rejects_robot_far_from_start() -> None:
    client = NavBridgeClient(
        "http://bridge",
        transport=lambda method, path, form: {"ok": True},
        pose_reader=lambda timeout: POSE,
        start_tolerance=0.25,
    )
    points = [
        {"x": 2.0, "y": 2.0, "ox": 0, "oy": 0, "oz": 0, "ow": 1},
        {"x": 3.0, "y": 2.0, "ox": 0, "oy": 0, "oz": 0, "ow": 1},
    ]
    with pytest.raises(NavBridgeError) as captured:
        client.start_path(points)
    assert captured.value.code == "ROBOT_START_MISMATCH"


def test_path_sends_each_waypoint_after_start_and_waits_for_success() -> None:
    sent = []

    def transport(method, path, form):
        if path == "/health":
            return {"ok": True}
        if path == "/go_to_async":
            sent.append(ast.literal_eval(form["task"]))
            return {"success": True}
        if path == "/go_to_status":
            return {"status": "3"}
        return {"status": "0"}

    client = NavBridgeClient(
        "http://bridge",
        transport=transport,
        pose_reader=lambda timeout: POSE,
        poll_interval=0.001,
        waypoint_timeout=0.1,
    )
    points = [
        {"x": 1.0, "y": 2.0, "ox": 0, "oy": 0, "oz": 0, "ow": 1},
        {"x": 2.0, "y": 2.0, "ox": 0, "oy": 0, "oz": 0, "ow": 1},
        {"x": 2.0, "y": 3.0, "ox": 0, "oy": 0, "oz": 0.707, "ow": 0.707},
    ]
    client.start_path(points)
    deadline = time.monotonic() + 1.0
    while client.execution_status()["status"] not in {"succeeded", "failed"} and time.monotonic() < deadline:
        time.sleep(0.005)

    assert client.execution_status()["status"] == "succeeded"
    assert sent == [(2.0, 2.0, 0.0, 0.0, 0.0, 1.0), (2.0, 3.0, 0.0, 0.0, 0.707, 0.707)]


def test_trajectory_metrics_use_distance_to_planned_polyline() -> None:
    metrics = trajectory_metrics(
        [{"x": 0.0, "y": 0.0}, {"x": 2.0, "y": 0.0}],
        [{"x": 0.0, "y": 1.0}, {"x": 1.0, "y": 1.0}, {"x": 2.0, "y": 1.0}],
    )

    assert metrics["sample_count"] == 3
    assert metrics["planned_length_m"] == pytest.approx(2.0)
    assert metrics["actual_length_m"] == pytest.approx(2.0)
    assert metrics["mean_error_m"] == pytest.approx(1.0)
    assert metrics["rms_error_m"] == pytest.approx(1.0)
    assert metrics["p95_error_m"] == pytest.approx(1.0)
    assert metrics["max_error_m"] == pytest.approx(1.0)
    assert metrics["endpoint_error_m"] == pytest.approx(1.0)


def test_path_records_and_persists_incremental_trajectory(tmp_path) -> None:
    pose_count = 0

    def pose_reader(timeout):
        nonlocal pose_count
        pose_count += 1
        return {**POSE, "x": 1.0 + min(pose_count - 1, 10) * 0.05}

    def transport(method, path, form):
        if path == "/health":
            return {"ok": True}
        if path == "/go_to_async":
            return {"success": True}
        if path == "/go_to_status":
            time.sleep(0.12)
            return {"status": "3"}
        return {"status": "0"}

    client = NavBridgeClient(
        "http://bridge",
        transport=transport,
        pose_reader=pose_reader,
        poll_interval=0.001,
        waypoint_timeout=1.0,
        trajectory_dir=tmp_path,
        trajectory_sample_interval=0.05,
    )
    queued = client.start_path(
        [
            {"x": 1.0, "y": 2.0, "ox": 0, "oy": 0, "oz": 0, "ow": 1},
            {"x": 2.0, "y": 2.0, "ox": 0, "oy": 0, "oz": 0, "ow": 1},
        ]
    )
    deadline = time.monotonic() + 2.0
    while client.trajectory_snapshot()["status"] == "recording" and time.monotonic() < deadline:
        time.sleep(0.01)

    snapshot = client.trajectory_snapshot(after_sequence=1)
    assert queued["trajectory_id"].startswith("trajectory_")
    assert snapshot["id"] == queued["trajectory_id"]
    assert snapshot["status"] == "succeeded"
    assert snapshot["sample_count"] >= 2
    assert all(sample["sequence"] > 1 for sample in snapshot["samples"])
    assert snapshot["metrics"]["max_error_m"] == pytest.approx(0.0)
    assert (tmp_path / f"{queued['trajectory_id']}.json").is_file()
