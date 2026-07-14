from __future__ import annotations

import ast
import time

import pytest

from robot_map_planner.navigation import NavBridgeClient, NavBridgeError


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
