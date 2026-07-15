from __future__ import annotations

from datetime import datetime, timezone
import json
import logging
import math
import os
from pathlib import Path
import tempfile
import threading
import time
from typing import Any, Callable
from urllib import error, parse, request
import uuid


LOGGER = logging.getLogger(__name__)


class NavBridgeError(RuntimeError):
    def __init__(self, code: str, message: str, *, status_code: int = 503) -> None:
        super().__init__(message)
        self.code = code
        self.status_code = status_code


def _polyline_length(points: list[dict[str, Any]]) -> float:
    return sum(
        math.hypot(float(current["x"]) - float(previous["x"]), float(current["y"]) - float(previous["y"]))
        for previous, current in zip(points, points[1:])
    )


def _point_to_segment_distance(
    x: float, y: float, start: dict[str, Any], end: dict[str, Any]
) -> float:
    start_x, start_y = float(start["x"]), float(start["y"])
    delta_x, delta_y = float(end["x"]) - start_x, float(end["y"]) - start_y
    length_squared = delta_x * delta_x + delta_y * delta_y
    if length_squared <= 1e-18:
        return math.hypot(x - start_x, y - start_y)
    ratio = max(0.0, min(1.0, ((x - start_x) * delta_x + (y - start_y) * delta_y) / length_squared))
    return math.hypot(x - (start_x + ratio * delta_x), y - (start_y + ratio * delta_y))


def trajectory_metrics(
    planned_points: list[dict[str, Any]], samples: list[dict[str, Any]]
) -> dict[str, Any]:
    """Measure actual samples against the planned polyline in their shared map frame."""
    if len(planned_points) < 2 or not samples:
        return {
            "sample_count": len(samples),
            "planned_length_m": _polyline_length(planned_points),
            "actual_length_m": _polyline_length(samples),
            "mean_error_m": None,
            "rms_error_m": None,
            "p95_error_m": None,
            "max_error_m": None,
            "endpoint_error_m": None,
        }
    errors = [
        min(
            _point_to_segment_distance(float(sample["x"]), float(sample["y"]), start, end)
            for start, end in zip(planned_points, planned_points[1:])
        )
        for sample in samples
    ]
    ordered = sorted(errors)
    p95_index = max(0, math.ceil(len(ordered) * 0.95) - 1)
    endpoint = planned_points[-1]
    last = samples[-1]
    return {
        "sample_count": len(samples),
        "planned_length_m": _polyline_length(planned_points),
        "actual_length_m": _polyline_length(samples),
        "mean_error_m": sum(errors) / len(errors),
        "rms_error_m": math.sqrt(sum(value * value for value in errors) / len(errors)),
        "p95_error_m": ordered[p95_index],
        "max_error_m": max(errors),
        "endpoint_error_m": math.hypot(float(last["x"]) - float(endpoint["x"]), float(last["y"]) - float(endpoint["y"])),
    }


class NavBridgeClient:
    def __init__(
        self,
        base_url: str,
        *,
        request_timeout: float = 5.0,
        pose_timeout: float = 5.0,
        waypoint_timeout: float = 300.0,
        poll_interval: float = 0.5,
        start_tolerance: float = 0.75,
        trajectory_dir: Path | None = None,
        trajectory_sample_interval: float = 0.2,
        trajectory_max_samples: int = 100000,
        transport: Callable[[str, str, dict[str, str] | None], dict[str, Any]] | None = None,
        pose_reader: Callable[[float], dict[str, Any]] | None = None,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.request_timeout = request_timeout
        self.pose_timeout = pose_timeout
        self.waypoint_timeout = waypoint_timeout
        self.poll_interval = poll_interval
        self.start_tolerance = start_tolerance
        self.trajectory_dir = trajectory_dir
        self.trajectory_sample_interval = max(0.05, trajectory_sample_interval)
        self.trajectory_max_samples = max(1, trajectory_max_samples)
        self._transport = transport or self._http_json
        self._pose_reader = pose_reader
        self._lock = threading.Lock()
        self._trajectory_started_monotonic = 0.0
        self._trajectory: dict[str, Any] = {
            "id": None,
            "status": "idle",
            "source": "nav_bridge_current_pose",
            "frame": "map",
            "planned_points": [],
            "samples": [],
            "started_at": None,
            "ended_at": None,
        }
        self._execution: dict[str, Any] = {
            "status": "idle",
            "message": "尚未下发路径",
            "current_waypoint": 0,
            "total_waypoints": 0,
            "started_at": None,
            "updated_at": self._now(),
        }

    @staticmethod
    def _now() -> str:
        return datetime.now(timezone.utc).isoformat()

    def _http_json(self, method: str, path: str, form: dict[str, str] | None = None) -> dict[str, Any]:
        data = parse.urlencode(form).encode("utf-8") if form is not None else None
        target = f"{self.base_url}{path}"
        http_request = request.Request(target, data=data, method=method)
        if data is not None:
            http_request.add_header("Content-Type", "application/x-www-form-urlencoded")
        try:
            with request.urlopen(http_request, timeout=self.request_timeout) as response:
                body = response.read(64 * 1024)
        except error.HTTPError as exc:
            body = exc.read(1024).decode("utf-8", errors="replace")
            raise NavBridgeError(
                "NAV_BRIDGE_REJECTED",
                f"NavBridge request failed with HTTP {exc.code}: {body[:300]}",
            ) from exc
        except (error.URLError, TimeoutError, OSError) as exc:
            raise NavBridgeError("NAV_BRIDGE_OFFLINE", "NavBridge is offline or unreachable") from exc
        try:
            payload = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise NavBridgeError("NAV_BRIDGE_INVALID_RESPONSE", "NavBridge returned invalid JSON") from exc
        if not isinstance(payload, dict):
            raise NavBridgeError("NAV_BRIDGE_INVALID_RESPONSE", "NavBridge returned a non-object response")
        return payload

    def health(self) -> dict[str, Any]:
        payload = self._transport("GET", "/health", None)
        if payload.get("ok") is not True:
            raise NavBridgeError("NAV_BRIDGE_OFFLINE", "NavBridge health check did not report ready")
        return payload

    def _read_pose(self, *, check_health: bool, log_success: bool) -> dict[str, Any]:
        if check_health:
            self.health()
        try:
            if self._pose_reader is not None:
                pose = dict(self._pose_reader(self.pose_timeout))
                pose.setdefault("localized", True)
            else:
                pose = dict(self._transport("GET", "/current_pose", None))
        except NavBridgeError:
            raise
        except Exception as exc:
            raise NavBridgeError("ROBOT_NOT_LOCALIZED", "robot localization pose is unavailable", status_code=409) from exc
        if pose.get("localized") is not True:
            raise NavBridgeError(
                "ROBOT_NOT_LOCALIZED",
                str(pose.get("message") or "robot localization has not produced a current pose"),
                status_code=409,
            )
        required = ("x", "y", "z", "ox", "oy", "oz", "ow")
        try:
            normalized = {key: float(pose[key]) for key in required}
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise NavBridgeError("ROBOT_NOT_LOCALIZED", "robot localization pose is invalid", status_code=409) from exc
        if not all(math.isfinite(value) for value in normalized.values()):
            raise NavBridgeError("ROBOT_NOT_LOCALIZED", "robot localization pose is not finite", status_code=409)
        quaternion_norm = math.sqrt(sum(normalized[key] ** 2 for key in ("ox", "oy", "oz", "ow")))
        if quaternion_norm < 1e-6:
            raise NavBridgeError("ROBOT_NOT_LOCALIZED", "robot orientation quaternion is invalid", status_code=409)
        normalized["localized"] = True
        if log_success:
            LOGGER.info("Robot localization pose acquired x=%.3f y=%.3f", normalized["x"], normalized["y"])
        return normalized

    def current_pose(self) -> dict[str, Any]:
        return self._read_pose(check_health=True, log_success=True)

    def trajectory_snapshot(self, after_sequence: int = 0) -> dict[str, Any]:
        with self._lock:
            trajectory = self._trajectory
            planned_points = [dict(point) for point in trajectory["planned_points"]]
            all_samples = [dict(sample) for sample in trajectory["samples"]]
            payload = {
                key: trajectory[key]
                for key in ("id", "status", "source", "frame", "started_at", "ended_at")
            }
        samples = [sample for sample in all_samples if int(sample["sequence"]) > after_sequence]
        payload.update(
            {
                "latest_sequence": int(all_samples[-1]["sequence"]) if all_samples else 0,
                "sample_count": len(all_samples),
                "samples": samples,
                "metrics": trajectory_metrics(planned_points, all_samples),
            }
        )
        return payload

    def _sample_from_pose(self, pose: dict[str, Any], sequence: int) -> dict[str, Any]:
        yaw = math.atan2(
            2 * (pose["ow"] * pose["oz"] + pose["ox"] * pose["oy"]),
            1 - 2 * (pose["oy"] * pose["oy"] + pose["oz"] * pose["oz"]),
        )
        return {
            "sequence": sequence,
            "received_at": self._now(),
            "elapsed_s": max(0.0, time.monotonic() - self._trajectory_started_monotonic),
            "x": pose["x"],
            "y": pose["y"],
            "z": pose["z"],
            "ox": pose["ox"],
            "oy": pose["oy"],
            "oz": pose["oz"],
            "ow": pose["ow"],
            "yaw": yaw,
        }

    def _begin_trajectory(self, points: list[dict[str, float]], initial_pose: dict[str, Any]) -> str:
        trajectory_id = f"trajectory_{uuid.uuid4().hex}"
        self._trajectory_started_monotonic = time.monotonic()
        initial_sample = self._sample_from_pose(initial_pose, 1)
        with self._lock:
            self._trajectory = {
                "id": trajectory_id,
                "status": "recording",
                "source": "nav_bridge_current_pose",
                "frame": "map",
                "planned_points": [dict(point) for point in points],
                "samples": [initial_sample],
                "started_at": self._now(),
                "ended_at": None,
            }
        LOGGER.info(
            "Trajectory recording started trajectory_id=%s planned_points=%s sample_interval_s=%.3f frame=map",
            trajectory_id,
            len(points),
            self.trajectory_sample_interval,
        )
        return trajectory_id

    def _sample_trajectory(self, trajectory_id: str, stop_event: threading.Event) -> None:
        consecutive_failures = 0
        while not stop_event.wait(self.trajectory_sample_interval):
            try:
                pose = self._read_pose(check_health=False, log_success=False)
                with self._lock:
                    if self._trajectory["id"] != trajectory_id or self._trajectory["status"] != "recording":
                        return
                    samples = self._trajectory["samples"]
                    if len(samples) >= self.trajectory_max_samples:
                        LOGGER.warning(
                            "Trajectory sample limit reached trajectory_id=%s max_samples=%s",
                            trajectory_id,
                            self.trajectory_max_samples,
                        )
                        return
                    samples.append(self._sample_from_pose(pose, len(samples) + 1))
                if consecutive_failures:
                    LOGGER.info(
                        "Trajectory pose sampling recovered trajectory_id=%s missed_samples=%s",
                        trajectory_id,
                        consecutive_failures,
                    )
                    consecutive_failures = 0
            except Exception:
                consecutive_failures += 1
                if consecutive_failures == 1:
                    LOGGER.warning(
                        "Trajectory pose sampling interrupted trajectory_id=%s",
                        trajectory_id,
                        exc_info=True,
                    )

    def _persist_trajectory(self, payload: dict[str, Any]) -> None:
        if self.trajectory_dir is None:
            return
        self.trajectory_dir.mkdir(parents=True, exist_ok=True)
        destination = self.trajectory_dir / f"{payload['id']}.json"
        fd, temporary_name = tempfile.mkstemp(prefix=f".{payload['id']}.", suffix=".tmp", dir=self.trajectory_dir)
        temporary = Path(temporary_name)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(payload, stream, ensure_ascii=False, separators=(",", ":"))
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, destination)
        except Exception:
            temporary.unlink(missing_ok=True)
            raise

    def _finalize_trajectory(self, trajectory_id: str) -> None:
        execution = self.execution_status()
        ended_at = self._now()
        with self._lock:
            if self._trajectory["id"] != trajectory_id:
                return
            planned_points = [dict(point) for point in self._trajectory["planned_points"]]
            samples = [dict(sample) for sample in self._trajectory["samples"]]
            payload = {
                key: self._trajectory[key]
                for key in ("id", "source", "frame", "started_at")
            }
            payload.update({"status": execution["status"], "ended_at": ended_at})
        metrics = trajectory_metrics(planned_points, samples)
        payload.update({"planned_points": planned_points, "samples": samples, "metrics": metrics})
        try:
            self._persist_trajectory(payload)
        except Exception:
            LOGGER.exception("Failed to persist trajectory trajectory_id=%s", trajectory_id)
        with self._lock:
            if self._trajectory["id"] == trajectory_id:
                self._trajectory["status"] = execution["status"]
                self._trajectory["ended_at"] = ended_at
        LOGGER.info(
            "Trajectory recording completed trajectory_id=%s status=%s samples=%s mean_error_m=%s max_error_m=%s",
            trajectory_id,
            execution["status"],
            len(samples),
            metrics["mean_error_m"],
            metrics["max_error_m"],
        )

    def execution_status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._execution)

    def start_path(self, points: list[dict[str, Any]]) -> dict[str, Any]:
        if len(points) < 2:
            raise NavBridgeError("INVALID_PATH", "planned path requires at least two points", status_code=400)
        normalized: list[dict[str, float]] = []
        required = ("x", "y", "ox", "oy", "oz", "ow")
        try:
            for point in points:
                values = {key: float(point[key]) for key in required}
                if not all(math.isfinite(value) for value in values.values()):
                    raise ValueError("non-finite waypoint")
                normalized.append(values)
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            raise NavBridgeError("INVALID_PATH", "planned path contains an invalid waypoint", status_code=400) from exc
        with self._lock:
            if self._execution["status"] in {"queued", "active"}:
                raise NavBridgeError("NAVIGATION_BUSY", "another path is already running", status_code=409)
        pose = self.current_pose()
        start_distance = math.hypot(normalized[0]["x"] - pose["x"], normalized[0]["y"] - pose["y"])
        if start_distance > self.start_tolerance:
            raise NavBridgeError(
                "ROBOT_START_MISMATCH",
                f"robot is {start_distance:.2f} m from the planned start; reset the start from current pose",
                status_code=409,
            )
        waypoints = normalized[1:]
        trajectory_id = self._begin_trajectory(normalized, pose)
        now = self._now()
        with self._lock:
            self._execution = {
                "status": "queued",
                "message": "路径已排队",
                "current_waypoint": 0,
                "total_waypoints": len(waypoints),
                "started_at": now,
                "updated_at": now,
                "trajectory_id": trajectory_id,
            }
        LOGGER.info(
            "Queued robot path waypoints=%s start_distance_m=%.3f bridge_url=%s",
            len(waypoints),
            start_distance,
            self.base_url,
        )
        threading.Thread(
            target=self._execute_path_with_trajectory,
            args=(waypoints, trajectory_id),
            daemon=True,
        ).start()
        return self.execution_status()

    def _execute_path_with_trajectory(self, waypoints: list[dict[str, float]], trajectory_id: str) -> None:
        stop_event = threading.Event()
        sampler = threading.Thread(
            target=self._sample_trajectory,
            args=(trajectory_id, stop_event),
            daemon=True,
        )
        sampler.start()
        try:
            self._execute_path(waypoints)
        finally:
            stop_event.set()
            sampler.join(timeout=max(1.0, self.trajectory_sample_interval * 2))
            self._finalize_trajectory(trajectory_id)

    def _update_execution(self, **values: Any) -> None:
        with self._lock:
            self._execution.update(values)
            self._execution["updated_at"] = self._now()

    def _execute_path(self, waypoints: list[dict[str, float]]) -> None:
        try:
            for index, point in enumerate(waypoints, start=1):
                self._update_execution(
                    status="active",
                    message=f"正在前往路径点 {index}/{len(waypoints)}",
                    current_waypoint=index,
                )
                self._transport("POST", "/reset_go_to_status", {"task": ""})
                task = repr((point["x"], point["y"], point["ox"], point["oy"], point["oz"], point["ow"]))
                response = self._transport("POST", "/go_to_async", {"task": task})
                if response.get("success") is not True:
                    raise NavBridgeError(
                        "NAVIGATION_REJECTED", str(response.get("message") or "NavBridge rejected waypoint")
                    )
                deadline = time.monotonic() + self.waypoint_timeout
                while time.monotonic() < deadline:
                    status = self._transport("POST", "/go_to_status", {"task": ""})
                    code = str(status.get("status", ""))
                    if code == "3":
                        break
                    if code in {"2", "4"}:
                        raise NavBridgeError(
                            "NAVIGATION_FAILED",
                            f"waypoint {index} failed with status {code}: {status.get('sub', '')}",
                        )
                    time.sleep(self.poll_interval)
                else:
                    raise NavBridgeError(
                        "NAVIGATION_TIMEOUT", f"waypoint {index} did not finish within {self.waypoint_timeout:.0f}s"
                    )
                LOGGER.info("Robot reached path waypoint index=%s total=%s", index, len(waypoints))
            self._update_execution(
                status="succeeded",
                message="机器人已完成规划路径",
                current_waypoint=len(waypoints),
            )
            LOGGER.info("Robot path completed waypoints=%s", len(waypoints))
        except Exception as exc:
            code = exc.code if isinstance(exc, NavBridgeError) else "NAVIGATION_FAILED"
            self._update_execution(status="failed", message=str(exc), error_code=code)
            LOGGER.exception(
                "Robot path execution failed current_waypoint=%s total_waypoints=%s code=%s",
                self.execution_status()["current_waypoint"],
                len(waypoints),
                code,
            )
