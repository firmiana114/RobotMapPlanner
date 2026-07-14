from __future__ import annotations

from datetime import datetime, timezone
import json
import logging
import math
import threading
import time
from typing import Any, Callable
from urllib import error, parse, request


LOGGER = logging.getLogger(__name__)


class NavBridgeError(RuntimeError):
    def __init__(self, code: str, message: str, *, status_code: int = 503) -> None:
        super().__init__(message)
        self.code = code
        self.status_code = status_code


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
        transport: Callable[[str, str, dict[str, str] | None], dict[str, Any]] | None = None,
        pose_reader: Callable[[float], dict[str, Any]] | None = None,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.request_timeout = request_timeout
        self.pose_timeout = pose_timeout
        self.waypoint_timeout = waypoint_timeout
        self.poll_interval = poll_interval
        self.start_tolerance = start_tolerance
        self._transport = transport or self._http_json
        self._pose_reader = pose_reader
        self._lock = threading.Lock()
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

    def current_pose(self) -> dict[str, Any]:
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
            LOGGER.exception("Failed to read robot pose from NavBridge current_pose endpoint")
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
        LOGGER.info("Robot localization pose acquired x=%.3f y=%.3f", normalized["x"], normalized["y"])
        return normalized

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
        now = self._now()
        with self._lock:
            self._execution = {
                "status": "queued",
                "message": "路径已排队",
                "current_waypoint": 0,
                "total_waypoints": len(waypoints),
                "started_at": now,
                "updated_at": now,
            }
        LOGGER.info(
            "Queued robot path waypoints=%s start_distance_m=%.3f bridge_url=%s",
            len(waypoints),
            start_distance,
            self.base_url,
        )
        threading.Thread(target=self._execute_path, args=(waypoints,), daemon=True).start()
        return self.execution_status()

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
