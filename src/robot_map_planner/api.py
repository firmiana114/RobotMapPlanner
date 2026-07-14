from __future__ import annotations

import json
import logging
import os
from contextlib import asynccontextmanager
from pathlib import Path
import platform
import shutil
import tempfile
import time
from typing import Any

from fastapi import FastAPI, File, Form, Request, UploadFile
from fastapi.responses import HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from . import __version__
from .config import Settings
from .errors import PlannerError
from .storage import MapStore


LOGGER = logging.getLogger(__name__)


class DraftPatch(BaseModel):
    revision: int = Field(ge=0)
    operation: str
    mode: str | None = None
    cells: list[tuple[int, int]] | None = None
    boundary: list[tuple[float, float]] | None = None


class PlanRequest(BaseModel):
    start: tuple[float, float]
    goal: tuple[float, float]
    snap_radius: float = Field(default=0.50, ge=0.0, le=10.0)
    point_spacing: float = Field(default=0.50, gt=0.0, le=20.0)
    cost_weight: float = Field(default=2.0, ge=0.0, le=100.0)
    max_traversable_cost: int = Field(default=0, ge=0, le=252)


class RecompileMapRequest(BaseModel):
    name: str = Field(min_length=1, max_length=200)
    resolution: float = 0.10
    obstacle_min_height: float = 0.15
    obstacle_max_height: float = 2.00
    min_points_per_cell: int = 1
    hard_clearance: float = 0.25
    inflation_radius: float = 0.50
    cost_scaling: float = 5.0


def _is_allowed_source(path: Path, roots: tuple[Path, ...]) -> bool:
    resolved = path.expanduser().resolve()
    return any(resolved == root or root in resolved.parents for root in roots)


def create_app(settings: Settings | None = None) -> FastAPI:
    settings = settings or Settings.from_env()
    store = MapStore(settings.data_dir)

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        LOGGER.info(
            "RobotMapPlanner started version=%s architecture=%s data_dir=%s import_roots=%s",
            __version__, platform.machine(), settings.data_dir, [str(root) for root in settings.import_roots],
        )
        yield

    app = FastAPI(title="RobotMapPlanner", version=__version__, lifespan=lifespan)
    app.state.settings = settings
    app.state.store = store
    static_dir = Path(__file__).parent / "static"
    app.mount("/static", StaticFiles(directory=static_dir), name="static")

    @app.exception_handler(PlannerError)
    async def planner_error_handler(request: Request, exc: PlannerError) -> JSONResponse:
        LOGGER.warning(
            "API request rejected method=%s path=%s code=%s message=%s",
            request.method,
            request.url.path,
            exc.code,
            exc,
        )
        return JSONResponse(status_code=exc.status_code, content={"error": {"code": exc.code, "message": str(exc)}})

    @app.exception_handler(Exception)
    async def unexpected_error_handler(request: Request, exc: Exception) -> JSONResponse:
        LOGGER.exception("Unhandled API error method=%s path=%s", request.method, request.url.path)
        return JSONResponse(status_code=500, content={"error": {"code": "INTERNAL_ERROR", "message": "internal server error"}})

    @app.get("/", response_class=HTMLResponse)
    async def index() -> HTMLResponse:
        return HTMLResponse((static_dir / "index.html").read_text(encoding="utf-8"))

    @app.get("/healthz")
    async def healthz() -> dict[str, Any]:
        return {"status": "ok", "version": __version__, "architecture": platform.machine()}

    @app.post("/api/v1/maps/import", status_code=201)
    async def import_map(
        name: str = Form("未命名地图"),
        source_path: str | None = Form(default=None),
        config_json: str = Form("{}"),
        file: UploadFile | None = File(default=None),
    ) -> dict[str, Any]:
        try:
            config = json.loads(config_json)
        except json.JSONDecodeError as exc:
            raise PlannerError("INVALID_CONFIG", "config_json is not valid JSON") from exc
        try:
            build_config = {
                "resolution": float(config.get("resolution", 0.10)),
                "obstacle_min_height": float(config.get("obstacle_min_height", 0.15)),
                "obstacle_max_height": float(config.get("obstacle_max_height", 2.00)),
                "min_points_per_cell": int(config.get("min_points_per_cell", 1)),
            }
            cost_config = {
                "hard_clearance": float(config.get("hard_clearance", 0.25)),
                "inflation_radius": float(config.get("inflation_radius", 0.50)),
                "cost_scaling": float(config.get("cost_scaling", 5.0)),
            }
        except (TypeError, ValueError, OverflowError) as exc:
            raise PlannerError("INVALID_CONFIG", "map import parameters must be valid numbers") from exc
        temporary: Path | None = None
        if file is not None:
            suffix = Path(file.filename or "upload.pcd").suffix.lower()
            if suffix != ".pcd":
                raise PlannerError("INVALID_PCD", "uploaded file must use .pcd extension")
            fd, path = tempfile.mkstemp(prefix="rmp-upload-", suffix=".pcd")
            os.close(fd)
            temporary = Path(path)
            with temporary.open("wb") as target:
                shutil.copyfileobj(file.file, target)
            source = temporary
        elif source_path:
            source = Path(source_path)
            if not _is_allowed_source(source, settings.import_roots):
                raise PlannerError("INVALID_CONFIG", "source_path is outside configured import roots", status_code=403)
        else:
            raise PlannerError("INVALID_CONFIG", "file or source_path is required")
        if not source.is_file():
            raise PlannerError("INVALID_PCD", "source file does not exist")
        try:
            return store.import_map(source, name=name.strip() or source.stem, build_config=build_config, cost_config=cost_config)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    @app.get("/api/v1/maps")
    async def list_maps() -> list[dict[str, Any]]:
        return store.list_maps()

    @app.get("/api/v1/maps/{map_id}")
    async def get_map(map_id: str) -> dict[str, Any]:
        return store.get_map(map_id)

    @app.post("/api/v1/maps/{map_id}/recompile", status_code=201)
    async def recompile_map(map_id: str, payload: RecompileMapRequest) -> dict[str, Any]:
        values = payload.model_dump()
        name = values.pop("name").strip()
        if not name:
            raise PlannerError("INVALID_CONFIG", "map name must not be blank")
        build_config = {
            key: values[key]
            for key in ("resolution", "obstacle_min_height", "obstacle_max_height", "min_points_per_cell")
        }
        cost_config = {
            key: values[key]
            for key in ("hard_clearance", "inflation_radius", "cost_scaling")
        }
        return store.recompile_map(
            map_id,
            name=name,
            build_config=build_config,
            cost_config=cost_config,
        )

    @app.post("/api/v1/maps/{map_id}/drafts", status_code=201)
    async def create_draft(map_id: str) -> dict[str, Any]:
        return store.create_draft(map_id)

    @app.get("/api/v1/drafts/{draft_id}")
    async def get_draft(draft_id: str) -> dict[str, Any]:
        return store.get_draft(draft_id)

    @app.patch("/api/v1/drafts/{draft_id}")
    async def patch_draft(draft_id: str, patch: DraftPatch) -> dict[str, Any]:
        return store.patch_draft(draft_id, patch.revision, patch.model_dump(exclude_none=True))

    @app.post("/api/v1/drafts/{draft_id}/undo")
    async def undo(draft_id: str) -> dict[str, Any]:
        return store.move_history(draft_id, "undo")

    @app.post("/api/v1/drafts/{draft_id}/redo")
    async def redo(draft_id: str) -> dict[str, Any]:
        return store.move_history(draft_id, "redo")

    @app.post("/api/v1/drafts/{draft_id}/validate")
    async def validate_draft(draft_id: str) -> dict[str, Any]:
        return {"valid": True, "stats": store.validate_draft(draft_id)}

    @app.post("/api/v1/drafts/{draft_id}/publish")
    async def publish_draft(draft_id: str) -> dict[str, Any]:
        return store.publish_draft(draft_id)

    @app.get("/api/v1/drafts/{draft_id}/grid/{layer}")
    async def draft_grid(draft_id: str, layer: str) -> Response:
        data, meta = store.grid_for_draft(draft_id, layer)
        headers = {"X-RMP-Meta": json.dumps(meta, separators=(",", ":"))}
        return Response(data, media_type="application/octet-stream", headers=headers)

    @app.post("/api/v1/versions/{version_id}/activate")
    async def activate_version(version_id: str) -> dict[str, Any]:
        return store.activate_version(version_id)

    @app.get("/api/v1/versions/{version_id}/grid/{layer}")
    async def full_grid(version_id: str, layer: str) -> Response:
        data, meta = store.grid_for_layer(version_id, layer)
        headers = {"X-RMP-Meta": json.dumps(meta, separators=(",", ":"))}
        return Response(data, media_type="application/octet-stream", headers=headers)

    @app.get("/api/v1/versions/{version_id}/tiles/{layer}/{tile_x}/{tile_y}")
    async def tile(version_id: str, layer: str, tile_x: int, tile_y: int) -> Response:
        data, meta = store.grid_for_layer(version_id, layer)
        width, height = int(meta["width"]), int(meta["height"])
        x0, y0 = tile_x * 256, tile_y * 256
        if x0 >= width or y0 >= height or tile_x < 0 or tile_y < 0:
            raise PlannerError("INVALID_CONFIG", "tile is outside grid", status_code=404)
        tile_width, tile_height = min(256, width - x0), min(256, height - y0)
        output = bytearray(tile_width * tile_height)
        for row in range(tile_height):
            start = (y0 + row) * width + x0
            output[row * tile_width:(row + 1) * tile_width] = data[start:start + tile_width]
        headers = {"X-RMP-Tile-Width": str(tile_width), "X-RMP-Tile-Height": str(tile_height)}
        return Response(bytes(output), media_type="application/octet-stream", headers=headers)

    @app.post("/api/v1/versions/{version_id}/plan")
    async def plan(version_id: str, payload: PlanRequest) -> dict[str, Any]:
        return store.plan(version_id, payload.model_dump())

    return app
