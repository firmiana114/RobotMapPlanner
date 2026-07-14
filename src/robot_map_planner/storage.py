from __future__ import annotations

from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import shutil
import sqlite3
import struct
import tempfile
import uuid
from typing import Any, Iterator

from . import _core
from .errors import PlannerError, translate_core_error


LOGGER = logging.getLogger(__name__)
GRID_HEADER = struct.Struct("<4sIii4dQ")
GRID_MAGIC = b"RMP1"
GRID_VERSION = 1


def validate_import_config(build_config: dict[str, Any], cost_config: dict[str, Any]) -> None:
    try:
        resolution = float(build_config["resolution"])
        obstacle_min_height = float(build_config["obstacle_min_height"])
        obstacle_max_height = float(build_config["obstacle_max_height"])
        min_points_per_cell = int(build_config["min_points_per_cell"])
        hard_clearance = float(cost_config["hard_clearance"])
        inflation_radius = float(cost_config["inflation_radius"])
        cost_scaling = float(cost_config["cost_scaling"])
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise PlannerError("INVALID_CONFIG", "map import parameters must be valid numbers") from exc

    finite_values = (
        resolution,
        obstacle_min_height,
        obstacle_max_height,
        hard_clearance,
        inflation_radius,
        cost_scaling,
    )
    if not all(math.isfinite(value) for value in finite_values):
        raise PlannerError("INVALID_CONFIG", "map import parameters must be finite numbers")
    if resolution <= 0.0:
        raise PlannerError("INVALID_CONFIG", f"resolution must be greater than 0 (got {resolution})")
    if obstacle_min_height > obstacle_max_height:
        raise PlannerError(
            "INVALID_CONFIG",
            "obstacle_min_height must be less than or equal to obstacle_max_height "
            f"(got {obstacle_min_height} > {obstacle_max_height})",
        )
    if min_points_per_cell < 1:
        raise PlannerError(
            "INVALID_CONFIG", f"min_points_per_cell must be at least 1 (got {min_points_per_cell})"
        )
    if hard_clearance < 0.0:
        raise PlannerError("INVALID_CONFIG", f"hard_clearance must be at least 0 (got {hard_clearance})")
    if inflation_radius < hard_clearance:
        raise PlannerError(
            "INVALID_CONFIG",
            "inflation_radius must be greater than or equal to hard_clearance "
            f"(got {inflation_radius} < {hard_clearance})",
        )
    if cost_scaling <= 0.0:
        raise PlannerError("INVALID_CONFIG", f"cost_scaling must be greater than 0 (got {cost_scaling})")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def new_id(prefix: str) -> str:
    return f"{prefix}_{uuid.uuid4().hex}"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def write_grid(path: Path, data: bytes, meta: dict[str, Any]) -> None:
    expected = int(meta["width"]) * int(meta["height"])
    if len(data) != expected:
        raise PlannerError("INVALID_CONFIG", f"grid byte count {len(data)} does not match {expected}")
    header = GRID_HEADER.pack(
        GRID_MAGIC,
        GRID_VERSION,
        int(meta["width"]),
        int(meta["height"]),
        float(meta["origin_x"]),
        float(meta["origin_y"]),
        float(meta["resolution"]),
        float(meta["ground_z"]),
        len(data),
    )
    atomic_write(path, header + data)


def read_grid(path: Path) -> tuple[bytes, dict[str, Any]]:
    with path.open("rb") as stream:
        raw = stream.read(GRID_HEADER.size)
        if len(raw) != GRID_HEADER.size:
            raise PlannerError("MAP_NOT_READY", f"truncated grid header: {path}", status_code=500)
        magic, version, width, height, origin_x, origin_y, resolution, ground_z, size = GRID_HEADER.unpack(raw)
        if magic != GRID_MAGIC or version != GRID_VERSION or size != width * height:
            raise PlannerError("MAP_NOT_READY", f"invalid grid file: {path}", status_code=500)
        data = stream.read(size)
        if len(data) != size:
            raise PlannerError("MAP_NOT_READY", f"truncated grid body: {path}", status_code=500)
    return data, {
        "width": width,
        "height": height,
        "origin_x": origin_x,
        "origin_y": origin_y,
        "resolution": resolution,
        "ground_z": ground_z,
    }


class MapStore:
    def __init__(self, data_dir: Path) -> None:
        self.data_dir = data_dir
        self.maps_dir = data_dir / "maps"
        self.database_path = data_dir / "catalog.sqlite3"
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.maps_dir.mkdir(parents=True, exist_ok=True)
        self._initialize_database()

    @contextmanager
    def connect(self) -> Iterator[sqlite3.Connection]:
        connection = sqlite3.connect(self.database_path, timeout=30)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA foreign_keys=ON")
        connection.execute("PRAGMA journal_mode=WAL")
        try:
            yield connection
            connection.commit()
        except Exception:
            connection.rollback()
            raise
        finally:
            connection.close()

    def _initialize_database(self) -> None:
        with self.connect() as connection:
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS maps (
                  id TEXT PRIMARY KEY, name TEXT NOT NULL, source_sha256 TEXT NOT NULL,
                  source_path TEXT NOT NULL, obstacle_path TEXT NOT NULL, meta_json TEXT NOT NULL,
                  active_version_id TEXT, created_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS versions (
                  id TEXT PRIMARY KEY, map_id TEXT NOT NULL REFERENCES maps(id), number INTEGER NOT NULL,
                  parent_version_id TEXT, boundary_json TEXT NOT NULL, overlay_path TEXT NOT NULL,
                  base_path TEXT NOT NULL, final_path TEXT NOT NULL, costmap_path TEXT NOT NULL,
                  stats_json TEXT NOT NULL, created_at TEXT NOT NULL,
                  UNIQUE(map_id, number)
                );
                CREATE TABLE IF NOT EXISTS drafts (
                  id TEXT PRIMARY KEY, map_id TEXT NOT NULL REFERENCES maps(id),
                  base_version_id TEXT NOT NULL REFERENCES versions(id), revision INTEGER NOT NULL,
                  cursor INTEGER NOT NULL, overlay_path TEXT NOT NULL, boundary_json TEXT NOT NULL,
                  created_at TEXT NOT NULL, updated_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS edits (
                  draft_id TEXT NOT NULL REFERENCES drafts(id) ON DELETE CASCADE,
                  sequence INTEGER NOT NULL, patch_json TEXT NOT NULL, before_json TEXT NOT NULL,
                  PRIMARY KEY(draft_id, sequence)
                );
                """
            )

    def import_map(
        self,
        source: Path,
        *,
        name: str,
        build_config: dict[str, Any],
        cost_config: dict[str, Any],
    ) -> dict[str, Any]:
        started = datetime.now(timezone.utc)
        try:
            validate_import_config(build_config, cost_config)
        except PlannerError as exc:
            LOGGER.warning(
                "Rejected map import code=%s build_config=%s cost_config=%s message=%s",
                exc.code,
                build_config,
                cost_config,
                exc,
            )
            raise
        map_id = new_id("map")
        map_dir = self.maps_dir / map_id
        source_copy = map_dir / "source" / source.name
        source_copy.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, source_copy)
        source_sha = sha256_file(source_copy)
        try:
            built = _core.build_map(str(source_copy), build_config)
            meta = dict(built["meta"])
            obstacles = bytes(built["obstacles"])
            base = bytes(built["base_grid"])
            overlay = bytes(len(base))
            final = bytes(_core.merge_overlay(base, overlay))
            costmap = bytes(_core.build_costmap(final, meta, cost_config))
            validation = dict(_core.validate_grid(final, costmap, meta))
        except Exception as exc:
            shutil.rmtree(map_dir, ignore_errors=True)
            LOGGER.error(
                "Map import failed map_id=%s source_sha256=%s build_config=%s cost_config=%s error=%s",
                map_id,
                source_sha,
                build_config,
                cost_config,
                exc,
                exc_info=True,
            )
            raise translate_core_error(exc) from exc
        obstacle_path = map_dir / "base" / "obstacles.rmp"
        write_grid(obstacle_path, obstacles, meta)
        version_id = new_id("ver")
        version_dir = map_dir / "versions" / "1"
        overlay_path = version_dir / "overlay.rmp"
        base_path = version_dir / "base.rmp"
        final_path = version_dir / "final.rmp"
        costmap_path = version_dir / "costmap.rmp"
        for path, data in ((overlay_path, overlay), (base_path, base), (final_path, final), (costmap_path, costmap)):
            write_grid(path, data, meta)
        manifest = {
            "meta": meta,
            "build_config": build_config,
            "cost_config": cost_config,
            "boundary": [list(point) for point in built["boundary"]],
            "source": {
                "sha256": source_sha,
                "declared_points": int(built["declared_points"]),
                "finite_points": int(built["finite_points"]),
                "obstacle_points": int(built["obstacle_points"]),
                "occupied_cells": int(built["occupied_cells"]),
                "data_encoding": str(built["data_encoding"]),
                "min_bound": list(built["min_bound"]),
                "max_bound": list(built["max_bound"]),
            },
        }
        elapsed_ms = int((datetime.now(timezone.utc) - started).total_seconds() * 1000)
        validation["import_ms"] = elapsed_ms
        now = utc_now()
        with self.connect() as connection:
            connection.execute(
                "INSERT INTO maps VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                (map_id, name, source_sha, str(source_copy), str(obstacle_path), json.dumps(manifest), version_id, now),
            )
            connection.execute(
                "INSERT INTO versions VALUES (?, ?, 1, NULL, ?, ?, ?, ?, ?, ?, ?)",
                (
                    version_id,
                    map_id,
                    json.dumps(manifest["boundary"]),
                    str(overlay_path),
                    str(base_path),
                    str(final_path),
                    str(costmap_path),
                    json.dumps(validation),
                    now,
                ),
            )
        LOGGER.info(
            "Imported map map_id=%s version_id=%s source_sha256=%s points=%s finite=%s occupied_cells=%s grid=%sx%s elapsed_ms=%s",
            map_id,
            version_id,
            source_sha,
            manifest["source"]["declared_points"],
            manifest["source"]["finite_points"],
            manifest["source"]["occupied_cells"],
            meta["width"],
            meta["height"],
            elapsed_ms,
        )
        return self.get_map(map_id)

    def recompile_map(
        self,
        map_id: str,
        *,
        name: str,
        build_config: dict[str, Any],
        cost_config: dict[str, Any],
    ) -> dict[str, Any]:
        with self.connect() as connection:
            row = connection.execute("SELECT source_path FROM maps WHERE id=?", (map_id,)).fetchone()
        if row is None:
            raise PlannerError("MAP_NOT_READY", "map not found", status_code=404)
        source = Path(row["source_path"])
        if not source.is_file():
            LOGGER.error("Cannot recompile map map_id=%s because source PCD is missing", map_id)
            raise PlannerError("MAP_NOT_READY", "saved source PCD is missing", status_code=500)

        LOGGER.info(
            "Recompiling map in place map_id=%s build_config=%s cost_config=%s",
            map_id,
            build_config,
            cost_config,
        )
        staging = self.import_map(source, name=name, build_config=build_config, cost_config=cost_config)
        staging_map_id = staging["id"]
        staging_dir = self.maps_dir / staging_map_id
        map_dir = self.maps_dir / map_id
        backup_dir = self.maps_dir / f".{map_id}.recompile-{uuid.uuid4().hex}"
        replaced_versions = 0
        discarded_drafts = 0
        try:
            if not map_dir.is_dir() or not staging_dir.is_dir():
                raise PlannerError("MAP_NOT_READY", "map storage directory is missing", status_code=500)
            os.replace(map_dir, backup_dir)
            try:
                os.replace(staging_dir, map_dir)
            except Exception:
                os.replace(backup_dir, map_dir)
                raise
            try:
                with self.connect() as connection:
                    original = connection.execute("SELECT * FROM maps WHERE id=?", (map_id,)).fetchone()
                    staged = connection.execute("SELECT * FROM maps WHERE id=?", (staging_map_id,)).fetchone()
                    staged_versions = connection.execute(
                        "SELECT * FROM versions WHERE map_id=? ORDER BY number", (staging_map_id,)
                    ).fetchall()
                    if original is None or staged is None or not staged_versions:
                        raise PlannerError("MAP_NOT_READY", "staged map metadata is missing", status_code=500)
                    replaced_versions = connection.execute(
                        "SELECT COUNT(*) FROM versions WHERE map_id=?", (map_id,)
                    ).fetchone()[0]
                    discarded_drafts = connection.execute(
                        "SELECT COUNT(*) FROM drafts WHERE map_id=?", (map_id,)
                    ).fetchone()[0]
                    connection.execute("UPDATE maps SET active_version_id=NULL WHERE id=?", (map_id,))
                    connection.execute("DELETE FROM drafts WHERE map_id=?", (map_id,))
                    connection.execute("DELETE FROM versions WHERE map_id=?", (map_id,))
                    for version in staged_versions:
                        promoted_paths = [
                            str(map_dir / Path(version[column]).relative_to(staging_dir))
                            for column in ("overlay_path", "base_path", "final_path", "costmap_path")
                        ]
                        connection.execute(
                            "UPDATE versions SET map_id=?, overlay_path=?, base_path=?, final_path=?, costmap_path=? "
                            "WHERE id=?",
                            (map_id, *promoted_paths, version["id"]),
                        )
                    source_path = str(map_dir / Path(staged["source_path"]).relative_to(staging_dir))
                    obstacle_path = str(map_dir / Path(staged["obstacle_path"]).relative_to(staging_dir))
                    connection.execute(
                        "UPDATE maps SET name=?, source_sha256=?, source_path=?, obstacle_path=?, meta_json=?, "
                        "active_version_id=? WHERE id=?",
                        (
                            staged["name"],
                            staged["source_sha256"],
                            source_path,
                            obstacle_path,
                            staged["meta_json"],
                            staged["active_version_id"],
                            map_id,
                        ),
                    )
                    connection.execute("DELETE FROM maps WHERE id=?", (staging_map_id,))
            except Exception:
                os.replace(map_dir, staging_dir)
                os.replace(backup_dir, map_dir)
                raise
        except Exception as exc:
            LOGGER.exception(
                "Map recompile failed map_id=%s staging_map_id=%s error=%s",
                map_id,
                staging_map_id,
                exc,
            )
            try:
                self.delete_map(staging_map_id)
            except Exception:
                LOGGER.exception("Failed to clean staged map staging_map_id=%s", staging_map_id)
            raise

        try:
            shutil.rmtree(backup_dir)
        except Exception:
            LOGGER.exception("Failed to remove recompile backup map_id=%s backup_dir=%s", map_id, backup_dir)
        result = self.get_map(map_id)
        LOGGER.info(
            "Recompiled map in place map_id=%s version_id=%s replaced_versions=%s discarded_drafts=%s",
            map_id,
            result["active_version_id"],
            replaced_versions,
            discarded_drafts,
        )
        return result

    def delete_map(self, map_id: str) -> dict[str, Any]:
        with self.connect() as connection:
            row = connection.execute("SELECT * FROM maps WHERE id=?", (map_id,)).fetchone()
            if row is None:
                raise PlannerError("MAP_NOT_READY", "map not found", status_code=404)
            version_count = connection.execute(
                "SELECT COUNT(*) FROM versions WHERE map_id=?", (map_id,)
            ).fetchone()[0]
            draft_count = connection.execute(
                "SELECT COUNT(*) FROM drafts WHERE map_id=?", (map_id,)
            ).fetchone()[0]

        map_dir = self.maps_dir / map_id
        trash_dir = self.maps_dir / f".{map_id}.delete-{uuid.uuid4().hex}"
        moved = False
        try:
            if map_dir.exists():
                os.replace(map_dir, trash_dir)
                moved = True
            with self.connect() as connection:
                connection.execute("UPDATE maps SET active_version_id=NULL WHERE id=?", (map_id,))
                connection.execute("DELETE FROM drafts WHERE map_id=?", (map_id,))
                connection.execute("DELETE FROM versions WHERE map_id=?", (map_id,))
                connection.execute("DELETE FROM maps WHERE id=?", (map_id,))
        except Exception as exc:
            if moved and trash_dir.exists():
                os.replace(trash_dir, map_dir)
            LOGGER.exception("Map deletion failed map_id=%s error=%s", map_id, exc)
            raise

        if moved:
            try:
                shutil.rmtree(trash_dir)
            except Exception:
                LOGGER.exception("Failed to remove deleted map files map_id=%s trash_dir=%s", map_id, trash_dir)
        LOGGER.info(
            "Deleted map map_id=%s source_sha256=%s versions=%s drafts=%s",
            map_id,
            row["source_sha256"],
            version_count,
            draft_count,
        )
        return {"deleted": True, "map_id": map_id}

    def list_maps(self) -> list[dict[str, Any]]:
        with self.connect() as connection:
            rows = connection.execute("SELECT * FROM maps ORDER BY created_at DESC").fetchall()
        return [self._map_row(row) for row in rows]

    def get_map(self, map_id: str) -> dict[str, Any]:
        with self.connect() as connection:
            row = connection.execute("SELECT * FROM maps WHERE id=?", (map_id,)).fetchone()
            if row is None:
                raise PlannerError("MAP_NOT_READY", "map not found", status_code=404)
            versions = connection.execute(
                "SELECT * FROM versions WHERE map_id=? ORDER BY number DESC", (map_id,)
            ).fetchall()
        result = self._map_row(row)
        result["versions"] = [self._version_row(version) for version in versions]
        return result

    def _map_row(self, row: sqlite3.Row) -> dict[str, Any]:
        manifest = json.loads(row["meta_json"])
        return {
            "id": row["id"],
            "name": row["name"],
            "source_sha256": row["source_sha256"],
            "active_version_id": row["active_version_id"],
            "created_at": row["created_at"],
            **manifest,
        }

    def _version_row(self, row: sqlite3.Row) -> dict[str, Any]:
        return {
            "id": row["id"],
            "map_id": row["map_id"],
            "number": row["number"],
            "parent_version_id": row["parent_version_id"],
            "boundary": json.loads(row["boundary_json"]),
            "stats": json.loads(row["stats_json"]),
            "created_at": row["created_at"],
        }

    def get_version_row(self, version_id: str) -> sqlite3.Row:
        with self.connect() as connection:
            row = connection.execute("SELECT * FROM versions WHERE id=?", (version_id,)).fetchone()
        if row is None:
            raise PlannerError("MAP_NOT_READY", "version not found", status_code=404)
        return row

    def create_draft(self, map_id: str) -> dict[str, Any]:
        map_data = self.get_map(map_id)
        base_version_id = map_data["active_version_id"]
        version = self.get_version_row(base_version_id)
        draft_id = new_id("draft")
        draft_dir = self.maps_dir / map_id / "drafts" / draft_id
        overlay_path = draft_dir / "overlay.rmp"
        overlay_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(version["overlay_path"], overlay_path)
        now = utc_now()
        with self.connect() as connection:
            connection.execute(
                "INSERT INTO drafts VALUES (?, ?, ?, 0, 0, ?, ?, ?, ?)",
                (draft_id, map_id, base_version_id, str(overlay_path), version["boundary_json"], now, now),
            )
        LOGGER.info("Created map draft map_id=%s draft_id=%s base_version_id=%s", map_id, draft_id, base_version_id)
        return self.get_draft(draft_id)

    def get_draft(self, draft_id: str) -> dict[str, Any]:
        with self.connect() as connection:
            row = connection.execute("SELECT * FROM drafts WHERE id=?", (draft_id,)).fetchone()
        if row is None:
            raise PlannerError("MAP_NOT_READY", "draft not found", status_code=404)
        return dict(row) | {"boundary": json.loads(row["boundary_json"])}

    def patch_draft(self, draft_id: str, revision: int, patch: dict[str, Any]) -> dict[str, Any]:
        draft = self.get_draft(draft_id)
        if revision != int(draft["revision"]):
            raise PlannerError("VERSION_CONFLICT", "draft revision is stale", status_code=409)
        overlay, meta = read_grid(Path(draft["overlay_path"]))
        values = bytearray(overlay)
        operation = str(patch.get("operation", ""))
        if operation == "set_boundary":
            boundary = patch.get("boundary")
            if not isinstance(boundary, list) or len(boundary) < 3:
                raise PlannerError("INVALID_CONFIG", "boundary requires at least three points")
            before: dict[str, Any] = {"boundary": draft["boundary"]}
            patch_data = {"operation": operation, "boundary": boundary}
            new_boundary_json = json.dumps(boundary)
        else:
            mode_values = {"inherit": 0, "free": 1, "occupied": 2}
            mode = str(patch.get("mode", ""))
            if mode not in mode_values:
                raise PlannerError("INVALID_CONFIG", "mode must be inherit, free, or occupied")
            cells = patch.get("cells")
            if not isinstance(cells, list) or len(cells) > 500_000:
                raise PlannerError("INVALID_CONFIG", "cells must be a bounded list")
            indexes: list[int] = []
            previous: list[int] = []
            for cell in cells:
                x, y = int(cell[0]), int(cell[1])
                if 0 <= x < meta["width"] and 0 <= y < meta["height"]:
                    index = y * meta["width"] + x
                    indexes.append(index)
                    previous.append(values[index])
                    values[index] = mode_values[mode]
            write_grid(Path(draft["overlay_path"]), bytes(values), meta)
            before = {"indexes": indexes, "values": previous}
            patch_data = {"operation": "set_cells", "mode": mode, "indexes": indexes}
            new_boundary_json = json.dumps(draft["boundary"])
        with self.connect() as connection:
            connection.execute("DELETE FROM edits WHERE draft_id=? AND sequence>?", (draft_id, draft["cursor"]))
            sequence = int(draft["cursor"]) + 1
            connection.execute(
                "INSERT INTO edits VALUES (?, ?, ?, ?)",
                (draft_id, sequence, json.dumps(patch_data), json.dumps(before)),
            )
            connection.execute(
                "UPDATE drafts SET revision=revision+1, cursor=?, boundary_json=?, updated_at=? WHERE id=?",
                (sequence, new_boundary_json, utc_now(), draft_id),
            )
        LOGGER.info("Applied draft patch draft_id=%s operation=%s changed=%s revision=%s", draft_id, operation or "set_cells", len(patch_data.get("indexes", [])), revision + 1)
        return self.get_draft(draft_id)

    def _restore_edit(self, draft: dict[str, Any], edit: sqlite3.Row, undo: bool) -> None:
        patch = json.loads(edit["patch_json"])
        before = json.loads(edit["before_json"])
        if patch["operation"] == "set_boundary":
            boundary = before["boundary"] if undo else patch["boundary"]
            with self.connect() as connection:
                connection.execute("UPDATE drafts SET boundary_json=? WHERE id=?", (json.dumps(boundary), draft["id"]))
            return
        overlay, meta = read_grid(Path(draft["overlay_path"]))
        values = bytearray(overlay)
        if undo:
            for index, value in zip(before["indexes"], before["values"]):
                values[index] = value
        else:
            target = {"inherit": 0, "free": 1, "occupied": 2}[patch["mode"]]
            for index in patch["indexes"]:
                values[index] = target
        write_grid(Path(draft["overlay_path"]), bytes(values), meta)

    def move_history(self, draft_id: str, direction: str) -> dict[str, Any]:
        draft = self.get_draft(draft_id)
        cursor = int(draft["cursor"])
        sequence = cursor if direction == "undo" else cursor + 1
        with self.connect() as connection:
            edit = connection.execute("SELECT * FROM edits WHERE draft_id=? AND sequence=?", (draft_id, sequence)).fetchone()
        if edit is None:
            return draft
        self._restore_edit(draft, edit, direction == "undo")
        new_cursor = cursor - 1 if direction == "undo" else cursor + 1
        with self.connect() as connection:
            connection.execute(
                "UPDATE drafts SET revision=revision+1, cursor=?, updated_at=? WHERE id=?",
                (new_cursor, utc_now(), draft_id),
            )
        LOGGER.info("Moved draft history draft_id=%s direction=%s cursor=%s", draft_id, direction, new_cursor)
        return self.get_draft(draft_id)

    def compile_draft(self, draft_id: str) -> tuple[dict[str, Any], bytes, bytes, bytes, dict[str, Any]]:
        draft = self.get_draft(draft_id)
        map_data = self.get_map(draft["map_id"])
        obstacle_data, meta = read_grid(Path(self.maps_dir / draft["map_id"] / "base" / "obstacles.rmp"))
        overlay, overlay_meta = read_grid(Path(draft["overlay_path"]))
        if meta != overlay_meta:
            raise PlannerError("MAP_NOT_READY", "draft grid metadata mismatch", status_code=500)
        try:
            base = bytes(_core.apply_boundary(obstacle_data, meta, draft["boundary"]))
            final = bytes(_core.merge_overlay(base, overlay))
            changed_indexes, boundary_changed = self._draft_changes(draft)
            previous_costmap, previous_meta = read_grid(
                Path(self.get_version_row(draft["base_version_id"])["costmap_path"])
            )
            if previous_meta != meta:
                raise PlannerError("MAP_NOT_READY", "base version metadata mismatch", status_code=500)
            if boundary_changed:
                costmap = bytes(_core.build_costmap(final, meta, map_data["cost_config"]))
                rebuild_mode = "full"
                rebuild_region = [0, 0, meta["width"] - 1, meta["height"] - 1]
            elif changed_indexes:
                costmap, rebuild_region = self._incremental_costmap(
                    final, previous_costmap, meta, map_data["cost_config"], changed_indexes
                )
                rebuild_mode = "incremental"
            else:
                costmap = previous_costmap
                rebuild_mode = "reused"
                rebuild_region = []
            stats = dict(_core.validate_grid(final, costmap, meta))
            stats["cost_rebuild_mode"] = rebuild_mode
            stats["cost_rebuild_region"] = rebuild_region
        except Exception as exc:
            if isinstance(exc, PlannerError):
                raise
            raise translate_core_error(exc) from exc
        return draft, base, final, costmap, stats

    def _draft_changes(self, draft: dict[str, Any]) -> tuple[list[int], bool]:
        with self.connect() as connection:
            rows = connection.execute(
                "SELECT patch_json FROM edits WHERE draft_id=? AND sequence<=? ORDER BY sequence",
                (draft["id"], draft["cursor"]),
            ).fetchall()
        indexes: set[int] = set()
        boundary_changed = False
        for row in rows:
            patch = json.loads(row["patch_json"])
            if patch["operation"] == "set_boundary":
                boundary_changed = True
            else:
                indexes.update(int(index) for index in patch.get("indexes", []))
        return sorted(indexes), boundary_changed

    @staticmethod
    def _incremental_costmap(
        final: bytes,
        previous: bytes,
        meta: dict[str, Any],
        cost_config: dict[str, Any],
        changed_indexes: list[int],
    ) -> tuple[bytes, list[int]]:
        width, height = int(meta["width"]), int(meta["height"])
        resolution = float(meta["resolution"])
        radius_cells = max(1, int(float(cost_config["inflation_radius"]) / resolution + 0.999999))
        changed_x = [index % width for index in changed_indexes]
        changed_y = [index // width for index in changed_indexes]
        target_x0 = max(0, min(changed_x) - radius_cells)
        target_y0 = max(0, min(changed_y) - radius_cells)
        target_x1 = min(width - 1, max(changed_x) + radius_cells)
        target_y1 = min(height - 1, max(changed_y) + radius_cells)
        source_x0 = max(0, target_x0 - radius_cells)
        source_y0 = max(0, target_y0 - radius_cells)
        source_x1 = min(width - 1, target_x1 + radius_cells)
        source_y1 = min(height - 1, target_y1 + radius_cells)
        source_width = source_x1 - source_x0 + 1
        source_height = source_y1 - source_y0 + 1
        source = bytearray(source_width * source_height)
        for row in range(source_height):
            full_start = (source_y0 + row) * width + source_x0
            source[row * source_width:(row + 1) * source_width] = final[full_start:full_start + source_width]
        source_meta = dict(meta)
        source_meta.update(
            {
                "width": source_width,
                "height": source_height,
                "origin_x": float(meta["origin_x"]) + source_x0 * resolution,
                "origin_y": float(meta["origin_y"]) + source_y0 * resolution,
            }
        )
        source_costmap = bytes(_core.build_costmap(bytes(source), source_meta, cost_config))
        result = bytearray(previous)
        target_width = target_x1 - target_x0 + 1
        source_column = target_x0 - source_x0
        for y in range(target_y0, target_y1 + 1):
            source_row = y - source_y0
            source_start = source_row * source_width + source_column
            target_start = y * width + target_x0
            result[target_start:target_start + target_width] = source_costmap[source_start:source_start + target_width]
        return bytes(result), [target_x0, target_y0, target_x1, target_y1]

    def validate_draft(self, draft_id: str) -> dict[str, Any]:
        _, _, _, _, stats = self.compile_draft(draft_id)
        if stats["traversable_cells"] == 0:
            raise PlannerError("INVALID_CONFIG", "draft has no traversable cells")
        return stats

    def publish_draft(self, draft_id: str) -> dict[str, Any]:
        draft, base, final, costmap, stats = self.compile_draft(draft_id)
        map_data = self.get_map(draft["map_id"])
        _, meta = read_grid(Path(draft["overlay_path"]))
        with self.connect() as connection:
            number = connection.execute("SELECT COALESCE(MAX(number), 0) + 1 FROM versions WHERE map_id=?", (draft["map_id"],)).fetchone()[0]
        version_id = new_id("ver")
        version_dir = self.maps_dir / draft["map_id"] / "versions" / str(number)
        overlay_path = version_dir / "overlay.rmp"
        base_path = version_dir / "base.rmp"
        final_path = version_dir / "final.rmp"
        costmap_path = version_dir / "costmap.rmp"
        overlay, _ = read_grid(Path(draft["overlay_path"]))
        for path, data in ((overlay_path, overlay), (base_path, base), (final_path, final), (costmap_path, costmap)):
            write_grid(path, data, meta)
        now = utc_now()
        try:
            with self.connect() as connection:
                connection.execute(
                    "INSERT INTO versions VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                    (version_id, draft["map_id"], number, draft["base_version_id"], json.dumps(draft["boundary"]),
                     str(overlay_path), str(base_path), str(final_path), str(costmap_path), json.dumps(stats), now),
                )
                connection.execute("UPDATE maps SET active_version_id=? WHERE id=?", (version_id, draft["map_id"]))
                connection.execute("DELETE FROM drafts WHERE id=?", (draft_id,))
        except Exception:
            shutil.rmtree(version_dir, ignore_errors=True)
            raise
        LOGGER.info("Published map version map_id=%s version_id=%s number=%s changed_cells=%s components=%s", draft["map_id"], version_id, number, sum(value != 0 for value in overlay), stats["connected_components"])
        return self.get_map(map_data["id"])

    def activate_version(self, version_id: str) -> dict[str, Any]:
        version = self.get_version_row(version_id)
        with self.connect() as connection:
            connection.execute("UPDATE maps SET active_version_id=? WHERE id=?", (version_id, version["map_id"]))
        LOGGER.info("Activated map version map_id=%s version_id=%s", version["map_id"], version_id)
        return self.get_map(version["map_id"])

    def grid_for_layer(self, version_id: str, layer: str) -> tuple[bytes, dict[str, Any]]:
        row = self.get_version_row(version_id)
        columns = {"base": "base_path", "overlay": "overlay_path", "final": "final_path", "costmap": "costmap_path"}
        if layer not in columns:
            raise PlannerError("INVALID_CONFIG", "unsupported layer")
        return read_grid(Path(row[columns[layer]]))

    def grid_for_draft(self, draft_id: str, layer: str) -> tuple[bytes, dict[str, Any]]:
        draft = self.get_draft(draft_id)
        overlay, meta = read_grid(Path(draft["overlay_path"]))
        if layer == "overlay":
            return overlay, meta
        _, base, final, costmap, _ = self.compile_draft(draft_id)
        layers = {"base": base, "final": final, "costmap": costmap}
        if layer not in layers:
            raise PlannerError("INVALID_CONFIG", "unsupported layer")
        return layers[layer], meta

    def plan(self, version_id: str, request: dict[str, Any]) -> dict[str, Any]:
        started = datetime.now(timezone.utc)
        row = self.get_version_row(version_id)
        costmap, meta = read_grid(Path(row["costmap_path"]))
        config = {
            "snap_radius": float(request.get("snap_radius", 0.50)),
            "point_spacing": float(request.get("point_spacing", 0.50)),
            "cost_weight": float(request.get("cost_weight", 2.0)),
            "max_traversable_cost": int(request.get("max_traversable_cost", 0)),
        }
        try:
            result = dict(_core.plan(costmap, meta, tuple(request["start"]), tuple(request["goal"]), config))
        except Exception as exc:
            raise translate_core_error(exc) from exc
        elapsed_ms = int((datetime.now(timezone.utc) - started).total_seconds() * 1000)
        result["planning_ms"] = elapsed_ms
        result["map_id"] = row["map_id"]
        result["version_id"] = version_id
        for key in ("requested_start", "requested_goal", "actual_start", "actual_goal"):
            value = result[key]
            result[key] = {"x": value[0], "y": value[1]}
        result["points"] = [{"x": point[0], "y": point[1]} for point in result["points"]]
        if not result["ok"]:
            LOGGER.info("Planning failed version_id=%s code=%s start=%s goal=%s elapsed_ms=%s", version_id, result["error_code"], request.get("start"), request.get("goal"), elapsed_ms)
            raise PlannerError(result["error_code"], result["message"], status_code=422)
        LOGGER.info("Planned path version_id=%s start=%s goal=%s max_traversable_cost=%s points=%s length_m=%.3f expanded=%s elapsed_ms=%s", version_id, request["start"], request["goal"], config["max_traversable_cost"], len(result["points"]), result["length_m"], result["expanded_nodes"], elapsed_ms)
        return result
