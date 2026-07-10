from __future__ import annotations

import argparse
import json
import logging
from pathlib import Path
import sys

import uvicorn
from .config import Settings
from .errors import PlannerError
from .storage import MapStore


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robot-map-planner")
    parser.add_argument("--data-dir", type=Path, default=None)
    subparsers = parser.add_subparsers(dest="command", required=True)

    import_parser = subparsers.add_parser("import", help="import a PCD map")
    import_parser.add_argument("pcd", type=Path)
    import_parser.add_argument("--name", default=None)
    import_parser.add_argument("--resolution", type=float, default=0.10)
    import_parser.add_argument("--obstacle-min-height", type=float, default=0.15)
    import_parser.add_argument("--obstacle-max-height", type=float, default=2.00)
    import_parser.add_argument("--min-points-per-cell", type=int, default=1)
    import_parser.add_argument("--hard-clearance", type=float, default=0.25)
    import_parser.add_argument("--inflation-radius", type=float, default=0.50)
    import_parser.add_argument("--cost-scaling", type=float, default=5.0)

    validate_parser = subparsers.add_parser("validate", help="validate a draft")
    validate_parser.add_argument("draft_id")

    plan_parser = subparsers.add_parser("plan", help="plan on a published map version")
    plan_parser.add_argument("version_id")
    plan_parser.add_argument("--start", type=float, nargs=2, required=True)
    plan_parser.add_argument("--goal", type=float, nargs=2, required=True)
    plan_parser.add_argument("--point-spacing", type=float, default=0.50)
    plan_parser.add_argument("--snap-radius", type=float, default=0.50)
    plan_parser.add_argument("--output", type=Path)

    serve_parser = subparsers.add_parser("serve", help="serve the HTTP API and editor")
    serve_parser.add_argument("--host", default=None)
    serve_parser.add_argument("--port", type=int, default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s")
    args = build_parser().parse_args(argv)
    settings = Settings.from_env()
    if args.data_dir is not None:
        settings = Settings(args.data_dir.resolve(), settings.import_roots, settings.host, settings.port)
    store = MapStore(settings.data_dir)
    try:
        if args.command == "import":
            result = store.import_map(
                args.pcd.resolve(),
                name=args.name or args.pcd.stem,
                build_config={
                    "resolution": args.resolution,
                    "obstacle_min_height": args.obstacle_min_height,
                    "obstacle_max_height": args.obstacle_max_height,
                    "min_points_per_cell": args.min_points_per_cell,
                },
                cost_config={
                    "hard_clearance": args.hard_clearance,
                    "inflation_radius": args.inflation_radius,
                    "cost_scaling": args.cost_scaling,
                },
            )
        elif args.command == "validate":
            result = store.validate_draft(args.draft_id)
        elif args.command == "plan":
            result = store.plan(
                args.version_id,
                {
                    "start": args.start,
                    "goal": args.goal,
                    "point_spacing": args.point_spacing,
                    "snap_radius": args.snap_radius,
                },
            )
            if args.output:
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        else:
            from .api import create_app

            host, port = args.host or settings.host, args.port or settings.port
            uvicorn.run(create_app(settings), host=host, port=port, workers=1)
            return 0
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except PlannerError as exc:
        logging.getLogger(__name__).error("Command failed code=%s message=%s", exc.code, exc)
        print(json.dumps({"error": {"code": exc.code, "message": str(exc)}}, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
