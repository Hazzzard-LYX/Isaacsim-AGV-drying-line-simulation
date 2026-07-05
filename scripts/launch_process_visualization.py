#!/usr/bin/env python3
"""Launch Isaac Sim GUI with the oven room process-flow scene."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = ROOT_DIR / "scenes" / "oven_room.usd"


def main() -> None:
    scene_path = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else DEFAULT_SCENE
    if not scene_path.is_file():
        raise FileNotFoundError(f"Scene not found: {scene_path}")

    try:
        from isaacsim import SimulationApp
    except ImportError:
        from omni.isaac.kit import SimulationApp

    app = SimulationApp({"headless": False})

    try:
        import omni.timeline
        import omni.usd
        from isaacsim.core.utils.stage import is_stage_loading, open_stage

        for _ in range(5):
            app.update()

        if not open_stage(str(scene_path)):
            raise RuntimeError(f"Failed to open stage: {scene_path}")

        while is_stage_loading():
            app.update()

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Stage handle is unavailable after loading.")

        default_prim = stage.GetDefaultPrim()
        if not default_prim or not default_prim.GetChildren():
            raise RuntimeError("Loaded stage appears empty. Regenerate scenes/oven_room.usd first.")

        time_codes_per_second = stage.GetTimeCodesPerSecond() or 24.0
        end_time = stage.GetEndTimeCode() / time_codes_per_second
        timeline = omni.timeline.get_timeline_interface()
        timeline.set_start_time(0.0)
        timeline.set_end_time(end_time)
        timeline.set_looping(True)
        timeline.play()

        print(f"Opened scene: {scene_path}", flush=True)
        print(f"Default prim: {default_prim.GetPath()}", flush=True)
        print(f"Animation: 0 - {end_time:.1f}s (looping)", flush=True)

        while app.is_running():
            app.update()
    finally:
        app.close()


if __name__ == "__main__":
    main()
