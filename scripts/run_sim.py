#!/usr/bin/env python3
"""
Open any run folder in the simulator: one command, any OS.
File: scripts/run_sim.py

    python scripts/run_sim.py ~/.lang2sim/projects/demo/runs/20261026_150000
    python scripts/run_sim.py --fixture small_warehouse
    python scripts/run_sim.py --fixture tb3_world --display vnc     # macOS: real Gazebo GUI in a browser
    python scripts/run_sim.py --fixture small_warehouse --mission  # also drive spec.json's mission

Steps: doctor checks -> display plan -> sim container -> world pre-flight ->
sim_launch.py -> Gazebo + TurtleBot3 + Nav2 (+ GUI/RViz) -> wait until every
readiness check passes -> keep running until Ctrl-C, then stop and clean up.
Until Part 2 wires `lang2sim run`, this is the entry point.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from lang2sim.runtime import doctor, hardware_probe  # noqa: E402
from lang2sim.runtime import platform as host_platform  # noqa: E402
from lang2sim.runtime.display import DISPLAY_MODES, choose_display  # noqa: E402
from lang2sim.runtime.run_folder import RunFolderError, load_manifest  # noqa: E402
from lang2sim.runtime.session import SimNotReady, SimSession, copy_fixture  # noqa: E402


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("run_dir", nargs="?", type=Path, help="run folder under ~/.lang2sim/projects")
    src.add_argument("--fixture", help="copy catalog/fixtures/<name> into a new run and open it")
    ap.add_argument("--display", choices=DISPLAY_MODES, help="override the auto-chosen display mode")
    ap.add_argument("--software", action="store_true", help="force CPU rendering")
    ap.add_argument("--image", help="sim image (default: $L2S_IMAGE or the pinned GHCR image)")
    ap.add_argument("--dev", action="store_true", help="mount this checkout's lang2sim/ into the container")
    ap.add_argument("--no-gui", action="store_true", help="no Gazebo window")
    ap.add_argument("--no-rviz", action="store_true")
    ap.add_argument("--no-camera", action="store_true", help="skip the camera bridge (faster without a GPU)")
    ap.add_argument("--mission", action="store_true", help="drive spec.json's waypoints + goal once ready")
    ap.add_argument("--timeout", type=float, default=300, help="seconds to wait for readiness")
    ap.add_argument("--exit-when-ready", action="store_true", help="stop as soon as the sim is ready (tests)")
    return ap.parse_args(argv)


def progress(st, elapsed):
    done = [k for k, v in st.readiness.items() if v]
    todo = [k for k, v in st.readiness.items() if not v]
    print(f"  {elapsed:5.0f}s  {st.status:<10} ready: {', '.join(done) or '-'}"
          + (f"  waiting: {', '.join(todo)}" if todo else ""), flush=True)


def nav_progress(st):
    print(f"  {st.status:<10} waypoint {st.waypoint_index + 1}/{st.waypoints_total}  "
          f"{st.distance_remaining:5.1f} m to go" + (f"  ({st.message})" if st.message else ""), flush=True)


async def run(a) -> int:
    run_dir = copy_fixture(a.fixture) if a.fixture else a.run_dir.expanduser().resolve()
    try:
        load_manifest(run_dir)
    except RunFolderError as e:
        print(f"[run_sim] {e}")
        return 2
    print(f"[run_sim] run folder: {run_dir}")

    plat, hw = host_platform.detect(), hardware_probe.probe()
    image = a.image or os.environ.get("L2S_IMAGE") or None
    checks = doctor.run_checks(plat, hw, prefer_display=a.display, image=image)
    print(doctor.format_report(checks))
    if doctor.exit_code(checks):
        return 1
    plan = choose_display(plat, hw, prefer=a.display, force_software=a.software)
    print(f"[run_sim] display: {plan.describe()}")

    extra = {"L2S_CAMERA": "0"} if a.no_camera else {}
    session = SimSession(plan, image=image, dev_src=REPO if a.dev else None, host_env=plat.env,
                         has_dxg=hw.has_dxg, extra_env=extra)
    print(f"[run_sim] starting {session.opts.image} ...")
    try:
        await session.start()
        print("[run_sim] control API up; launching world + robot + Nav2")
        st = await session.open_run(run_dir, gui=not a.no_gui, rviz=not a.no_rviz,
                                    timeout=a.timeout, on_progress=progress)
        for line in st.world_report:
            print(f"[run_sim] world check {line}")
        print(f"[run_sim] READY  sim time {st.sim_time:.1f}s, RTF {st.rtf:.2f}. {session.view_hint}.")
        if a.mission:
            print("[run_sim] driving the mission from spec.json")
            nav = await session.run_mission(run_dir, on_progress=nav_progress)
            print(f"[run_sim] MISSION {nav.status.upper()}" + (f": {nav.message}" if nav.message else ""))
            if a.exit_when_ready:
                return 0 if nav.status == "succeeded" else 1
        if a.exit_when_ready:
            return 0
        print("[run_sim] Ctrl-C to stop.")
        while True:
            await asyncio.sleep(5)
            st = await session.api.sim_status()
            if st.status in ("error", "stopped"):
                print(f"[run_sim] simulation {st.status}; last log lines:")
                print("\n".join(await session.api.sim_logs(40)))
                return 1
    except SimNotReady as e:
        print(f"[run_sim] {e}\n{e.logs}")
        return 1
    finally:
        print("[run_sim] stopping")
        await session.stop()
        print(f"[run_sim] logs: {run_dir / 'logs'}")


def main(argv=None) -> int:
    a = parse(argv)
    try:
        return asyncio.run(run(a))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
