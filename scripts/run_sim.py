#!/usr/bin/env python3
"""
Open any run folder in the simulator: one command, any OS.
File: scripts/run_sim.py

    python scripts/run_sim.py ~/.lang2sim/projects/demo/runs/20261026_150000
    python scripts/run_sim.py --fixture small_warehouse
    python scripts/run_sim.py --fixture tb3_world --display vnc     # macOS: real Gazebo GUI in a browser
    python scripts/run_sim.py --fixture small_warehouse --mission  # also drive spec.json's mission
    python scripts/run_sim.py --fixture small_warehouse --benchmark --light 0.1   # nominal vs stress NQI

--mission records the run's telemetry and writes <run>/benchmark_report.json
(NQI, nominal only). --benchmark runs the mission twice, nominal then with the
spec's adversarial triggers (default: a crate dropped on the robot's path),
and prints the nominal vs adversarial table (architecture §8.4).

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
    ap.add_argument("--benchmark", action="store_true",
                    help="nominal + adversarial runs of the mission, scored into benchmark_report.json")
    ap.add_argument("--light", type=float, help="with --benchmark: sun intensity 0..1 during the stress run")
    ap.add_argument("--goal-tolerance", type=float, default=0.15,
                    help="NQI mission success radius in m (architecture: 0.15; Nav2 TB3 goal checker: 0.25)")
    ap.add_argument("--no-score", action="store_true", help="with --mission: drive only, no telemetry/NQI")
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


def _params(a):
    from lang2sim.telemetry.nqi_scorer import NQIParams
    return NQIParams(eps_pos=a.goal_tolerance)


def _print_report(report) -> None:
    from lang2sim.telemetry.nqi_scorer import render_to_console
    render_to_console(report)
    for name, run in report["runs"].items():
        print(f"[run_sim] {name}: pose from {run.get('pose_source')}, collisions via "
              f"{run.get('contact_force_source')}, nav {run.get('nav_status')}, "
              f"goal error {run.get('final_goal_error_m')} m")
    print(f"[run_sim] report: {report['path']}")


async def scored_mission(session, run_dir, a):
    """--mission: drive with telemetry recording and write a nominal benchmark_report.json."""
    from lang2sim.runtime.control_api.client import ControlAPIError
    if not a.no_score:
        try:
            setup = await session.api.telemetry_setup()
        except ControlAPIError:
            setup = None
            print("[run_sim] this sim image has no telemetry API; driving without scoring (rebuild the image)")
        if setup is not None:
            if not setup.get("enabled"):
                print(f"[run_sim] telemetry: stock robot ({setup.get('reason')}); scoring from AMCL poses, "
                      "no collision sensing")
            report = await session.score_single_run(run_dir, on_progress=nav_progress, params=_params(a))
            _print_report(report)
            from lang2sim.core.models import NavStatus
            return NavStatus.model_validate(report["nav"])
    return await session.run_mission(run_dir, on_progress=nav_progress)


async def benchmark(session, run_dir, a):
    """--benchmark: nominal vs adversarial runs (architecture §8.4 table)."""
    def on_trigger(tr, err):
        pose = tr.get("spawn_pose") or {}
        print(f"[run_sim] stress: spawn {tr.get('model_name')} at ({pose.get('x')}, {pose.get('y')})"
              + (f" FAILED: {err}" if err else ""), flush=True)

    report = await session.run_benchmark(run_dir, light=a.light, on_progress=nav_progress, on_trigger=on_trigger,
                                         on_phase=lambda m: print(f"[run_sim] benchmark: {m}", flush=True),
                                         params=_params(a))
    _print_report(report)
    return report


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
        if a.benchmark:
            report = await benchmark(session, run_dir, a)
            if a.exit_when_ready:
                return 0 if report else 1
        elif a.mission:
            print("[run_sim] driving the mission from spec.json")
            nav = await scored_mission(session, run_dir, a)
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
