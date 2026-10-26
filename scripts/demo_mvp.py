#!/usr/bin/env python3
"""
Lang2Sim MVP demo: one command from the host to an autonomously navigating robot.
File: scripts/demo_mvp.py

    python scripts/demo_mvp.py                 # auto display (X11 on Ubuntu)
    python scripts/demo_mvp.py --loop 0 --hold # tour forever, keep windows open
    python scripts/demo_mvp.py --display vnc   # macOS / any OS: http://localhost:6080

Pipeline it exercises (Part 3):
  doctor checks -> platform/hardware probe -> display plan (x11|wslg|vnc|headless,
  nvidia|dri|software) -> DockerRunner starts the sim image -> entrypoint sets up
  display/GPU -> Gazebo Harmonic + TurtleBot3 + ros_gz_bridge + Nav2 (+ Gazebo GUI,
  RViz) -> Nav2 mission over a waypoint tour -> logs land in
  ~/.lang2sim/projects/mvp_demo/runs/<timestamp>/.
"""

from __future__ import annotations

import argparse
import dataclasses
import importlib
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from sim2lang.runtime import doctor, hardware_probe  # noqa: E402
from sim2lang.runtime import platform as host_platform  # noqa: E402
from sim2lang.runtime.display import DISPLAY_MODES, choose_display  # noqa: E402
from sim2lang.runtime.docker_runner import (  # noqa: E402
    DockerRunner,
    default_run_options,
    host_to_container_path,
)

cfg = importlib.import_module("sim2lang.core.config")
NAME = "lang2sim-demo"


def main() -> int:
    sys.stdout.reconfigure(line_buffering=True)  # keep our lines in order with `docker logs -f`
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--display", choices=DISPLAY_MODES, help="override the auto-chosen display mode")
    ap.add_argument("--software", action="store_true", help="force CPU rendering")
    ap.add_argument("--image", default=None, help="sim image (default: $L2S_IMAGE or lang2sim-sim:dev)")
    ap.add_argument("--loop", type=int, default=1, help="tours to drive (0 = forever)")
    ap.add_argument("--hold", action="store_true", help="keep the sim open after the mission")
    ap.add_argument("--no-rviz", action="store_true")
    ap.add_argument("--no-gazebo-gui", action="store_true")
    a = ap.parse_args()

    image = a.image or os.environ.get("L2S_IMAGE") or "lang2sim-sim:dev"

    # 1. Can we run at all?
    plat = host_platform.detect()
    hw = hardware_probe.probe()
    checks = doctor.run_checks(plat, hw, prefer_display=a.display, image=image)
    print(doctor.format_report(checks))
    if doctor.exit_code(checks):
        return 1

    # 2. How will the user see it?
    plan = choose_display(plat, hw, prefer=a.display, force_software=a.software)
    print(f"\n[demo] display plan: {plan.describe()}")

    # 3. Run directory on the host, mounted into the container.
    run_dir = cfg.get_run_dir("mvp_demo")
    projects = cfg.get_projects_dir()
    c_run_dir = host_to_container_path(run_dir, projects)

    inside = ["/opt/lang2sim/src/scripts/demo/mvp_inside.sh", "--loop", str(a.loop)]
    env = {"L2S_DEMO_RUN_DIR": c_run_dir, "L2S_DEMO_HOLD": "1" if a.hold else "0"}
    if a.no_rviz or not plan.has_gui:
        env["L2S_DEMO_RVIZ"] = "false"
    if a.no_gazebo_gui or not plan.has_gui:
        env["L2S_DEMO_GUI"] = "false"

    opts = default_run_options(plan, image=image, project_src=REPO, host_env=plat.env, has_dxg=hw.has_dxg)
    opts = dataclasses.replace(opts, name=NAME, extra_env=env,
                               command=("bash", "-c", " ".join(inside)))

    runner = DockerRunner()
    print(f"[demo] starting {opts.image} as '{NAME}'")
    runner.start(opts)
    if plan.mode == "vnc":
        print(f"[demo] open http://localhost:{opts.novnc_port}  to watch Gazebo + RViz")
    if plan.foxglove:
        print(f"[demo] Foxglove: connect to ws://localhost:{opts.foxglove_port}")

    # 4. Follow the container until the mission (or --hold) ends.
    rc = 1
    try:
        subprocess.run(["docker", "logs", "-f", NAME], check=False)
        st = runner.state(NAME) or {}
        rc = int(st.get("ExitCode", 1))
    except KeyboardInterrupt:
        print("\n[demo] Ctrl-C: stopping")
        runner.stop(NAME, timeout=10)
        rc = 130
    finally:
        runner.remove(NAME)

    print(f"[demo] logs: {run_dir}")
    print("[demo] RESULT:", "PASS" if rc == 0 else f"FAIL (exit {rc})")
    return rc


if __name__ == "__main__":
    sys.exit(main())
