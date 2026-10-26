#!/usr/bin/env python3
"""
One prompt -> running simulation (pitch build, branch demo/pitch-e2e).
File: scripts/prompt_to_sim.py

    python scripts/prompt_to_sim.py "a warehouse with 4 racks and 2 crates, dim lighting"
    python scripts/prompt_to_sim.py "an empty room" --no-llm
    python scripts/prompt_to_sim.py "..." --display vnc        # macOS: http://localhost:6080

Uses the LLM when NVIDIA_API_KEY / OPENAI_API_KEY is set, else the scenario parser.
Extra flags go to scripts/run_sim.py (--software, --no-camera, --no-rviz, ...).
"""

import asyncio
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from lang2sim.pipeline import (  # noqa: E402
    build_run,
    describe,
    launch_and_drive,
    make_session,
    prompt_to_spec,
)


def main() -> int:
    args = sys.argv[1:]
    if not args or args[0].startswith("-"):
        print(__doc__)
        return 2
    prompt, rest = args[0], args[1:]
    use_llm = "--no-llm" not in rest
    rest = [a for a in rest if a != "--no-llm"]

    print(f'[1/4] Prompt: "{prompt}"')
    spec, source = asyncio.run(prompt_to_spec(prompt, use_llm=use_llm))
    print(f"[2/4] Spec from {source}:\n      {describe(spec)}")
    run, steps = build_run(spec)
    print("[3/4] Part 1 world builder:")
    for s in steps:
        print(f"      {s}")
    display = rest[rest.index("--display") + 1] if "--display" in rest else None
    session, plan = make_session(display=display, software="--software" in rest,
                                 camera="--no-camera" not in rest)
    print(f"[4/4] Part 3 runtime ({plan.describe()}): starting {session.opts.image}")
    try:
        return asyncio.run(_drive(session, run, gui="--no-gui" not in rest, rviz="--no-rviz" not in rest))
    except KeyboardInterrupt:  # session.stop() already ran in _drive's finally
        print("\nstopped.")
        return 0


async def _drive(session, run, gui, rviz) -> int:
    try:
        await session.start()
        print(f"      {session.view_hint}")
        nav, report = await launch_and_drive(session, run, gui=gui, rviz=rviz,
                                             on_event=lambda s: print(f"      {s}", flush=True))
        print(f"\nMISSION {nav.status.upper()}" + (f": {nav.message}" if nav.message else ""))
        if report and "nqi" in report:
            import json

            from lang2sim.telemetry.nqi_scorer import render_to_console
            render_to_console(json.loads((run / "benchmark_report.json").read_text()))
            print(f"Report: {report['report_path']}")
        print(f"Dashboard: http://localhost:{session.opts.api_port}/dashboard   "
              "TUI: lang2sim, then /mission stress, /stress, /report")
        print("Simulation still running - Ctrl-C to stop.")
        while True:
            await asyncio.sleep(5)
    except (KeyboardInterrupt, asyncio.CancelledError):
        return 0
    finally:
        await session.stop()


if __name__ == "__main__":
    sys.exit(main())
