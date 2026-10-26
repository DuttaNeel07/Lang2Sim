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
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from sim2lang.pipeline import build_run, describe, prompt_to_spec  # noqa: E402


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
    print("[4/4] Part 3 runtime: container -> pre-flight -> Gazebo + TurtleBot3 + Nav2 -> mission")
    return subprocess.call([sys.executable, str(REPO / "scripts" / "run_sim.py"), str(run), "--mission", *rest])


if __name__ == "__main__":
    sys.exit(main())
