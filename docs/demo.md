# Demo runbook (3-minute pitch)

Maps the pitch in the architecture doc (§8) to exact commands, with a fallback
for every step that can fail on stage. Telemetry internals: [telemetry.md](telemetry.md).

## Before you go on stage (T−15 min)

1. Laptop on power, Docker Desktop running (macOS: 8 GB, 4+ CPUs), and the host extras
   installed once: `pip install -e ".[sim,dev]"` (FastAPI/uvicorn are needed for `/replay`).
   Rehearse the fallback once: `python -m lang2sim.telemetry.replay catalog/demo_runs/empty_room_benchmark --loop --port 9191`
   and open `http://localhost:9191/dashboard`.
2. Start the app with your LLM key set (without a key the scenario parser builds the world
   and the demo commands still work; only free-form chat needs the key):
   ```bash
   export NVIDIA_API_KEY=nvapi-...      # or OPENAI_API_KEY
   lang2sim
   ```
3. Warm-up, about 3–5 min, so the image is pulled and a nominal score exists before the
   judges arrive. In the workspace type:
   ```
   /sim build a dark warehouse with 3 racks and 2 crates
   ```
   The chat shows each stage: spec (LLM or parser), Part 1 world + map, container start,
   READY, the mission, then the NQI table. Gazebo/RViz open on Ubuntu (macOS:
   `http://localhost:6080` with the VNC image, or Foxglove).
4. Browser: open `http://localhost:9090/dashboard`, full screen on the projector.
   The badge should read **LIVE SIM**.

   Alternative without the TUI: `bash pitch_demo.sh "a dark warehouse with 3 racks and 2 crates"`
   (Ubuntu; builds the image if needed), or `python scripts/run_sim.py --fixture empty_room`
   and then `lang2sim` (the app attaches to any running sim).

## On stage

| Beat | You do | They see |
| --- | --- | --- |
| **1. Hook** (0:00–0:30) | Talk. Dashboard on screen (the warm-up run is already scored). | Map, racks, robot, live tiles, nominal NQI |
| **2. Wow** (0:30–1:15) | `/mission stress`, then talk while it drives | Robot moves, live trail, `● REC ADVERSARIAL` |
| **3. Twist** (1:15–2:15) | Type the stress prompt when the robot is mid-leg (below) | Lights dim in Gazebo, crate drops ahead, robot detours; clearance chart dives toward the C_safe line |
| **4. Payoff** (2:15–3:00) | Mission ends by itself | NQI table in the TUI and the dashboard: nominal vs stress, Δ% |

Beat 3 prompt (plain English works; the app shows what it executed):

```
Stress-test navigation: drop ambient lighting to 10% and spawn a crate directly in the robot's path
```

Or the exact command: `/stress light=0.1 model=wooden_crate`. A rack wall works too:
*"put a rack across the aisle in front of the robot"*, or at a spot: *"drop a barrel at 1.5, 0"*.

Live steering, if the judges ask "what if…": *"detour and reach rack 2"* (re-plans with
cleared costmaps), *"stop the robot"* (sim keeps running), *"resume"*, and `/stop` to end.

Then ask the copilot, for example *"why did the NQI drop?"*. It answers from the
measured numbers (clearance term, detour length, jerk), because each chat turn
carries the live telemetry and the report.

No time for Beats 2–3? `/benchmark` does nominal → back to start → stress
(crate dropped automatically, lights 10%) and prints the table (about 5 min of sim).

## If something breaks

| Symptom | Do this |
| --- | --- |
| Sim crashed / Docker died mid-run | The run was streamed to disk: `python -m lang2sim.telemetry.benchmark ~/.lang2sim/projects/<project>/runs/<ts>` scores what was recorded. |
| Sim won't come back in time | Stop the container (`docker rm -f lang2sim-sim`), then in the TUI type `/replay demo`. It plays the recorded real benchmark (`catalog/demo_runs/empty_room_benchmark`) on port 9090, so the TUI, the dashboard (badge **REPLAY**) and the copilot keep working. Outside the TUI: `python -m lang2sim.telemetry.replay catalog/demo_runs/empty_room_benchmark --loop` |
| Crate didn't spawn ("no free spot ahead") | The robot is too close to a waypoint; wait 2–3 s and send it again. |
| A sentence started a new simulation by mistake | Only "build/create/simulate … warehouse/room/world" without a question mark does that; `/stop` ends it. Questions always go to the chat. |
| Dashboard blank | Reload it. It reconnects by itself after sim restarts. |
| Copilot says it can't reach the API | The LLM key isn't set; the demo commands still work without it. |

## What the numbers mean (for Q&A)

- **Sim-time stamps**: scores don't depend on how fast the laptop is.
- **Ground-truth poses**: Gazebo's own pose of the robot, so AMCL errors don't hide in the score (we report them separately as localization error).
- **Collisions**: a real contact sensor on the chassis, > 5 N.
- **C_min**: closest distance between the robot's chassis outline and any obstacle, including the crate.
- **Mission success**: within Nav2's own goal tolerance for the run (0.25 m for the TurtleBot3); `--goal-tolerance 0.15` applies the architecture's stricter Archetype 1 criterion.
- Reference run shipped in the repo (`/benchmark`, empty_room, crate + 10% light): NQI **81.6 → 39.8**, path +9.6%, smoothness S_τ +117%, closest approach 6 cm, 0 collisions, both missions succeeded.
- Rehearsal of this exact script in the TUI (empty_room, software rendering, RTF ≈ 0.5): `/mission` NQI **80.0**, then `/mission stress` + the Beat 3 sentence mid-leg: **59.0 (−26.3%)**, path +9.2%, S_τ +71%, closest approach 0.30 m, 0 collisions, both succeeded. Scores vary with where the crate lands; the shape (detour, jerk, near-miss, no contact) is consistent.
