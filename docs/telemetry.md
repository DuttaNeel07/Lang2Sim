# Telemetry engine: how it is wired

The Telemetry Engine (architecture §4.1, §6 Phase 3–4, §7) turns a running
simulation into live metrics and the NQI `benchmark_report.json`. Part 4 built
the scorer (`lang2sim/telemetry/nqi_scorer.py`); this page covers how real runs
reach it.

```
 sim container                                        host
 ───────────────────────────────────────────────     ───────────────────────────────
 Gazebo  ─ robot.l2s.sdf ─┬─ /lang2sim/ground_truth/odom ─┐
   (bumper + ground truth)└─ /lang2sim/contact_states ───┤
 /odom /scan /clock TF(AMCL) ─────────────────────────────┤
                                                          ▼
                    RosBridge ── TelemetryRecorder + ContactMonitor
                                  (sim time, world frame, rising-edge contacts)
                                                          │
         control API  /v1/telemetry/live ───────────────────────▶ TUI sidebar, agent tool,
                      /v1/nav/feedback (WS "live")                TelemetryCollector
                      /v1/telemetry/record/start|stop ──────────▶ SimSession.run_recorded_mission
                      /v1/world/spawn (footprint log)              └ telemetry/benchmark.py
                                                                     └ <run>/benchmark_report.json
```

## Rules that keep scores comparable

| Rule | Why |
| --- | --- |
| Samples are stamped in **sim time** | S_τ is a jerk integral; wall-clock stamps made a slow Mac (RTF 0.3) score far worse than a GPU box for the same motion. |
| Poses are **world frame**: Gazebo ground truth > AMCL map pose > `/odom` + spawn offset | Raw `/odom` starts at the spawn point, so goals, map and obstacles were offset whenever the robot did not start at 0,0. One run never mixes sources. |
| Collisions are **rising-edge chassis contacts** in sim time, ground and self ignored, > 5 N | Architecture §7.4. Gazebo fills the contact wrench, so the force threshold is real; a contact without force data still counts ("geometric"). |
| L_optimal is the **whole mission** (start → waypoints → goal), A* on the inflated map | A patrol is not penalised against a straight start→goal line. |
| Spawned obstacles are scored by their **real footprint** (box outline) | They are not in `map.pgm`; a bounding circle reported near-misses that never happened. |

## The plan, step by step (one commit each)

1. **Contract + recorder**: `core/models.py` (`TelemetryLive`, `TelemetryRecordRequest`, `TelemetryRecordResult`), `runtime/telemetry_recorder.py` (stdlib only, runs in the container, outputs `Trajectory.to_dict()`).
2. **Instrument the robot**: `runtime/robot_instrument.py` writes `robot.l2s.sdf` (contact sensor on `base_collision` + `OdometryPublisher` with `odom_frame=world`) and `bridge.l2s.yaml` per run; `world_check` adds the Contact system. `L2S_TELEMETRY=0` uses the stock robot.
3. **Feed the recorder** from `RosBridge` callbacks (ground truth, `/odom`, `/scan`, contacts).
4. **Control API**: `/v1/telemetry/live`, `/setup`, `/record/start`, `/record/stop` (also writes `logs/trajectory_<label>.json`), WS `live` key, spawn footprint log.
5. **Score real runs**: `telemetry/benchmark.py`, `SimSession.run_recorded_mission` / `run_benchmark` / `score_single_run`, `run_sim.py --mission` (scored) and `--benchmark`.
6. **Consumers**: TUI sidebar (1 s poll in a thread worker), agent `query_robot_telemetry()`, `TelemetryCollector` control-API backend.
7. **Fixes found running it for real**: sim image shipping the renamed package, `/v1/world` mutations in the container, exact box clearance, stress crate kept off waypoints, Docker integration tests.

## Using it

```bash
python scripts/run_sim.py --fixture small_warehouse --mission              # one scored run
python scripts/run_sim.py --fixture small_warehouse --benchmark --light 0.1  # nominal vs stress table
curl -s localhost:9090/v1/telemetry/live                                    # live metrics
```

`--benchmark` uses `spec.json`'s `adversarial_triggers` (immediate / time /
waypoint_distance), or by default drops a wooden crate on the nominal path,
≥ 1.2 m from any waypoint, while the robot is ~35% of the way there.

## Verified in the sim container

Integration tests (`L2S_INTEGRATION=1 pytest tests/integration -k telemetry`)
and a full benchmark on `empty_room`, headless, software rendering, RTF ≈ 0.6:

| Metric | Nominal | Stress (crate + 10% light) |
| --- | --- | --- |
| Nav2 result | succeeded (goal error 0.152 m) | succeeded (0.082 m) |
| Duration (sim s) | 60.6 | 150.2 |
| Path length | 13.41 m | 14.69 m (+9.6%, detour) |
| E_p / S_τ | 0.97 / 47.0 | 0.89 / 102.0 |
| C_min | 0.89 m | −0.01 m (lidar: ~0.07 m from chassis) |
| C_force | 0 | 0 |
| NQI | 81.6 | 35.2 |
| AMCL vs ground truth | 0.10 m mean | 0.06 m mean |

## Open decisions

- **Goal tolerance**: the architecture's 0.15 m marks a run that Nav2 calls
  succeeded (TB3 goal checker: 0.25 m, final error 0.152 m) as failed.
  `--goal-tolerance` sets it per run; pick a team default.
- **Robot footprint in C_min**: the scorer models the Waffle as a 0.22 m
  circle, so a 7 cm pass reads as −0.01 m and zeroes the clearance term
  (~40 NQI points). An oriented-box robot footprint would fix it.
- **"SLAM Drift (ATE)" row**: the pipeline is zero-SLAM, so this column is
  AMCL-vs-ground-truth localization error; the table label could say so.
- **Failed missions** keep E_p from the distance actually driven; NQI has no
  mission-success term in the architecture's formula.
