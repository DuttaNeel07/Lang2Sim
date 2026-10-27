# Robot navigation: rerouting around dropped obstacles

How the TurtleBot3 gets around obstacles that appear on its path mid-mission, what
was changed to make it more reliable, and how a failure is reported. Telemetry and
scoring: [telemetry.md](telemetry.md).

## What happens when an obstacle appears

1. The laser (`/scan`, 360 beams, 0.12–3.5 m) marks it in both costmaps. The
   global costmap updates at 2 Hz (stock: 1 Hz).
2. The behaviour tree (`<run>/navigate_l2s.xml`) replans the global path at 2 Hz
   (stock: 1 Hz) with **Smac 2D**, whose travel cost keeps the new path in the
   middle of the gap beside the obstacle. Inflation is 0.55 m with a gentle 3.0
   decay (was 0.35 m / 5.0), so the middle is clearly the cheapest line.
3. **Regulated Pure Pursuit** follows that path. It slows down near obstacles,
   turns in place toward a sharp detour instead of arcing into the obstacle, and
   stops if the path ahead would collide. The stock DWB controller scored
   trajectories only ~1.5 s ahead, cut the detour's corner and ended nose-to-crate
   with 21 recoveries until the mission timed out (measured below).
4. If the robot still gets boxed in (no progress for 15 s, counting turning in
   place as progress), Nav2's recoveries run in this order: clear the costmaps,
   **back up 25 cm**, spin 90°, wait 3 s. Stock spins before backing up, which
   fails when the robot is nose-to-crate. Up to 10 rounds (stock: 6).
5. If Nav2 still gives up on that leg, the dispatcher retries the leg twice more
   after clearing the costmaps and backing up 20 cm
   (`runtime/nav2_dispatcher.py`, `LEG_RETRIES`). Before, one failed leg ended the
   whole mission.

The costmaps and the collision monitor use the Waffle's real chassis rectangle
(the one the NQI clearance uses) instead of a 0.15 m circle that is smaller than
the robot. The collision monitor's slow-down zone starts 1.5 s before contact
(stock: 2.0 s, which slowed the robot almost to a stop about 0.6 m from a crate).

### "Nav2 succeeded" but the score said "failed"

The score checks the robot's **real** (Gazebo ground-truth) position against the
goal; Nav2 stops when its **estimate** (AMCL) is inside its goal checker. With
both at 0.25 m, AMCL's 0.15–0.19 m error at the goal ended runs 0.28–0.34 m from
it: Nav2 reported success and the score reported a failed mission. Now:

- Nav2 aims for 0.12 m (`NAV_GOAL_TOLERANCE_M`), the score keeps the mission's
  0.25 m (`<run>/scoring.l2s.json`, read by `telemetry/benchmark.py`), so
  localization error has room.
- AMCL trusts Gazebo's near-exact wheel odometry more (motion noise alphas 0.05
  instead of 0.2) and updates every 10 cm / 0.1 rad (stock: 25 cm / 0.2 rad).
  Its error at the goal fell from 0.16–0.19 m to about 0.10 m.

`L2S_NAV_PROFILE=stock` brings back the stock TurtleBot3 settings for comparison.
The retries, the route check and the failure explanations below stay on either way.

## Obstacles that leave a way around

When the app places an obstacle from a sentence ("drop a crate in front of the
robot"), it rejects any spot that closes every route the robot fits through
(`route_exists` in `cli/demo_commands.py`). The check uses the map shrunk by the
robot's clearance, plus every obstacle dropped so far, and checks the robot's
next waypoint and all the ones after it. If you give exact coordinates that block
the route, the obstacle is still dropped, with a warning that the run will be
scored as failed.

*"remove the obstacles"* (or *"clear the path"*) removes everything dropped into
the running simulation (`POST /v1/world/remove`). The world's own racks and crates
stay.

## When a mission fails

The chat says which of two things happened:

- **Waypoint N was unreachable:** the dropped obstacles closed every route. That
  is a failure of the scenario, not of the robot, and no planner could do better.
- **A route existed, but Nav2 stopped trying:** the robot got boxed in and its
  recoveries ran out. The Nav2 reason is quoted.

A mission that only got through after a retry says so ("Recovered: leg 2 reached
on retry 1").

## Measured (small_warehouse, 2 CPU cores, software rendering, RTF ≈ 0.55–0.6)

Obstacles were dropped on the robot's recorded lane, the given distance ahead,
while it drove (`model:leg:metres ahead[:lateral offset]`, harness:
`scripts/nav_trials.py`). Every row: Nav2 status / mission success in
the score / closest approach C_min / NQI. No run had a contact over 5 N.

| Scenario | Stock TurtleBot3 (DWB, NavFn) | DWB + real footprint, 2 Hz replanning | **Now** (RPP + Smac 2D + AMCL + goal margin) |
| --- | --- | --- | --- |
| nominal lap | succeeded, 136 s, NQI 59.9 | succeeded, 121 s, NQI 60.3 | succeeded / **reached** (0.15 m from goal), 127 s, C_min 0.39 m, **NQI 72.4** |
| crate 2.0 m ahead, leg 2, 0.4 m off-centre (~(2.9, 1.0)) | | **stuck nose-to-crate, 21 recoveries in 250 s, mission failed** | succeeded / reached, 0 recoveries, C_min 0.25 m, NQI 59.8 |
| crate 2.0 m ahead, leg 2 | succeeded, C_min 0.13 m, NQI 47.2 | | succeeded / reached, 0 recoveries, C_min **0.33 m**, NQI **63.7** |
| crate 1.0 m ahead, leg 1 | succeeded, C_min 0.07 m, NQI 41.0 | succeeded, C_min 0.01 m, NQI 35.8 | succeeded / reached, 0 recoveries, C_min **0.29 m**, NQI **59.0** |
| barrel 1.2 m ahead, leg 3 | succeeded, C_min 0.17 m, NQI 49.2 | | succeeded / reached, C_min 0.16 m, NQI 48.3 |
| crate 0.8 m ahead, leg 4 | succeeded, C_min 0.16 m, NQI 47.7 | | succeeded / reached, C_min 0.00 m (grazed, no contact force), NQI 39.9 |

With RPP alone (before the AMCL and goal-margin change) the same crate run ended
with Nav2 "succeeded" but the score "failed": the robot stopped 0.28 m from the
goal by ground truth while AMCL put it 0.24 m away. With the change, every run
ended 0.15–0.18 m from the goal and AMCL's error there was 0.09–0.11 m.

What's left: an obstacle dropped less than ~1 m ahead (closer than the app's own
placement, which uses 1.2–3.2 m) can still be grazed, because the robot is
already committed to the old line when the laser first sees it. In the stock
runs, removing obstacles between trials failed (no remove endpoint in that
build), so later stock trials also had the earlier obstacles in the world.

MPPI was also tried in place of DWB. With the real footprint it stalled for about
4 minutes at a waypoint between racks: 14 "failed to make progress" aborts on the
nominal lap, which took 332 s instead of 121 s. So it is not used.
