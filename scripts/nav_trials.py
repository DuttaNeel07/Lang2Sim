"""A/B harness: drop obstacles on the robot's real lane mid-mission and record how Nav2 copes.

usage: python scripts/nav_trials.py <fixture> <profile> <out.json> [scenario ...]
scenario = model:leg:ahead_m[:lateral_m]   e.g. wooden_crate:0:1.0  barrel:2:1.2:0.2
"""
import asyncio, dataclasses, json, math, subprocess, sys, time
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))

FIX, PROFILE, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
SCEN = sys.argv[4:] or ["wooden_crate:0:1.0", "wooden_crate:1:2.0", "barrel:2:1.2"]
LOG = open(OUT.replace(".json", ".log"), "w")
def log(*a):
    print(time.strftime("%H:%M:%S"), *a, file=LOG, flush=True)

def gz_remove(world, name):
    cmd = (f"source /opt/ros/jazzy/setup.bash; gz service -s /world/{world}/remove --reqtype gz.msgs.Entity "
           f"--reptype gz.msgs.Boolean --timeout 3000 --req 'name: \"{name}\" type: MODEL'")
    r = subprocess.run(["docker", "exec", "lang2sim-sim", "bash", "-c", cmd], capture_output=True, text=True)
    return r.stdout.strip() + r.stderr.strip()

def along(path, i0, d):
    acc = 0.0
    for i in range(i0, len(path) - 1):
        (ax, ay), (bx, by) = path[i], path[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        if acc + seg >= d:
            f = (d - acc) / seg if seg else 0
            return ax + f * (bx - ax), ay + f * (by - ay), math.atan2(by - ay, bx - ax)
        acc += seg
    return None

def end_err(run):
    s = run.result.trajectory["samples"][-1]
    g = run.result.trajectory.get("goal") or [0, 0]
    gt = math.hypot(s["x"] - g[0], s["y"] - g[1])
    loc = math.hypot(s.get("slam_x", s["x"]) - s["x"], s.get("slam_y", s["y"]) - s["y"])
    return {"gt_to_goal": round(gt, 3), "amcl_err": round(loc, 3)}

async def main():
    from lang2sim.pipeline import make_session
    from lang2sim.runtime.session import copy_fixture
    run_dir = copy_fixture(FIX, project=f"navexp_{PROFILE}")
    session, plan = make_session(software=True, camera=False)
    session.opts = dataclasses.replace(session.opts, extra_env={**session.opts.extra_env, "L2S_NAV_PROFILE": PROFILE})
    results = {"fixture": FIX, "profile": PROFILE, "run_dir": str(run_dir), "trials": []}
    world = json.loads((run_dir / "manifest.json").read_text()).get("world_name")
    try:
        await session.start()
        st = await session.open_run(run_dir, gui=False, rviz=False)
        log("ready", st.rtf if hasattr(st, "rtf") else "")
        nom = await session.run_recorded_mission(run_dir, label="nominal")
        path = [(p["x"], p["y"]) for p in nom.result.trajectory["samples"]]
        results["nominal"] = {"status": nom.nav.status, "msg": nom.nav.message, "nqi": nom.scored.get("nqi"),
                              "dur": nom.result.duration_s, "coll": nom.result.collisions, "S": nom.scored.get("S_mission"),
                              "cmin": nom.scored.get("C_min_m"), "end": end_err(nom)}
        log("nominal", results["nominal"])
        spawned = []
        for k, sc in enumerate(SCEN):
            model, leg, ahead, *lat = sc.split(":")
            leg, ahead, lat = int(leg), float(ahead), float(lat[0]) if lat else 0.0
            for name in spawned:
                try:
                    log("remove", name, await session.api.world_remove(name))
                except Exception as e:  # stock image code: no remove endpoint
                    log("remove", name, gz_remove(world, name), repr(e)[:120])
            spawned = []
            await session.return_to_start(run_dir)
            await asyncio.sleep(2)
            info = {"scenario": sc}
            max_rec = {}
            async def watcher():
                fired = False
                leg_start = None
                while True:
                    await asyncio.sleep(0.5)
                    try:
                        nav = await session.api.nav_status(); live = await session.api.telemetry_live()
                    except Exception:
                        continue
                    if nav.is_active:
                        max_rec[nav.waypoint_index] = max(max_rec.get(nav.waypoint_index, 0), nav.recoveries)
                    if fired or not live.recording or live.pose is None or not nav.is_active:
                        continue
                    if nav.waypoint_index != leg:
                        continue
                    if leg_start is None:
                        leg_start = (live.pose.x, live.pose.y)
                    if math.hypot(live.pose.x - leg_start[0], live.pose.y - leg_start[1]) < 0.8:
                        continue
                    x, y = live.pose.x, live.pose.y
                    i0 = min(range(len(path)), key=lambda i: (path[i][0] - x) ** 2 + (path[i][1] - y) ** 2)
                    spot = along(path, i0, ahead)
                    if spot is None:
                        continue
                    sx, sy, tan = spot
                    sx, sy = sx - lat * math.sin(tan), sy + lat * math.cos(tan)
                    res = await session.api.world_spawn({"model_name": model, "x": sx, "y": sy, "yaw": tan})
                    name = (res.get("data") or {}).get("entity_name") or model
                    spawned.append(name)
                    info.update(spawn=(round(sx, 2), round(sy, 2)), robot=(round(x, 2), round(y, 2)),
                                dist=round(math.hypot(sx - x, sy - y), 2), name=name)
                    log("spawned", sc, info)
                    fired = True
            w = asyncio.ensure_future(watcher())
            t0 = time.time()
            try:
                run = await session.run_recorded_mission(run_dir, label="adversarial")
            finally:
                w.cancel()
            info.update(status=run.nav.status, msg=run.nav.message, nqi=run.scored.get("nqi"),
                        S=run.scored.get("S_mission"), cmin=run.scored.get("C_min_m"),
                        dur=run.result.duration_s, coll=run.result.collisions, recoveries=max_rec, end=end_err(run),
                        wall_s=round(time.time() - t0))
            log("trial", info)
            results["trials"].append(info)
            json.dump(results, open(OUT, "w"), indent=1)
    finally:
        json.dump(results, open(OUT, "w"), indent=1)
        await session.stop()

asyncio.run(main())
