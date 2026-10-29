"""
MVP demo mission: localize, then drive a waypoint tour with Nav2.
File: scripts/demo/mvp_mission.py   (runs INSIDE the sim container)

A preview of Step 3's nav2_dispatcher (BasicNavigator wrapper). Goals are in
the `map` frame of turtlebot3_navigation2/map/map.yaml; each has >= 0.5 m
clearance from obstacles.

    python3 scripts/demo/mvp_mission.py [--loop N]
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from rclpy.parameter import Parameter

START = ("dock", -2.0, -0.5, 0.0)
TOUR = [
    ("centre gap", 0.55, -0.55, 0.0),
    ("east bay", 2.0, 0.5, math.pi / 2),
    ("north bay", 0.5, 1.8, math.pi),
    ("west gap", -0.55, 0.55, -math.pi / 2),
    ("dock", -2.0, -0.5, math.pi),
]


def pose(nav: BasicNavigator, x: float, y: float, yaw: float) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z = math.sin(yaw / 2)
    p.pose.orientation.w = math.cos(yaw / 2)
    return p


def log(msg: str) -> None:
    print(f"[mission] {msg}", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--loop", type=int, default=1, help="times to run the tour (0 = forever)")
    ap.add_argument("--goal-timeout", type=float, default=180.0)
    a = ap.parse_args()

    rclpy.init()
    nav = BasicNavigator("l2s_mvp_mission")
    nav.set_parameters([Parameter("use_sim_time", value=True)])

    name, x, y, yaw = START
    log(f"setting initial pose at {name} ({x}, {y})")
    nav.setInitialPose(pose(nav, x, y, yaw))
    log("waiting for Nav2 (AMCL + bt_navigator) to become active...")
    nav.waitUntilNav2Active(localizer="amcl")
    log("Nav2 active")

    ok = fail = 0
    run = 0
    while a.loop == 0 or run < a.loop:
        run += 1
        for name, x, y, yaw in TOUR:
            log(f"[tour {run}] -> {name} ({x:+.2f}, {y:+.2f})")
            t0 = time.monotonic()
            nav.goToPose(pose(nav, x, y, yaw))
            last = 0.0
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                if fb and time.monotonic() - last > 2.0:
                    last = time.monotonic()
                    log(
                        f"    {fb.distance_remaining:5.2f} m to go, "
                        f"nav time {Duration.from_msg(fb.navigation_time).nanoseconds / 1e9:5.1f}s, "
                        f"recoveries {fb.number_of_recoveries}"
                    )
                    if Duration.from_msg(fb.navigation_time) > Duration(seconds=a.goal_timeout):
                        log("    goal timeout: cancelling")
                        nav.cancelTask()
            res = nav.getResult()
            dt = time.monotonic() - t0
            if res == TaskResult.SUCCEEDED:
                ok += 1
                log(f"    reached {name} in {dt:.1f}s (wall)")
            else:
                fail += 1
                log(f"    FAILED to reach {name}: {res}")

    log(f"done: {ok} goals reached, {fail} failed")
    nav.destroy_node()
    rclpy.shutdown()
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
