"""
Nav2 dispatcher: initial pose, goals, waypoint missions, cancel, live status.
File: sim2lang/runtime/nav2_dispatcher.py

Runs inside the container, in the API server. One worker thread owns one
nav2_simple_commander.BasicNavigator (it spins itself, so it must never be
touched from another thread); API handlers only enqueue commands and read a
lock-protected NavStatus.

Waypoint missions are driven leg by leg with goToPose, so every leg reports
distance remaining, ETA and recoveries. A new goal preempts the current task.

BasicNavigator.waitUntilNav2Active() is deliberately not used: until AMCL
answers, it republishes its default (0, 0) pose, which would teleport the
localization estimate. The server only dispatches once readiness says Nav2 is
active and the robot is localized.

The navigator is injected via `navigator_factory` / `make_pose` so the logic is
unit-tested on the host without ROS.
"""

from __future__ import annotations

import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, List, Optional, Sequence

from sim2lang.core.models import NavStatus, Pose2D

POLL_S = 0.1


def _ros_navigator() -> Any:
    from nav2_simple_commander.robot_navigator import BasicNavigator
    from rclpy.parameter import Parameter

    nav = BasicNavigator("l2s_nav_dispatcher")
    nav.set_parameters([Parameter("use_sim_time", value=True)])
    return nav


def _ros_pose(nav: Any, p: Pose2D) -> Any:
    from geometry_msgs.msg import PoseStamped

    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.header.stamp = nav.get_clock().now().to_msg()
    ps.pose.position.x, ps.pose.position.y = p.x, p.y
    ps.pose.orientation.z, ps.pose.orientation.w = math.sin(p.yaw / 2), math.cos(p.yaw / 2)
    return ps


def _seconds(duration: Any) -> float:
    """builtin_interfaces/Duration (or a float in tests) -> seconds."""
    if duration is None:
        return 0.0
    if isinstance(duration, (int, float)):
        return float(duration)
    return duration.sec + duration.nanosec * 1e-9


@dataclass
class _Task:
    kind: str                      # "goal" | "waypoints" | "initial_pose"
    poses: List[Pose2D]
    timeout_s: float = 120.0


class NavDispatcher:
    def __init__(self, navigator_factory: Callable[[], Any] = _ros_navigator,
                 make_pose: Callable[[Any, Pose2D], Any] = _ros_pose) -> None:
        self._factory = navigator_factory
        self._make_pose = make_pose
        self._lock = threading.Lock()
        self._status = NavStatus()
        self._queue: "queue.Queue[_Task]" = queue.Queue()
        self._cancel = threading.Event()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._nav: Any = None

    # --- API (any thread) ---------------------------------------------------
    def status(self) -> NavStatus:
        with self._lock:
            return self._status.model_copy()

    def set_initial_pose(self, pose: Pose2D) -> None:
        self._submit(_Task("initial_pose", [pose]))

    def go_to(self, goal: Pose2D, timeout_s: float = 120.0) -> NavStatus:
        return self._submit(_Task("goal", [goal], timeout_s))

    def follow(self, waypoints: Sequence[Pose2D], timeout_s: float = 180.0) -> NavStatus:
        if not waypoints:
            raise ValueError("at least one waypoint is required")
        return self._submit(_Task("waypoints", list(waypoints), timeout_s))

    def cancel(self) -> NavStatus:
        self._drain()
        if self.status().is_active:
            self._cancel.set()
        return self.status()

    def shutdown(self) -> None:
        self._stop.set()
        self._cancel.set()
        if self._thread:
            self._thread.join(timeout=5)

    # --- internals -------------------------------------------------------------
    def _submit(self, task: _Task) -> NavStatus:
        self._ensure_worker()
        if task.kind != "initial_pose":
            self._drain()                          # newest command wins
            if self.status().is_active:
                self._cancel.set()                 # preempt the running task
            with self._lock:                       # report the new task immediately
                self._status = NavStatus(is_active=True, status="navigating", current_task=task.kind,
                                         target_pose=task.poses[0], waypoints_total=len(task.poses))
        self._queue.put(task)
        return self.status()

    def _drain(self) -> None:
        try:
            while True:
                self._queue.get_nowait()
        except queue.Empty:
            pass

    def _ensure_worker(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._worker, name="l2s-nav", daemon=True)
        self._thread.start()

    def _set(self, **kw: Any) -> None:
        with self._lock:
            self._status = self._status.model_copy(update=kw)

    def _worker(self) -> None:
        self._nav = self._factory()
        while not self._stop.is_set():
            try:
                task = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if task.kind == "initial_pose":
                self._nav.setInitialPose(self._make_pose(self._nav, task.poses[0]))
                continue
            self._cancel.clear()
            self._run(task)

    def _run(self, task: _Task) -> None:
        total = len(task.poses)
        for i, pose in enumerate(task.poses):
            self._set(is_active=True, status="navigating", current_task=task.kind, target_pose=pose,
                      waypoint_index=i, waypoints_total=total, distance_remaining=0.0,
                      navigation_time=0.0, eta_seconds=None, recoveries=0, message=None)
            outcome, msg = self._leg(pose, task.timeout_s)
            if outcome != "succeeded":
                if outcome == "canceled" and not self._queue.empty():
                    return  # preempted: the next task's status is already showing
                self._set(is_active=False, status=outcome, message=msg)
                return
        self._set(is_active=False, status="succeeded", distance_remaining=0.0, eta_seconds=0.0,
                  message=None)

    def _leg(self, pose: Pose2D, timeout_s: float):
        nav = self._nav
        if not nav.goToPose(self._make_pose(nav, pose)):
            return "failed", "goal rejected by Nav2 (bt_navigator not active?)"
        timed_out = False
        while not nav.isTaskComplete():
            fb = nav.getFeedback()
            if fb is not None:
                t = _seconds(getattr(fb, "navigation_time", None))
                self._set(distance_remaining=round(float(getattr(fb, "distance_remaining", 0.0)), 3),
                          navigation_time=round(t, 2),
                          eta_seconds=round(_seconds(getattr(fb, "estimated_time_remaining", None)), 1),
                          recoveries=int(getattr(fb, "number_of_recoveries", 0)))
                if t > timeout_s and not timed_out:
                    timed_out = True
                    nav.cancelTask()
            if self._cancel.is_set():
                nav.cancelTask()
                self._cancel.clear()
                return "canceled", "canceled by request"
            time.sleep(POLL_S)
        name = getattr(nav.getResult(), "name", str(nav.getResult())).upper()
        if timed_out:
            return "failed", f"timed out after {timeout_s:.0f} s (sim time)"
        if "SUCCEEDED" in name:
            return "succeeded", None
        if "CANCELED" in name:
            return "canceled", "canceled by Nav2"
        return "failed", f"Nav2 result {name}"
