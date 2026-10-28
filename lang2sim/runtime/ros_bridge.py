"""
The control API's window into ROS: readiness, telemetry, RTF.
File: lang2sim/runtime/ros_bridge.py

Runs inside the container, in the API server process, as one rclpy node on its
own executor thread. It only listens (/clock, /odom, /scan, TF, Nav2 lifecycle
states, and the telemetry topics /lang2sim/ground_truth/odom and
/lang2sim/contact_states); commands go through nav2_dispatcher.

It also feeds the Telemetry Engine: every pose update goes to a
telemetry_recorder.TelemetryRecorder (sim-time stamps, world-frame poses) and
every contact message to its ContactMonitor, so the control API can serve live
telemetry and recorded trajectories for NQI scoring.

Pure helpers (yaw_from_quaternion, rtf_from_samples, readiness) carry the logic
and are unit-tested on the host; the RosBridge class needs rclpy and is created
with RosBridge.try_create(), which returns None where ROS is not installed.
"""

from __future__ import annotations

import collections
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

from lang2sim.runtime import robot_instrument
from lang2sim.runtime.telemetry_recorder import (
    ContactMonitor,
    TelemetryRecorder,
    best_pose,
    build_live,
    min_valid_range,
)

FRESH_S = 3.0  # a topic counts as alive if a message arrived this recently (wall time)
RTF_WINDOW_S = 5.0
NAV2_NODES = ("amcl", "bt_navigator", "controller_server", "planner_server")
GROUND_TRUTH_STALE_S = (
    1.0  # fall back to map/odom poses when ground truth stops arriving (wall time)
)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rtf_from_samples(samples: Sequence[Tuple[float, float]]) -> Optional[float]:
    """(wall_s, sim_s) pairs, oldest first -> real-time factor over the span, or None."""
    if len(samples) < 2:
        return None
    (w0, s0), (w1, s1) = samples[0], samples[-1]
    if w1 - w0 <= 0.2:
        return None
    return max(0.0, (s1 - s0) / (w1 - w0))


@dataclass
class RosSnapshot:
    now: float = 0.0  # wall time of the snapshot
    clock_wall: Optional[float] = None  # wall time of the last /clock
    sim_time: float = 0.0
    odom_wall: Optional[float] = None
    scan_wall: Optional[float] = None
    odom_pose: Optional[Tuple[float, float, float]] = None  # x, y, yaw in odom
    map_pose: Optional[Tuple[float, float, float]] = None  # x, y, yaw in map (TF)
    velocity: Optional[Tuple[float, float, float]] = None  # vx, vy, wz
    lifecycle: Dict[str, str] = field(default_factory=dict)  # node -> "active", ...
    rtf: Optional[float] = None
    ground_truth: Optional[Tuple[float, float, float]] = None  # x, y, yaw in world (Gazebo)
    ground_truth_wall: Optional[float] = None
    scan_min: Optional[float] = None  # closest valid lidar return, m
    spawn: Optional[Tuple[float, float, float]] = None  # odom frame origin in world

    def fresh(self, t: Optional[float]) -> bool:
        return t is not None and self.now - t <= FRESH_S


def stamp_s(stamp: Any) -> float:
    """builtin_interfaces/Time -> seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def contact_pairs(
    entries: Sequence[Tuple[str, str, Optional[float]]], robot: str
) -> List[Tuple[str, Optional[float]]]:
    """(collision1, collision2, force) -> [(the entity that is not the robot, force)].

    Names are Gazebo-scoped ("waffle::base_link::base_collision"); a contact whose
    sides are both the robot (self-contact) is dropped. force None = no wrench data.
    """
    out: List[Tuple[str, Optional[float]]] = []
    for a, b, force in entries:
        a_robot = a.split("::")[0] == robot
        b_robot = b.split("::")[0] == robot
        if a_robot and b_robot:
            continue
        out.append((b if a_robot else a, force))
    return out


def readiness(s: RosSnapshot) -> Dict[str, bool]:
    """What 'the simulation is ready to navigate' means, check by check."""
    return {
        "gazebo_clock": s.fresh(s.clock_wall),
        "odometry": s.fresh(s.odom_wall),
        "laser_scan": s.fresh(s.scan_wall),
        "nav2_active": all(s.lifecycle.get(n) == "active" for n in NAV2_NODES),
        "localized": s.map_pose is not None,
    }


class RosBridge:
    """rclpy node + executor thread. Create with try_create()."""

    def __init__(self, robot_name: str = "waffle") -> None:
        import rclpy  # type: ignore[import-not-found]
        from lifecycle_msgs.srv import GetState  # type: ignore[import-not-found]
        from nav_msgs.msg import Odometry  # type: ignore[import-not-found]
        from rclpy.executors import MultiThreadedExecutor  # type: ignore[import-not-found]
        from rclpy.node import Node  # type: ignore[import-not-found]
        from rclpy.parameter import Parameter  # type: ignore[import-not-found]
        from rclpy.qos import qos_profile_sensor_data  # type: ignore[import-not-found]
        from rosgraph_msgs.msg import Clock  # type: ignore[import-not-found]
        from sensor_msgs.msg import LaserScan  # type: ignore[import-not-found]
        from tf2_ros import Buffer, TransformListener  # type: ignore[import-not-found]

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._lock = threading.Lock()
        self._s = RosSnapshot()
        self._clock_samples: Deque[Tuple[float, float]] = collections.deque(maxlen=400)
        self.robot_name = robot_name
        self.recorder = TelemetryRecorder(ContactMonitor(robot_name=robot_name))
        self.expect_ground_truth = False  # set per run: the robot was instrumented
        self._record_wall0 = 0.0

        node = Node("l2s_control_api", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.node = node
        node.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        node.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        node.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        node.create_subscription(
            Odometry,
            robot_instrument.GROUND_TRUTH_TOPIC,
            self._on_ground_truth,
            qos_profile_sensor_data,
        )
        try:  # ros_gz_interfaces ships with ros_gz_bridge; tolerate its absence
            from ros_gz_interfaces.msg import Contacts  # type: ignore[import-not-found]

            node.create_subscription(
                Contacts,
                robot_instrument.CONTACT_ROS_TOPIC,
                self._on_contacts,
                qos_profile_sensor_data,
            )
        except ImportError:
            pass
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, node, spin_thread=False)
        self._GetState = GetState
        self._lc_clients = {n: node.create_client(GetState, f"/{n}/get_state") for n in NAV2_NODES}
        node.create_timer(1.0, self._poll)

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._executor.spin, name="l2s-ros", daemon=True)
        self._thread.start()

    @classmethod
    def try_create(cls) -> Optional["RosBridge"]:
        try:
            return cls()
        except Exception:  # noqa: BLE001 - no ROS here (host tests) or rclpy failed to init
            return None

    # --- callbacks (executor threads) -------------------------------------
    def _on_clock(self, msg) -> None:
        now = time.time()
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        with self._lock:
            self._s.clock_wall, self._s.sim_time = now, sim
            self._clock_samples.append((now, sim))
            while self._clock_samples and now - self._clock_samples[0][0] > RTF_WINDOW_S:
                self._clock_samples.popleft()

    def _on_odom(self, msg) -> None:
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        now = time.time()
        with self._lock:
            self._s.odom_wall = now
            self._s.odom_pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
            self._s.velocity = (t.linear.x, t.linear.y, t.angular.z)
            last_gt = self._s.ground_truth_wall
        if self._odom_drives_recording(now, last_gt):
            self._record(stamp_s(msg.header.stamp))

    def _odom_drives_recording(self, now: float, last_gt: Optional[float]) -> bool:
        """Record from /odom (map/odom poses) only when ground truth is not coming.

        Stock robot: always. Instrumented robot: only after ground truth has been
        silent for GROUND_TRUTH_STALE_S (counted from the recording start if it never
        arrived), so one run never mixes pose sources at startup.
        """
        if not self.expect_ground_truth:
            return last_gt is None or now - last_gt > GROUND_TRUTH_STALE_S
        ref = last_gt if last_gt is not None else self._record_wall0
        return now - ref > GROUND_TRUTH_STALE_S

    def _on_ground_truth(self, msg) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        with self._lock:
            self._s.ground_truth_wall = time.time()
            self._s.ground_truth = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        self._record(stamp_s(msg.header.stamp))

    def _on_scan(self, msg) -> None:
        m = min_valid_range(msg.ranges, msg.range_min, msg.range_max)
        with self._lock:
            self._s.scan_wall = time.time()
            self._s.scan_min = m

    def _on_contacts(self, msg) -> None:
        entries = []
        for c in msg.contacts:
            forces = []
            for w in c.wrenches:
                f = w.body_1_wrench.force
                forces.append(math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z))
            force = max(forces) if forces and max(forces) > 0.0 else None
            entries.append((c.collision1.name, c.collision2.name, force))
        t = stamp_s(msg.header.stamp)
        with self._lock:
            t = t if t > 0 else self._s.sim_time
        self.recorder.contacts.process(contact_pairs(entries, self.robot_name), t)

    def _map_pose_now(self) -> Optional[Tuple[float, float, float]]:
        try:
            from rclpy.time import Time

            tr = self._tf.lookup_transform("map", "base_footprint", Time())
        except Exception:  # noqa: BLE001 - not localized yet
            return None
        tt, q = tr.transform.translation, tr.transform.rotation
        return (tt.x, tt.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _record(self, t: float) -> None:
        """Offer the current state to the recorder (only does work while recording)."""
        self.recorder.contacts.tick(t)
        if not self.recorder.recording:
            return
        map_pose = self._map_pose_now()
        with self._lock:
            gt, odom, spawn = self._s.ground_truth, self._s.odom_pose, self._s.spawn
            vel, scan_min = self._s.velocity, self._s.scan_min
            gt_fresh = (
                self._s.ground_truth_wall is not None
                and time.time() - self._s.ground_truth_wall <= GROUND_TRUTH_STALE_S
            )
        pose, source = best_pose(gt if gt_fresh else None, map_pose, odom, spawn)
        v, w = (vel[0], vel[2]) if vel else (0.0, 0.0)
        self.recorder.offer(t, pose, source, v, w, estimate=map_pose, scan_min=scan_min)

    def _poll(self) -> None:
        # Map pose from TF (AMCL map->odom + odom->base_footprint)
        pose = self._map_pose_now()
        with self._lock:
            self._s.map_pose = pose
        # Nav2 lifecycle states, asynchronously
        for name, client in self._lc_clients.items():
            if not client.service_is_ready():
                with self._lock:
                    self._s.lifecycle.pop(name, None)
                continue
            fut = client.call_async(self._GetState.Request())
            fut.add_done_callback(lambda f, n=name: self._on_state(n, f))

    def _on_state(self, name: str, fut) -> None:
        try:
            label = fut.result().current_state.label
        except Exception:  # noqa: BLE001
            return
        with self._lock:
            self._s.lifecycle[name] = label

    # --- API ---------------------------------------------------------------
    def snapshot(self) -> RosSnapshot:
        with self._lock:
            s = RosSnapshot(
                **{k: (dict(v) if isinstance(v, dict) else v) for k, v in vars(self._s).items()}
            )
            s.rtf = rtf_from_samples(list(self._clock_samples))
        s.now = time.time()
        return s

    def reset(
        self, spawn: Optional[Tuple[float, float, float]] = None, expect_ground_truth: bool = False
    ) -> None:
        """Forget state from a previous run (called on /sim/start with the run's spawn pose and
        whether the robot was instrumented with a ground-truth publisher)."""
        self.expect_ground_truth = expect_ground_truth
        with self._lock:
            self._s = RosSnapshot(spawn=spawn)
            self._clock_samples.clear()
        self.recorder.stop()
        self.recorder.contacts.reset()

    # --- telemetry -------------------------------------------------------------
    def record_start(
        self,
        label: str,
        goal: Optional[Tuple[float, float, float]] = None,
        rate_hz: float = 20.0,
        stream_path: Any = None,
    ) -> None:
        with self._lock:
            spawn = self._s.spawn
        self._record_wall0 = time.time()
        self.recorder.start(label, goal=goal, rate_hz=rate_hz, spawn=spawn, stream_path=stream_path)

    def record_stop(self, keep_partial: bool = False) -> Optional[Dict[str, Any]]:
        return self.recorder.stop(keep_partial=keep_partial)

    def live(self, goal: Optional[Tuple[float, float, float]] = None) -> Dict[str, Any]:
        """TelemetryLive fields (core.models) for the current state."""
        s = self.snapshot()
        gt = s.ground_truth if s.fresh(s.ground_truth_wall) else None
        pose, source = best_pose(gt, s.map_pose, s.odom_pose, s.spawn)
        return build_live(
            sim_time=s.sim_time,
            rtf=s.rtf,
            pose=pose,
            source=source,
            velocity=s.velocity,
            ground_truth=gt,
            map_pose=s.map_pose,
            scan_min=s.scan_min,
            recorder=self.recorder,
            goal=goal,
        )

    def stop(self) -> None:
        try:
            self._executor.shutdown(timeout_sec=2)
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
