"""
The control API's window into ROS: readiness, telemetry, RTF.
File: lang2sim/runtime/ros_bridge.py

Runs inside the container, in the API server process, as one rclpy node on its
own executor thread. It only listens (/clock, /odom, /scan, TF, Nav2 lifecycle
states); commands go through nav2_dispatcher.

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
from typing import Deque, Dict, Optional, Sequence, Tuple

FRESH_S = 3.0          # a topic counts as alive if a message arrived this recently (wall time)
RTF_WINDOW_S = 5.0
NAV2_NODES = ("amcl", "bt_navigator", "controller_server", "planner_server")


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
    now: float = 0.0                      # wall time of the snapshot
    clock_wall: Optional[float] = None    # wall time of the last /clock
    sim_time: float = 0.0
    odom_wall: Optional[float] = None
    scan_wall: Optional[float] = None
    odom_pose: Optional[Tuple[float, float, float]] = None   # x, y, yaw in odom
    map_pose: Optional[Tuple[float, float, float]] = None    # x, y, yaw in map (TF)
    velocity: Optional[Tuple[float, float, float]] = None    # vx, vy, wz
    lifecycle: Dict[str, str] = field(default_factory=dict)  # node -> "active", ...
    rtf: Optional[float] = None

    def fresh(self, t: Optional[float]) -> bool:
        return t is not None and self.now - t <= FRESH_S


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

    def __init__(self) -> None:
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

        node = Node("l2s_control_api", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.node = node
        node.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        node.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)
        node.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
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
        with self._lock:
            self._s.odom_wall = time.time()
            self._s.odom_pose = (p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
            self._s.velocity = (t.linear.x, t.linear.y, t.angular.z)

    def _on_scan(self, _msg) -> None:
        with self._lock:
            self._s.scan_wall = time.time()

    def _poll(self) -> None:
        # Map pose from TF (AMCL map->odom + odom->base_footprint)
        try:
            from rclpy.time import Time
            tr = self._tf.lookup_transform("map", "base_footprint", Time())
            tt, q = tr.transform.translation, tr.transform.rotation
            pose = (tt.x, tt.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        except Exception:  # noqa: BLE001 - not localized yet
            pose = None
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
            s = RosSnapshot(**{k: (dict(v) if isinstance(v, dict) else v) for k, v in vars(self._s).items()})
            s.rtf = rtf_from_samples(list(self._clock_samples))
        s.now = time.time()
        return s

    def reset(self) -> None:
        """Forget state from a previous run (called on /sim/start)."""
        with self._lock:
            self._s = RosSnapshot()
            self._clock_samples.clear()

    def stop(self) -> None:
        try:
            self._executor.shutdown(timeout_sec=2)
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
