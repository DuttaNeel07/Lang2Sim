"""ROS 2 Telemetry Subscribers & Collision Event Monitor.

Asynchronously monitors:
  - /odom (nav_msgs/Odometry): Linear velocity v(t), angular velocity w(t), pose (x, y, yaw).
  - /tf (tf2_msgs/TFMessage): Coordinate transforms (map -> odom -> base_link).
  - /lang2sim/contact_states: Chassis bumper contacts from Gazebo plugins.

Includes:
  - CollisionCounter: Rising-edge filter counting collisions when ||F||_2 > 5.0 N with 0.5s cooldown.
  - TelemetryCollector: Thread-safe ring buffer and run recorder producing Trajectory objects.
  - ReplaySource: Synthetic / recorded replay provider for testing without ROS/Gazebo.
  - query_robot_telemetry(): High-level async inspector consumed by the CLI / TUI sidebar.
"""

from __future__ import annotations

import argparse
import collections
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Deque, Mapping, Optional, Sequence

import numpy as np

from lang2sim.telemetry.trajectory import Trajectory

__all__ = [
    "CollisionCounter",
    "TelemetrySample",
    "TelemetryCollector",
    "ReplaySource",
    "get_telemetry_collector",
    "query_robot_telemetry",
]

FORCE_THRESHOLD_NEWTONS = 5.0
COOLDOWN_SECONDS = 0.5
RING_BUFFER_SIZE = 18000  # ~10 minutes @ 30 Hz


# --------------------------------------------------------------------------- Collision Counter
class CollisionCounter:
    """Rising-edge collision detector filtering out minor ground contact and vibrations."""

    def __init__(
        self,
        threshold_n: float = FORCE_THRESHOLD_NEWTONS,
        cooldown_s: float = COOLDOWN_SECONDS,
        ignored_entities: Optional[Sequence[str]] = None,
    ) -> None:
        self.threshold_n = threshold_n
        self.cooldown_s = cooldown_s
        self.ignored = {
            e.lower() for e in (ignored_entities or ["ground_plane", "floor", "ground", "world"])
        }
        self.collision_count = 0
        self.last_collision_time = 0.0
        self.in_collision = False
        self.last_force_n = 0.0
        self.last_contact_entity = ""

    def process_contact(
        self,
        force_vector: Sequence[float],
        entity_name: str = "",
        timestamp: Optional[float] = None,
    ) -> bool:
        """Process a contact force sample. Returns True if a new structural collision is registered."""
        t_now = time.time() if timestamp is None else timestamp
        mag = float(
            math.hypot(
                force_vector[0], force_vector[1], force_vector[2] if len(force_vector) > 2 else 0.0
            )
        )
        self.last_force_n = mag
        self.last_contact_entity = entity_name

        if any(ign in entity_name.lower() for ign in self.ignored):
            return False

        if mag > self.threshold_n:
            if not self.in_collision and (t_now - self.last_collision_time >= self.cooldown_s):
                self.collision_count += 1
                self.last_collision_time = t_now
                self.in_collision = True
                return True
        else:
            self.in_collision = False
        return False

    def reset(self) -> None:
        self.collision_count = 0
        self.last_collision_time = 0.0
        self.in_collision = False
        self.last_force_n = 0.0
        self.last_contact_entity = ""


# --------------------------------------------------------------------------- Telemetry Sample
@dataclass
class TelemetrySample:
    t: float
    x: float
    y: float
    yaw: float
    v: float
    w: float
    collision_count: int = 0
    last_force_n: float = 0.0
    slam_x: Optional[float] = None
    slam_y: Optional[float] = None
    ate: Optional[float] = None


# --------------------------------------------------------------------------- Telemetry Collector
class TelemetryCollector:
    """Thread-safe telemetry observer and trajectory recorder with live SLAM ATE drift tracking."""

    def __init__(
        self,
        buffer_size: int = RING_BUFFER_SIZE,
        api_base_url: Optional[str] = None,
        poll_interval: float = 0.1,
    ) -> None:
        self.lock = threading.Lock()
        self.ring_buffer: Deque[TelemetrySample] = collections.deque(maxlen=buffer_size)
        self.bumper = CollisionCounter()
        self.active_run_samples: Optional[list[dict[str, float]]] = None
        self.run_label: str = ""
        self.goal: Optional[tuple[float, float]] = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._ros_ok = False
        self._api_ok = False
        self._api_running = False
        self._api_thread: Optional[threading.Thread] = None
        self.rtf: float = 1.0
        self.sim_time: float = 0.0
        self.poll_interval = poll_interval
        self.api_base_url = (
            api_base_url or os.environ.get("LANG2SIM_CONTROL_API") or "http://127.0.0.1:9090"
        )
        # SLAM drift tracking metrics
        self.latest_slam_pose: Optional[tuple[float, float, float]] = None
        self.current_ate: float = 0.0
        self.max_ate: float = 0.0
        self.ate_history: list[float] = []
        self.ate_divergence_threshold: float = 0.35
        self._api_live: Optional[dict[str, Any]] = None  # last TelemetryLive from the control API

        self._init_ros()
        if not self._ros_ok:
            self._start_api_poll()

    def _init_ros(self) -> None:
        try:
            import rclpy  # type: ignore[import-not-found]
            from nav_msgs.msg import Odometry  # type: ignore[import-not-found]

            if not rclpy.ok():
                rclpy.init(args=None)
            node = rclpy.create_node("lang2sim_telemetry_subscriber")

            def _odom_cb(msg: Odometry):
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                # Quaternion -> Yaw
                siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                v = msg.twist.twist.linear.x
                w = msg.twist.twist.angular.z
                self.record_kinematics(time.time(), p.x, p.y, yaw, v, w)

            node.create_subscription(Odometry, "/odom", _odom_cb, 20)

            # Contact states (dynamic type check for gazebo_msgs or ros_gz_interfaces)
            try:
                from gazebo_msgs.msg import ContactsState  # type: ignore[import-not-found]

                def _contact_cb(msg: ContactsState):
                    for state in msg.states:
                        total_force = [0.0, 0.0, 0.0]
                        for f in state.total_wrench:
                            total_force[0] += f.force.x
                            total_force[1] += f.force.y
                            total_force[2] += f.force.z
                        self.bumper.process_contact(total_force, state.collision2_name)

                node.create_subscription(ContactsState, "/lang2sim/contact_states", _contact_cb, 20)
            except ImportError:
                pass

            from rclpy.executors import MultiThreadedExecutor  # type: ignore[import-not-found]

            self._executor = MultiThreadedExecutor()
            self._executor.add_node(node)
            self._thread = threading.Thread(target=self._executor.spin, daemon=True)
            self._thread.start()
            self._ros_ok = True
        except Exception:
            self._ros_ok = False

    def record_kinematics(
        self,
        t: float,
        x: float,
        y: float,
        yaw: float,
        v: float,
        w: float,
        slam_x: Optional[float] = None,
        slam_y: Optional[float] = None,
    ) -> None:
        calc_ate = None
        if slam_x is not None and slam_y is not None:
            calc_ate = math.hypot(x - slam_x, y - slam_y)
            self.current_ate = calc_ate
            if calc_ate > self.max_ate:
                self.max_ate = calc_ate
            self.ate_history.append(calc_ate)

        sample = TelemetrySample(
            t=t,
            x=x,
            y=y,
            yaw=yaw,
            v=v,
            w=w,
            collision_count=self.bumper.collision_count,
            last_force_n=self.bumper.last_force_n,
            slam_x=slam_x,
            slam_y=slam_y,
            ate=calc_ate,
        )
        with self.lock:
            self.ring_buffer.append(sample)
            if self.active_run_samples is not None:
                sample_dict: dict[str, float] = {
                    "t": t,
                    "x": x,
                    "y": y,
                    "yaw": yaw,
                    "v": v,
                    "w": w,
                }
                if slam_x is not None:
                    sample_dict["slam_x"] = slam_x
                if slam_y is not None:
                    sample_dict["slam_y"] = slam_y
                if calc_ate is not None:
                    sample_dict["ate"] = calc_ate
                self.active_run_samples.append(sample_dict)

    def record_slam_pose(self, t: float, x: float, y: float, yaw: float) -> float:
        """Record SLAM estimated pose and update real-time ATE drift tracking.

        ATE = ||p_gt - p_slam||_2
        Returns the instantaneous ATE drift in meters.
        """
        with self.lock:
            self.latest_slam_pose = (x, y, yaw)
            ate = 0.0
            if self.ring_buffer:
                # Find the GT sample closest in timestamp t
                best_gt = min(self.ring_buffer, key=lambda s: abs(s.t - t))
                ate = math.hypot(best_gt.x - x, best_gt.y - y)
                self.current_ate = ate
                if ate > self.max_ate:
                    self.max_ate = ate
                self.ate_history.append(ate)
                best_gt.slam_x = x
                best_gt.slam_y = y
                best_gt.ate = ate
            if self.active_run_samples and len(self.active_run_samples) > 0:
                best_active = min(self.active_run_samples, key=lambda s: abs(s["t"] - t))
                best_active["slam_x"] = x
                best_active["slam_y"] = y
                best_active["ate"] = ate
            return ate

    def start_run(self, label: str = "run", goal: Optional[tuple[float, float]] = None) -> None:
        """Begin recording an evaluation run for NQI scoring."""
        with self.lock:
            self.run_label = label
            self.goal = goal
            self.active_run_samples = []
            self.bumper.reset()
            self.current_ate = 0.0
            self.max_ate = 0.0
            self.ate_history.clear()

    def stop_run(self) -> Trajectory:
        """Stop recording and return captured Trajectory."""
        with self.lock:
            if self.active_run_samples and len(self.active_run_samples) >= 2:
                samples = list(self.active_run_samples)
            elif len(self.ring_buffer) >= 2:
                samples = [
                    {
                        "t": s.t,
                        "x": s.x,
                        "y": s.y,
                        "yaw": s.yaw,
                        "v": s.v,
                        "w": s.w,
                        **({"slam_x": s.slam_x} if s.slam_x is not None else {}),
                        **({"slam_y": s.slam_y} if s.slam_y is not None else {}),
                        **({"ate": s.ate} if s.ate is not None else {}),
                    }
                    for s in self.ring_buffer
                ]
            else:
                t_now = time.time()
                samples = [
                    {"t": t_now - 1.0, "x": 0.0, "y": 0.0, "yaw": 0.0, "v": 0.0, "w": 0.0},
                    {"t": t_now, "x": 0.01, "y": 0.0, "yaw": 0.0, "v": 0.01, "w": 0.0},
                ]
            traj = Trajectory.from_samples(
                samples,
                goal=self.goal,
                collisions=self.bumper.collision_count,
                label=self.run_label,
            )
            self.active_run_samples = None
            return traj

    def _start_api_poll(self) -> None:
        if self._api_thread is not None and self._api_thread.is_alive():
            return
        self._api_running = True
        self._api_thread = threading.Thread(target=self._api_worker, daemon=True)
        self._api_thread.start()

    def _api_worker(self) -> None:
        """Control-API backend for hosts without rclpy (the normal case: ROS lives in the container).

        Polls GET /v1/telemetry/live (falls back to /v1/telemetry on older images). Samples are
        stamped in sim time when the API reports it, and collisions/clearance come from the
        container's bumper and lidar, so snapshot() matches what the recorder scores.
        """
        from lang2sim.telemetry.live import fetch_live

        while self._api_running:
            live = fetch_live(self.api_base_url, timeout=0.8)
            if live is None or live.get("pose") is None:
                self._api_ok = False
            else:
                self._api_ok = True
                self._api_live = live
                self.rtf = float(live.get("real_time_factor") or 0.0)
                self.sim_time = float(live.get("sim_time") or 0.0)
                pose = live["pose"]
                t = self.sim_time if self.sim_time > 0 else time.time()
                with self.lock:
                    last_t = self.ring_buffer[-1].t if self.ring_buffer else None
                if last_t is None or t > last_t:  # sim paused: no duplicate stamps
                    self.record_kinematics(
                        t,
                        float(pose["x"]),
                        float(pose["y"]),
                        float(pose.get("yaw", 0.0)),
                        float(live.get("linear_speed") or 0.0),
                        float(live.get("angular_speed") or 0.0),
                    )
            time.sleep(self.poll_interval)

    def close(self) -> None:
        """Cleanly terminate background polling threads."""
        self._api_running = False
        if self._thread and self._thread.is_alive():
            try:
                if self._executor:
                    self._executor.shutdown()
            except Exception:
                pass

    def snapshot(self) -> dict[str, Any]:
        """Return instantaneous state for the Textual TUI sidebar at 10Hz+."""
        with self.lock:
            if self.ring_buffer:
                latest = self.ring_buffer[-1]
                x, y, yaw, v, w = latest.x, latest.y, latest.yaw, latest.v, latest.w
            else:
                x, y, yaw, v, w = 0.0, 0.0, 0.0, 0.0, 0.0
            backend = "ros2" if self._ros_ok else ("control_api" if self._api_ok else "mock")
            mean_ate = (sum(self.ate_history) / len(self.ate_history)) if self.ate_history else 0.0
            live = self._api_live if (self._api_ok and not self._ros_ok) else None
            collisions = (
                int(live.get("collision_count") or 0) if live else self.bumper.collision_count
            )
            force = (
                float(live.get("last_contact_force_n") or 0.0) if live else self.bumper.last_force_n
            )
            entity = (
                str(live.get("last_contact_entity") or "")
                if live
                else self.bumper.last_contact_entity
            )
            return {
                "active": self._ros_ok or self._api_ok,
                "backend": backend,
                "x": round(x, 2),
                "y": round(y, 2),
                "yaw": round(yaw, 3),
                "v": round(v, 2),
                "w": round(w, 2),
                "rtf": round(self.rtf, 2),
                "sim_time": round(self.sim_time, 2),
                "collisions": collisions,
                "last_force_n": round(force, 1),
                "last_contact_entity": entity,
                "pose_source": live.get("pose_source") if live else backend,
                "obstacle_clearance_m": live.get("obstacle_clearance_m") if live else None,
                "distance_to_goal_m": live.get("distance_to_goal_m") if live else None,
                "samples_buffered": len(self.ring_buffer),
                "ate": round(self.current_ate, 3),
                "max_ate": round(self.max_ate, 3),
                "mean_ate": round(mean_ate, 3),
                "ate_diverged": bool(self.current_ate > self.ate_divergence_threshold),
            }

    get_snapshot = snapshot


# --------------------------------------------------------------------------- Replay Source
class ReplaySource:
    """Mock telemetry provider replaying synthetic trajectories or recorded JSON files."""

    def __init__(self, trajectory: Optional[Trajectory] = None) -> None:
        self.trajectory = trajectory
        self.index = 0
        self.collisions = 0
        self.t_start = time.time()

    @classmethod
    def from_json(cls, json_path: str | Path) -> "ReplaySource":
        traj = Trajectory.from_json(json_path)
        return cls(traj)

    def snapshot(self) -> dict[str, Any]:
        if self.trajectory is None or self.trajectory.n == 0:
            return {
                "active": False,
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "v": 0.0,
                "w": 0.0,
                "collisions": 0,
            }
        elapsed = time.time() - self.t_start
        # Find closest sample
        dt_arr = np.abs((self.trajectory.t - self.trajectory.t[0]) - elapsed)
        idx = int(np.argmin(dt_arr))
        return {
            "active": True,
            "x": round(float(self.trajectory.x[idx]), 2),
            "y": round(float(self.trajectory.y[idx]), 2),
            "yaw": round(float(self.trajectory.yaw[idx]), 3),
            "v": round(float(self.trajectory.v[idx]), 2),
            "w": round(float(self.trajectory.w[idx]), 2),
            "collisions": self.trajectory.collisions,
            "last_force_n": 0.0,
            "samples_buffered": self.trajectory.n,
        }


_GLOBAL_COLLECTOR: Optional[TelemetryCollector] = None


def get_telemetry_collector() -> TelemetryCollector:
    global _GLOBAL_COLLECTOR
    if _GLOBAL_COLLECTOR is None:
        _GLOBAL_COLLECTOR = TelemetryCollector()
    return _GLOBAL_COLLECTOR


async def query_robot_telemetry() -> dict[str, Any]:
    """Asynchronous query function returning live robot kinematics and collision status."""
    return get_telemetry_collector().snapshot()


# --------------------------------------------------------------------------- CLI
def main() -> int:
    parser = argparse.ArgumentParser(
        description="Lang2Sim Telemetry Subscriber & Collision Monitor"
    )
    parser.add_argument(
        "--live", action="store_true", help="Monitor live /odom and /lang2sim/contact_states"
    )
    parser.add_argument("--print-hz", type=float, default=2.0, help="Print rate in Hz")
    parser.add_argument("--record", help="Record live run to JSON file on exit")
    parser.add_argument(
        "--goal", nargs=2, type=float, metavar=("X", "Y"), help="Mission goal for recorded run"
    )
    args = parser.parse_args()

    collector = get_telemetry_collector()
    goal = tuple(args.goal) if args.goal else None
    if args.record:
        collector.start_run(label="recorded_run", goal=goal)
        print(f"Recording telemetry run to '{args.record}'... Press Ctrl+C to stop.")

    dt = 1.0 / max(0.1, args.print_hz)
    try:
        while True:
            snap = collector.snapshot()
            status = "ROS ACTIVE" if snap["active"] else "STANDBY / REPLAY"
            sys.stdout.write(
                f"\r[{status}] Pose: ({snap['x']:6.2f}, {snap['y']:6.2f}) | "
                f"Yaw: {snap['yaw']:5.2f} | Vel: {snap['v']:4.2f} m/s | "
                f"Collisions: {snap['collisions']:2d} | Force: {snap['last_force_n']:4.1f} N"
            )
            sys.stdout.flush()
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nStopping telemetry monitor.")
        if args.record:
            traj = collector.stop_run()
            traj.to_json(args.record)
            print(f"Saved {traj.n} samples to {args.record}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
