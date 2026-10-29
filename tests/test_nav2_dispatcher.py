"""
Nav2 dispatcher logic against a fake BasicNavigator (no ROS).
File: tests/test_nav2_dispatcher.py
"""

import threading
import time
from types import SimpleNamespace

import pytest

from lang2sim.core.models import Pose2D
from lang2sim.runtime.nav2_dispatcher import NavDispatcher


class FakeNav:
    """Each leg takes `steps` polls; distance shrinks; result SUCCEEDED unless told otherwise."""

    def __init__(self, steps=5, result="SUCCEEDED", accept=True, nav_time_per_step=1.0):
        self.steps, self.result, self.accept, self.dt = steps, result, accept, nav_time_per_step
        self.goals, self.initial_poses, self.cancels = [], [], 0
        self._left = 0
        self._canceled = False
        self.lock = threading.Lock()

    def goToPose(self, pose):
        with self.lock:
            self.goals.append(pose)
            self._left, self._canceled, self._t = self.steps, False, 0.0
        return self.accept

    def isTaskComplete(self):
        with self.lock:
            if self._canceled or self._left <= 0:
                return True
            self._left -= 1
            self._t += self.dt
            return False

    def getFeedback(self):
        return SimpleNamespace(
            distance_remaining=float(self._left),
            navigation_time=self._t,
            estimated_time_remaining=float(self._left),
            number_of_recoveries=0,
        )

    def getResult(self):
        return SimpleNamespace(name="CANCELED" if self._canceled else self.result)

    def cancelTask(self):
        with self.lock:
            self.cancels += 1
            self._canceled = True

    def setInitialPose(self, pose):
        self.initial_poses.append(pose)


def make(nav):
    return NavDispatcher(navigator_factory=lambda: nav, make_pose=lambda _n, p: p)


def wait_for(d, pred, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        st = d.status()
        if pred(st):
            return st
        time.sleep(0.02)
    raise AssertionError(f"timed out; last status {d.status()}")


def test_single_goal_succeeds_with_progress():
    nav = FakeNav(steps=10)
    d = make(nav)
    st = d.go_to(Pose2D(x=1, y=2))
    assert st.is_active and st.status == "navigating" and st.waypoints_total == 1
    mid = wait_for(d, lambda s: 0 < s.distance_remaining < 10)
    assert mid.target_pose == Pose2D(x=1, y=2) and mid.eta_seconds is not None
    done = wait_for(d, lambda s: not s.is_active)
    assert done.status == "succeeded" and done.message is None
    assert nav.goals == [Pose2D(x=1, y=2)]
    d.shutdown()


def test_waypoints_run_in_order():
    nav = FakeNav(steps=2)
    d = make(nav)
    wps = [Pose2D(x=1, y=0), Pose2D(x=2, y=0), Pose2D(x=3, y=0)]
    d.follow(wps)
    seen = set()
    t0 = time.time()
    while time.time() - t0 < 5:
        s = d.status()
        if s.is_active:
            seen.add(s.waypoint_index)
        else:
            break
        time.sleep(0.005)
    assert d.status().status == "succeeded" and nav.goals == wps
    assert seen <= {0, 1, 2} and d.status().waypoints_total == 3
    with pytest.raises(ValueError):
        d.follow([])
    d.shutdown()


def test_cancel_and_preempt():
    nav = FakeNav(steps=10_000)
    d = make(nav)
    d.go_to(Pose2D(x=5, y=5))
    wait_for(d, lambda s: s.distance_remaining > 0)
    st = d.cancel()
    done = wait_for(d, lambda s: not s.is_active)
    assert done.status == "canceled" and nav.cancels == 1
    # preempt: a new goal while one runs replaces it
    d.go_to(Pose2D(x=1, y=1))
    wait_for(d, lambda s: s.distance_remaining > 0)
    d.go_to(Pose2D(x=2, y=2))
    st = wait_for(d, lambda s: s.target_pose == Pose2D(x=2, y=2) and s.distance_remaining > 0)
    assert st.status == "navigating"
    d.cancel()
    wait_for(d, lambda s: not s.is_active)
    d.shutdown()


def test_timeout_rejection_and_failure():
    nav = FakeNav(steps=10_000, nav_time_per_step=1.0)
    d = make(nav)
    d.go_to(Pose2D(x=1, y=1), timeout_s=5)
    st = wait_for(d, lambda s: not s.is_active)
    assert st.status == "failed" and "timed out" in st.message
    d.shutdown()

    d = make(FakeNav(accept=False))
    d.go_to(Pose2D(x=1, y=1))
    assert "rejected" in wait_for(d, lambda s: not s.is_active).message
    d.shutdown()

    d = make(FakeNav(steps=2, result="FAILED"))
    d.follow([Pose2D(x=1, y=1), Pose2D(x=2, y=2)])
    st = wait_for(d, lambda s: not s.is_active)
    assert st.status == "failed" and st.waypoint_index == 0
    d.shutdown()


def test_initial_pose_is_forwarded():
    nav = FakeNav()
    d = make(nav)
    d.set_initial_pose(Pose2D(x=-2, y=-0.5, yaw=0.3))
    t0 = time.time()
    while not nav.initial_poses and time.time() - t0 < 5:
        time.sleep(0.02)
    assert nav.initial_poses == [Pose2D(x=-2, y=-0.5, yaw=0.3)]
    assert d.status().status == "idle"
    d.shutdown()


def test_resume_with_keeps_the_rest_of_the_mission():
    nav = FakeNav(steps=10_000)
    d = make(nav)
    wps = [Pose2D(x=1, y=0), Pose2D(x=2, y=0), Pose2D(x=3, y=0)]
    d.follow(wps)
    wait_for(d, lambda s: s.distance_remaining > 0)
    assert d.remaining() == wps
    d.resume_with(Pose2D(x=1.2, y=0.3))
    st = wait_for(d, lambda s: s.target_pose == Pose2D(x=1.2, y=0.3) and s.distance_remaining > 0)
    assert st.waypoints_total == 3 and d.remaining()[1:] == wps[1:]
    d.cancel()
    wait_for(d, lambda s: not s.is_active)
    d.shutdown()
