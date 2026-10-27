"""Mission-level resilience: a leg Nav2 gives up on is retried after clearing costmaps and backing up."""

from __future__ import annotations

import time
from types import SimpleNamespace

from lang2sim.core.models import Pose2D
from lang2sim.runtime import nav2_dispatcher as nd


class ScriptedNav:
    """goToPose results come from `results` (one per call); backup/clear calls are recorded."""

    def __init__(self, results):
        self.results, self.goals, self.calls = list(results), [], []
        self._result = "SUCCEEDED"

    def goToPose(self, p):
        self.goals.append(p)
        self._result = self.results.pop(0) if self.results else "SUCCEEDED"
        return True

    def isTaskComplete(self):
        return True

    def getFeedback(self):
        return SimpleNamespace(distance_remaining=0.0, navigation_time=1.0, estimated_time_remaining=0.0,
                               number_of_recoveries=3)

    def getResult(self):
        return SimpleNamespace(name=self._result)

    def cancelTask(self):
        self.calls.append("cancel")

    def clearAllCostmaps(self):
        self.calls.append("clear")

    def backup(self, backup_dist, backup_speed, time_allowance):
        self.calls.append(("backup", backup_dist))
        return True


def run_mission(nav, poses):
    d = nd.NavDispatcher(navigator_factory=lambda: nav, make_pose=lambda _n, p: p)
    try:
        d.follow(poses, timeout_s=60)
        for _ in range(200):
            st = d.status()
            if not st.is_active and st.status != "navigating":
                return st
            time.sleep(0.02)
        raise AssertionError("mission did not finish")
    finally:
        d.shutdown()


WPS = [Pose2D(x=1, y=0), Pose2D(x=2, y=0), Pose2D(x=3, y=0)]


def test_a_failed_leg_is_retried_and_the_mission_continues():
    nav = ScriptedNav(["SUCCEEDED", "FAILED", "SUCCEEDED", "SUCCEEDED"])
    st = run_mission(nav, WPS)
    assert st.status == "succeeded"
    assert [g.x for g in nav.goals] == [1, 2, 2, 3]                  # leg 2 driven twice, then leg 3
    assert nav.calls == ["clear", ("backup", nd.BACKUP_M), "clear"]


def test_gives_up_after_the_retries():
    nav = ScriptedNav(["FAILED"] * 5)
    st = run_mission(nav, WPS)
    assert st.status == "failed" and "Nav2 result FAILED" in st.message
    assert len(nav.goals) == 1 + nd.LEG_RETRIES


def test_rejections_and_timeouts_are_not_retried():
    assert not nd.NavDispatcher._retryable("goal rejected by Nav2 (bt_navigator not active?)")
    assert not nd.NavDispatcher._retryable("timed out after 120 s (sim time)")
    assert nd.NavDispatcher._retryable("Nav2 result FAILED")


def test_navigators_without_backup_still_retry():
    nav = ScriptedNav(["FAILED", "SUCCEEDED"])
    nav.backup = None                                                   # older nav2_simple_commander
    st = run_mission(nav, WPS[:1])
    assert st.status == "succeeded" and nav.calls == ["clear"]
