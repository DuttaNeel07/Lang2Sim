"""
Health monitor states and reasons (no ROS).
File: tests/test_health_monitor.py
"""

from lang2sim.runtime.health_monitor import HealthMonitor
from lang2sim.runtime.ros_bridge import NAV2_NODES, RosSnapshot


def snap(now=100.0, rtf=1.0, **kw):
    base = dict(now=now, clock_wall=now, odom_wall=now, scan_wall=now, sim_time=50.0, rtf=rtf,
                map_pose=(0.0, 0.0, 0.0), lifecycle={n: "active" for n in NAV2_NODES})
    base.update(kw)
    return RosSnapshot(**base)


def test_basic_states():
    m = HealthMonitor()
    assert m.evaluate("error", snap()).health == "crashed"
    assert m.evaluate("idle", None).health == "healthy"
    assert m.evaluate("launching", snap(scan_wall=None)).health == "healthy"   # still starting
    r = m.evaluate("running", snap())
    assert r.health == "healthy" and not r.issues and r.rtf == 1.0


def test_lost_checks_and_stall():
    m = HealthMonitor()
    r = m.evaluate("degraded", snap(scan_wall=90.0, clock_wall=90.0))
    assert r.health == "degraded"
    assert "laser_scan lost" in r.issues and any("stalled" in i for i in r.issues)


def test_low_rtf_needs_to_persist():
    m = HealthMonitor(low_rtf=0.2, low_rtf_for_s=15)
    assert m.evaluate("running", snap(now=100, rtf=0.1)).health == "healthy"
    assert m.evaluate("running", snap(now=110, rtf=0.1)).health == "healthy"
    r = m.evaluate("running", snap(now=116, rtf=0.1))
    assert r.health == "degraded" and "real-time factor 0.10" in r.issues[0]
    assert m.evaluate("running", snap(now=117, rtf=0.9)).health == "healthy"   # recovers, timer resets
    assert m.evaluate("running", snap(now=118, rtf=0.1)).health == "healthy"
