"""
Per-run robot instrumentation for the Telemetry Engine.
File: lang2sim/runtime/robot_instrument.py

Architecture §6 Phase 3: "Instrument robot chassis collision meshes with contact
sensor plugins" and "implement telemetry recording for kinematics and collision
events". The stock TurtleBot3 model has neither a bumper nor a world-frame pose
source, so for every run we write an instrumented copy of its model.sdf into the
run folder and spawn that instead:

  * a contact sensor on base_link's chassis collision, publishing gz.msgs.Contacts
    (needs the world's gz::sim::systems::Contact, which world_check now adds);
  * gz::sim::systems::OdometryPublisher with odom_frame=world: Gazebo's ground-truth
    world pose + twist of the robot, independent of AMCL and of wheel odometry.

and a bridge config = the stock TurtleBot3 one + these two topics:

  gz <contact topic>                 -> ROS /lang2sim/contact_states  (ros_gz_interfaces/Contacts)
  gz /lang2sim/ground_truth/odom     -> ROS /lang2sim/ground_truth/odom (nav_msgs/Odometry)

The contact sensor's topic is set explicitly, but both it and Gazebo's default
scoped topic are bridged to the same ROS topic, so the data arrives whichever one
this Gazebo build honours (only one of them ever publishes).

Mesh URIs in the stock model are model://..., which stay resolvable from the run
folder because GZ_SIM_RESOURCE_PATH includes turtlebot3_gazebo/models.

Pure XML/YAML transforms, unit-tested on the host; the server calls
write_instrumented() inside the container where the stock files exist.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

CONTACT_ROS_TOPIC = "/lang2sim/contact_states"
CONTACT_GZ_TOPIC = "/lang2sim/contact_states"
GROUND_TRUTH_TOPIC = "/lang2sim/ground_truth/odom"
SENSOR_NAME = "l2s_bumper"
CHASSIS_LINK = "base_link"
ROBOT_SDF_NAME = "robot.l2s.sdf"
BRIDGE_YAML_NAME = "bridge.l2s.yaml"
GROUND_TRUTH_HZ = 30
CONTACT_HZ = 50


class InstrumentError(ValueError):
    pass


def default_contact_topic(
    world: str, robot: str, link: str = CHASSIS_LINK, sensor: str = SENSOR_NAME
) -> str:
    """Gazebo's scoped topic for a contact sensor that sets no explicit topic."""
    return f"/world/{world}/model/{robot}/link/{link}/sensor/{sensor}/contact"


def _sub(parent: ET.Element, tag: str, text: Optional[str] = None) -> ET.Element:
    el = ET.SubElement(parent, tag)
    if text is not None:
        el.text = text
    return el


def instrument_model(
    sdf_text: str,
    *,
    link: str = CHASSIS_LINK,
    collision: Optional[str] = None,
    contact_topic: str = CONTACT_GZ_TOPIC,
    ground_truth_topic: str = GROUND_TRUTH_TOPIC,
    base_frame: str = "base_footprint",
) -> Tuple[str, str]:
    """Add the bumper + ground-truth publisher to a robot model SDF.

    Returns (new SDF text, chassis collision name used). Idempotent: an SDF that
    already carries the sensor/plugin is returned with nothing added twice.
    """
    try:
        root = ET.fromstring(sdf_text)
    except ET.ParseError as e:
        raise InstrumentError(f"robot model is not valid XML: {e}") from e
    model = root if root.tag == "model" else root.find("model")
    if model is None:
        raise InstrumentError("no <model> element in robot SDF")
    chassis = next((ln for ln in model.findall("link") if ln.get("name") == link), None)
    if chassis is None:
        raise InstrumentError(f"robot model has no link '{link}'")
    collisions = [c.get("name") for c in chassis.findall("collision") if c.get("name")]
    if not collisions:
        raise InstrumentError(f"link '{link}' has no named <collision> to attach a bumper to")
    chosen = (
        collision
        if collision in collisions
        else ("base_collision" if "base_collision" in collisions else collisions[0])
    )

    if not any(s.get("name") == SENSOR_NAME for s in chassis.findall("sensor")):
        sensor = ET.SubElement(chassis, "sensor", {"name": SENSOR_NAME, "type": "contact"})
        _sub(sensor, "always_on", "true")
        _sub(sensor, "update_rate", str(CONTACT_HZ))
        _sub(sensor, "topic", contact_topic)
        contact = _sub(sensor, "contact")
        _sub(contact, "collision", chosen)
        _sub(contact, "topic", contact_topic)

    odom_cls = "gz::sim::systems::OdometryPublisher"
    if not any(p.get("name") == odom_cls for p in model.findall("plugin")):
        plugin = ET.SubElement(
            model, "plugin", {"filename": "gz-sim-odometry-publisher-system", "name": odom_cls}
        )
        _sub(plugin, "odom_frame", "world")
        _sub(plugin, "robot_base_frame", base_frame)
        _sub(plugin, "odom_topic", ground_truth_topic)
        _sub(plugin, "odom_publish_frequency", str(GROUND_TRUTH_HZ))
        _sub(plugin, "dimensions", "2")
        # Keep its TF off /tf so it never fights AMCL/diff-drive for odom->base_footprint.
        _sub(plugin, "tf_topic", "/lang2sim/ground_truth/tf")

    ET.indent(root, space="  ")
    out = ET.tostring(root, encoding="unicode")
    if not out.startswith("<?xml"):
        out = '<?xml version="1.0" ?>\n' + out
    return out + "\n", chosen


def telemetry_bridge_entries(world: str, robot: str) -> List[Dict[str, str]]:
    """ros_gz_bridge config entries for the instrumented topics."""
    contact = {
        "ros_topic_name": CONTACT_ROS_TOPIC,
        "ros_type_name": "ros_gz_interfaces/msg/Contacts",
        "gz_type_name": "gz.msgs.Contacts",
        "direction": "GZ_TO_ROS",
    }
    entries = [dict(contact, gz_topic_name=CONTACT_GZ_TOPIC)]
    scoped = default_contact_topic(world, robot)
    if scoped != CONTACT_GZ_TOPIC:
        entries.append(dict(contact, gz_topic_name=scoped))
    entries.append(
        {
            "ros_topic_name": GROUND_TRUTH_TOPIC,
            "gz_topic_name": GROUND_TRUTH_TOPIC,
            "ros_type_name": "nav_msgs/msg/Odometry",
            "gz_type_name": "gz.msgs.Odometry",
            "direction": "GZ_TO_ROS",
        }
    )
    return entries


def merge_bridge_config(stock: list, extra: List[Dict[str, str]]) -> list:
    """Stock bridge entries + extra ones, skipping any (ros_topic, gz_topic) pair already present."""
    seen = {
        (e.get("ros_topic_name"), e.get("gz_topic_name"))
        for e in stock or []
        if isinstance(e, dict)
    }
    out = list(stock or [])
    for e in extra:
        if (e["ros_topic_name"], e["gz_topic_name"]) not in seen:
            out.append(e)
    return out


def write_instrumented(
    run_dir: Path, stock_model: Path, stock_bridge: Path, *, world: str, robot: str
) -> Tuple[Path, Path, str]:
    """Write robot.l2s.sdf and bridge.l2s.yaml into run_dir. Returns (sdf, yaml, collision name)."""
    import yaml  # the sim image ships PyYAML (launch files use it); imported lazily for clarity

    text, collision = instrument_model(Path(stock_model).read_text(encoding="utf-8"))
    sdf_out = Path(run_dir) / ROBOT_SDF_NAME
    sdf_out.write_text(text, encoding="utf-8")
    stock = yaml.safe_load(Path(stock_bridge).read_text(encoding="utf-8")) or []
    merged = merge_bridge_config(stock, telemetry_bridge_entries(world, robot))
    yaml_out = Path(run_dir) / BRIDGE_YAML_NAME
    yaml_out.write_text(
        "# Generated by lang2sim (runtime/robot_instrument.py): TurtleBot3 bridge + telemetry topics.\n"
        + yaml.safe_dump(merged, sort_keys=False),
        encoding="utf-8",
    )
    return sdf_out, yaml_out, collision
