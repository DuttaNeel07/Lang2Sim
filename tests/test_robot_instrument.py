"""Per-run robot instrumentation: bumper + ground-truth publisher + bridge topics."""

import ast
import xml.etree.ElementTree as ET

import pytest
import yaml

from lang2sim.core.models import Pose2D, SimStartRequest
from lang2sim.runtime import launch_builder
from lang2sim.runtime import robot_instrument as ri
from lang2sim.runtime.control_api.server import SimProcess

# Shape of turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf (Jazzy), trimmed.
STOCK_MODEL = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="turtlebot3_waffle">
    <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>
    <link name="base_footprint"/>
    <link name="base_link">
      <collision name="base_collision">
        <pose>-0.064 0 0.048 0 0 0</pose>
        <geometry><box><size>0.265 0.265 0.089</size></box></geometry>
      </collision>
      <visual name="base_visual"><geometry><mesh>
        <uri>model://turtlebot3_common/meshes/bases/waffle_base.stl</uri></mesh></geometry></visual>
    </link>
    <link name="base_scan"><collision name="lidar_sensor_collision"/></link>
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <odom_topic>odom</odom_topic>
    </plugin>
  </model>
</sdf>
"""

STOCK_BRIDGE = [
    {
        "ros_topic_name": "clock",
        "gz_topic_name": "clock",
        "ros_type_name": "rosgraph_msgs/msg/Clock",
        "gz_type_name": "gz.msgs.Clock",
        "direction": "GZ_TO_ROS",
    },
    {
        "ros_topic_name": "odom",
        "gz_topic_name": "odom",
        "ros_type_name": "nav_msgs/msg/Odometry",
        "gz_type_name": "gz.msgs.Odometry",
        "direction": "GZ_TO_ROS",
    },
]


def _model(text):
    return ET.fromstring(text).find("model")


def test_adds_bumper_on_chassis_and_ground_truth_publisher():
    text, coll = ri.instrument_model(STOCK_MODEL)
    assert coll == "base_collision"
    m = _model(text)
    base = next(ln for ln in m.findall("link") if ln.get("name") == "base_link")
    sensor = base.find("sensor")
    assert sensor.get("type") == "contact" and sensor.get("name") == ri.SENSOR_NAME
    assert sensor.find("contact/collision").text == "base_collision"
    assert sensor.find("topic").text == ri.CONTACT_GZ_TOPIC
    odom = [
        p for p in m.findall("plugin") if p.get("name") == "gz::sim::systems::OdometryPublisher"
    ]
    assert len(odom) == 1
    assert odom[0].find("odom_frame").text == "world"
    assert odom[0].find("odom_topic").text == ri.GROUND_TRUTH_TOPIC
    assert odom[0].find("tf_topic").text != "/tf"  # never fight AMCL over /tf
    # The stock content is untouched: mesh URIs and diff drive survive.
    assert "model://turtlebot3_common/meshes/bases/waffle_base.stl" in text
    assert any(p.get("name") == "gz::sim::systems::DiffDrive" for p in m.findall("plugin"))


def test_instrumentation_is_idempotent():
    once, _ = ri.instrument_model(STOCK_MODEL)
    twice, _ = ri.instrument_model(once)
    m = _model(twice)
    assert len(m.find("link[@name='base_link']").findall("sensor")) == 1
    assert len([p for p in m.findall("plugin") if "OdometryPublisher" in p.get("name")]) == 1


def test_rejects_models_it_cannot_instrument():
    with pytest.raises(ri.InstrumentError):
        ri.instrument_model("<sdf><model name='x'><link name='other'/></model></sdf>")
    with pytest.raises(ri.InstrumentError):
        ri.instrument_model("<sdf><model name='x'><link name='base_link'/></model></sdf>")
    with pytest.raises(ri.InstrumentError):
        ri.instrument_model("not xml")


def test_bridge_entries_cover_explicit_and_scoped_contact_topics():
    entries = ri.telemetry_bridge_entries("warehouse", "waffle")
    gz_topics = {e["gz_topic_name"] for e in entries}
    assert ri.CONTACT_GZ_TOPIC in gz_topics
    assert "/world/warehouse/model/waffle/link/base_link/sensor/l2s_bumper/contact" in gz_topics
    assert ri.GROUND_TRUTH_TOPIC in gz_topics
    assert {e["ros_topic_name"] for e in entries} == {ri.CONTACT_ROS_TOPIC, ri.GROUND_TRUTH_TOPIC}
    merged = ri.merge_bridge_config(STOCK_BRIDGE, entries)
    assert merged[:2] == STOCK_BRIDGE and len(merged) == 2 + len(entries)
    assert ri.merge_bridge_config(merged, entries) == merged


def test_write_instrumented_files(tmp_path):
    share = tmp_path / "share"
    (share).mkdir()
    (share / "model.sdf").write_text(STOCK_MODEL)
    (share / "bridge.yaml").write_text(yaml.safe_dump(STOCK_BRIDGE))
    run = tmp_path / "run"
    run.mkdir()
    sdf, yml, coll = ri.write_instrumented(
        run, share / "model.sdf", share / "bridge.yaml", world="w", robot="waffle"
    )
    assert (
        sdf.name == ri.ROBOT_SDF_NAME
        and yml.name == ri.BRIDGE_YAML_NAME
        and coll == "base_collision"
    )
    topics = [e["ros_topic_name"] for e in yaml.safe_load(yml.read_text())]
    assert "clock" in topics and ri.CONTACT_ROS_TOPIC in topics and ri.GROUND_TRUTH_TOPIC in topics


def _config(src):
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "CONFIG":
            return ast.literal_eval(node.value)
    raise AssertionError("no CONFIG")


def test_launch_uses_instrumented_files_when_given():
    cfg = launch_builder.LaunchConfig(
        run_dir="/r",
        world_sdf="/r/w.sdf",
        world_name="w",
        map_yaml="/r/m.yaml",
        robot_sdf="/r/robot.l2s.sdf",
        bridge_config="/r/bridge.l2s.yaml",
    )
    src = launch_builder.render(cfg)
    conf = _config(src)
    assert conf["robot_sdf"] == "/r/robot.l2s.sdf" and conf["bridge_config"] == "/r/bridge.l2s.yaml"
    assert 'CONFIG["robot_sdf"] or' in src and 'CONFIG["bridge_config"] or' in src
    stock = _config(
        launch_builder.render(
            launch_builder.LaunchConfig(
                run_dir="/r", world_sdf="/r/w.sdf", world_name="w", map_yaml="/r/m.yaml"
            )
        )
    )
    assert stock["robot_sdf"] == "" and stock["bridge_config"] == ""


def _req(tmp_path):
    run = tmp_path / "run"
    run.mkdir(parents=True)
    (run / "world.sdf").write_text(
        '<?xml version="1.0"?><sdf version="1.9"><world name="w"></world></sdf>'
    )
    (run / "map.yaml").write_text("image: map.pgm\n")
    return SimStartRequest(
        run_dir=str(run),
        world_sdf_path=str(run / "world.sdf"),
        map_yaml_path=str(run / "map.yaml"),
        initial_pose=Pose2D(x=1, y=2, yaw=0.5),
    )


def test_server_prepare_instruments_robot(tmp_path, monkeypatch):
    share = tmp_path / "tb3"
    (share / "models" / "turtlebot3_waffle").mkdir(parents=True)
    (share / "params").mkdir()
    (share / "models" / "turtlebot3_waffle" / "model.sdf").write_text(STOCK_MODEL)
    (share / "params" / "turtlebot3_waffle_bridge.yaml").write_text(yaml.safe_dump(STOCK_BRIDGE))
    monkeypatch.setattr("lang2sim.runtime.control_api.server.tb3_gazebo_share", lambda: share)
    req = _req(tmp_path)
    sim = SimProcess()
    conf = _config(sim.prepare(req).read_text())
    assert conf["robot_sdf"].endswith(ri.ROBOT_SDF_NAME) and conf["bridge_config"].endswith(
        ri.BRIDGE_YAML_NAME
    )
    assert sim.telemetry_setup["enabled"] and sim.telemetry_setup["collision"] == "base_collision"
    assert "instrumented" in (tmp_path / "run" / "logs" / "telemetry_setup.txt").read_text()
    # The world gets the Contact system the bumper needs.
    assert "gz-sim-contact-system" in (tmp_path / "run" / "world.l2s.sdf").read_text()


def test_server_prepare_falls_back_to_stock_robot(tmp_path, monkeypatch):
    monkeypatch.setattr("lang2sim.runtime.control_api.server.tb3_gazebo_share", lambda: None)
    sim = SimProcess()
    conf = _config(sim.prepare(_req(tmp_path)).read_text())
    assert conf["robot_sdf"] == "" and not sim.telemetry_setup["enabled"]
    monkeypatch.setenv("L2S_TELEMETRY", "0")
    sim.prepare(_req(tmp_path / "again"))
    assert "disabled" in sim.telemetry_setup["reason"]
