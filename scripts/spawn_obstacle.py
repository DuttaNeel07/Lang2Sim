"""
Dynamically drop an obstacle or rack into the running Gazebo simulation.
File: scripts/spawn_obstacle.py

Usage:
    python3 scripts/spawn_obstacle.py --model rack --x 0.0 --y 1.0
    python3 scripts/spawn_obstacle.py --model crate --x 1.5 --y 0.5
    python3 scripts/spawn_obstacle.py --model barrel --x 2.0 --y -0.5
"""

import argparse
import subprocess
import sys


SDF_TEMPLATES = {
    "rack": """<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="c">
        <geometry><box><size>2.8 1.2 3.0</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>2.8 1.2 3.0</size></box></geometry>
        <material>
          <ambient>0.2 0.4 0.7 1</ambient>
          <diffuse>0.2 0.4 0.7 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>""",
    "crate": """<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial><mass>15.0</mass></inertial>
      <collision name="c">
        <geometry><box><size>0.8 0.8 0.8</size></box></geometry>
      </collision>
      <visual name="v">
        <geometry><box><size>0.8 0.8 0.8</size></box></geometry>
        <material>
          <ambient>0.9 0.4 0.1 1</ambient>
          <diffuse>0.9 0.4 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>""",
    "barrel": """<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <inertial><mass>20.0</mass></inertial>
      <collision name="c">
        <geometry><cylinder><radius>0.35</radius><length>0.9</length></cylinder></geometry>
      </collision>
      <visual name="v">
        <geometry><cylinder><radius>0.35</radius><length>0.9</length></cylinder></geometry>
        <material>
          <ambient>0.8 0.1 0.1 1</ambient>
          <diffuse>0.8 0.1 0.1 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>""",
}


def main():
    parser = argparse.ArgumentParser(description="Spawn a dynamic obstacle into the running Gazebo world.")
    parser.add_argument("--model", choices=["rack", "crate", "barrel"], default="rack", help="Type of model to spawn.")
    parser.add_argument("--name", default=None, help="Unique name for the spawned entity.")
    parser.add_argument("--x", type=float, default=0.0, help="X position (meters).")
    parser.add_argument("--y", type=float, default=0.0, help="Y position (meters).")
    parser.add_argument("--z", type=float, default=1.5, help="Z position (meters).")
    parser.add_argument("--world", default=None, help="Gazebo world name (auto-detected if omitted).")
    args = parser.parse_args()

    # Auto-detect world name from Gazebo services if not given
    world_name = args.world
    if not world_name:
        try:
            res = subprocess.run(
                ["docker", "exec", "lang2sim-sim", "gz", "service", "-l"],
                capture_output=True, text=True, check=True
            )
            for line in res.stdout.splitlines():
                if line.startswith("/world/") and line.endswith("/create"):
                    world_name = line.split("/")[2]
                    break
        except Exception:
            world_name = "default_world"

    if not world_name:
        world_name = "default_world"

    entity_name = args.name or f"dynamic_{args.model}_{int(abs(args.x * 100))}"
    sdf_content = SDF_TEMPLATES[args.model].format(name=entity_name).replace("\n", " ")

    inner_cmd = f"source /opt/ros/jazzy/setup.bash && ros2 run ros_gz_sim create -world {world_name} -name {entity_name} -string '{sdf_content}' -x {args.x} -y {args.y} -z {args.z if args.model == 'rack' else 0.4}"
    cmd = [
        "docker", "exec", "lang2sim-sim",
        "/bin/bash", "-c", inner_cmd,
    ]

    print(f"==> Spawning dynamic {args.model} '{entity_name}' at ({args.x}, {args.y}) in world '{world_name}'...")
    ret = subprocess.run(cmd)
    if ret.returncode == 0:
        print(f"✔ Success! {args.model.capitalize()} '{entity_name}' dropped into Gazebo.")
        print(f"  Watch the robot's LiDAR detect the new obstacle and re-plan its path!")
    else:
        print(f"✖ Failed to spawn entity (exit code: {ret.returncode})")


if __name__ == "__main__":
    main()
