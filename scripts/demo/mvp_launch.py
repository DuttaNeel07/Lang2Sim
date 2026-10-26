"""
MVP demo launch: TurtleBot3 Waffle in the TurtleBot3 world + Nav2 + RViz.
File: scripts/demo/mvp_launch.py   (runs INSIDE the sim container)

A hand-written preview of what Step 3's launch_builder will render per run
(sim_launch.py). Unlike the stock turtlebot3_world.launch.py it can run without
the Gazebo GUI and closing the GUI window does not kill the simulation.

    ros2 launch scripts/demo/mvp_launch.py gui:=true rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tb3_gz = get_package_share_directory("turtlebot3_gazebo")
    tb3_nav = get_package_share_directory("turtlebot3_navigation2")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map")
    params = LaunchConfiguration("params_file")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")

    args = [
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo GUI window"),
        DeclareLaunchArgument("rviz", default_value="true", description="RViz2 with Nav2 panels"),
        DeclareLaunchArgument("world", default_value=os.path.join(tb3_gz, "worlds", "turtlebot3_world.world")),
        DeclareLaunchArgument("map", default_value=os.path.join(tb3_nav, "map", "map.yaml")),
        DeclareLaunchArgument("params_file", default_value=os.path.join(tb3_nav, "param", "waffle.yaml")),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),
    ]

    # Gazebo server: the sim itself. Its exit ends the whole launch.
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -s -v2 ", world], "on_exit_shutdown": "true"}.items(),
    )
    # Gazebo GUI: optional, and closing it must not stop the sim.
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-g -v2 ", "on_exit_shutdown": "false"}.items(),
        condition=IfCondition(gui),
    )
    # robot_state_publisher (URDF -> /tf_static, /robot_description)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gz, "launch", "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items(),
    )
    # Spawn the waffle + ros_gz_bridge (/clock /scan /odom /tf /cmd_vel /joint_states /imu)
    # + image bridge (/camera/image_raw).
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_gz, "launch", "spawn_turtlebot3.launch.py")),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )
    # Nav2: map_server + AMCL (map->odom) + planner/controller/BT navigator.
    # Delayed so /clock and the robot's TF exist before the lifecycle manager activates.
    nav2 = TimerAction(period=5.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup, "launch", "bringup_launch.py")),
        launch_arguments={"map": map_yaml, "use_sim_time": "true",
                          "params_file": params, "autostart": "true"}.items(),
    )])
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", os.path.join(tb3_nav, "rviz", "tb3_navigation2.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        *args,
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(tb3_gz, "models")),
        gz_server, gz_gui, rsp, spawn, nav2, rviz_node,
    ])
