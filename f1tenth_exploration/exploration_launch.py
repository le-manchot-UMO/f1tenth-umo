#!/usr/bin/env python3
"""F1TENTH autonomous exploration stack (ROS 2 Humble).

Assumes the hardware is ALREADY up in another terminal:

    ros2 launch f1tenth_stack bringup_launch.py \
        vesc_config:=~/f1tenth_exploration/vesc_ekf.yaml

This launch adds everything above the hardware:

    /sensors/imu/raw -> vesc_imu_conditioner -> /imu/data ---+
                                                             +-> ekf_filter_node
    vesc_to_odom     -> /odom -------------------------------+       |
                                                                     v
                                                        /odometry/filtered
                                                        odom -> base_link TF

    /scan -> slam_toolbox -> /map + map -> odom TF
    /scan + /map -> Nav2 costmaps -> planner/controller -> /cmd_vel
    /cmd_vel -> cmd_vel_to_ackermann -> /drive -> ackermann_mux -> VESC

AMCL and map_server are deliberately absent: SLAM Toolbox owns /map and the
map->odom transform while exploring. Add them back only when localising against
an already-saved map.

The frontier explorer is NOT launched here. It needs the /navigate_to_pose
action server to exist first, so run_exploration.sh starts it separately once
Nav2 has activated.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    ekf_params_file = LaunchConfiguration("ekf_params_file")

    # Humble names: behavior_server (was recoveries_server in Galactic).
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    # ---- odometry fusion ---------------------------------------------------
    # The VESC IMU is mounted flat and reads +1 g on z, so its axes already
    # agree with base_link: identity transform. Change this if you remount it.
    imu_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_base_link_to_imu",
        arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0",
                   "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                   "--frame-id", "base_link", "--child-frame-id", "imu_link"],
    )

    # These two are standalone scripts rather than installed ROS packages, so
    # they run via ExecuteProcess instead of launch_ros Node.
    imu_conditioner = ExecuteProcess(
        cmd=["python3", os.path.join(here, "vesc_imu_conditioner.py")],
        name="vesc_imu_conditioner",
        output="screen",
    )

    cmd_vel_bridge = ExecuteProcess(
        cmd=["python3", os.path.join(here, "cmd_vel_to_ackermann.py")],
        name="cmd_vel_to_ackermann",
        output="screen",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file, {"use_sim_time": use_sim_time}],
    )

    # ---- mapping -----------------------------------------------------------
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
    )

    # ---- Nav2 --------------------------------------------------------------
    controller_server = Node(
        package="nav2_controller", executable="controller_server",
        name="controller_server", output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    planner_server = Node(
        package="nav2_planner", executable="planner_server",
        name="planner_server", output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    behavior_server = Node(
        package="nav2_behaviors", executable="behavior_server",
        name="behavior_server", output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    bt_navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        name="bt_navigator", output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower", executable="waypoint_follower",
        name="waypoint_follower", output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", output="screen",
        parameters=[{"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes}],
    )

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(here, "nav2_params.yaml")),
        DeclareLaunchArgument(
            "slam_params_file", default_value=os.path.join(here, "slam_params.yaml")),
        DeclareLaunchArgument(
            "ekf_params_file", default_value=os.path.join(here, "ekf.yaml")),
        imu_static_tf,
        imu_conditioner,
        ekf_node,
        cmd_vel_bridge,
        slam_node,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
    ])
