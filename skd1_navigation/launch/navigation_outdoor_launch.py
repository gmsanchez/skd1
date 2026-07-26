#!/usr/bin/env python3
"""
navigation_outdoor_launch.py

Brings up a scan-free, costmap-obstacle-free Nav2 stack for GPS/EKF-only
outdoor navigation. Assumes dual_ekf_navsat (robot_localization) is already
running elsewhere and publishing map->odom->base_link.

Does NOT start map_server, amcl, collision_monitor, docking_server, or
route_server. Only starts:
  controller_server, planner_server, smoother_server, behavior_server,
  bt_navigator, waypoint_follower, velocity_smoother, and their
  lifecycle_manager.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---- Launch arguments ----
    # Defaults to <install>/share/skd1_navigation/config/nav2_outdoor_params.yaml
    # if not overridden on the command line / by an including launch file.
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("skd1_navigation"), "config", "nav2_outdoor_params.yaml"]
        ),
        description="Full path to the Nav2 parameters yaml file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically start the Nav2 lifecycle nodes",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")

    # Substitute use_sim_time into the yaml at load time so it matches
    # whatever value is passed on the command line, without editing the file.
    configured_params = ParameterFile(
        params_file,
        allow_substs=True,
    )

    # Only the nodes actually needed for GPS/EKF-only navigation.
    # No map_server, amcl, collision_monitor, docking_server, or route_server.
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "smoother_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
    )

    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
    )

    smoother_server_node = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
    )

    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
    )

    waypoint_follower_node = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
    )

    velocity_smoother_node = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": lifecycle_nodes,
            }
        ],
    )

    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            autostart_arg,
            log_level_arg,
            controller_server_node,
            planner_server_node,
            smoother_server_node,
            behavior_server_node,
            bt_navigator_node,
            waypoint_follower_node,
            velocity_smoother_node,
            lifecycle_manager_node,
        ]
    )