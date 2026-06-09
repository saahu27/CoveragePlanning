"""Minimal Nav2 launch for GPR coverage FollowPath."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace, SetParameter


def generate_launch_description():
    pkg = get_package_share_directory("gpr_bringup")
    default_params = os.path.join(pkg, "config", "nav2_gpr_params.yaml")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    lifecycle_nodes = ["controller_server", "velocity_smoother", "collision_monitor"]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),
        GroupAction([
            SetParameter("use_sim_time", use_sim_time),
            PushROSNamespace(namespace=namespace),
            Node(
                package="nav2_controller",
                executable="controller_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "node_names": lifecycle_nodes,
                }],
            ),
        ]),
    ])
