"""Launch GPR coverage stack: sim + Nav2 + single BT mission node."""

from os.path import join

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(v):
    return str(v).lower() in ("true", "1", "yes") if not isinstance(v, bool) else v


def _load_params(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)["gpr_mission"]["ros__parameters"]


def _world_for_scenario(pkg, scenario):
    if scenario == "polygon":
        return join(pkg, "worlds", "gpr_scan_polygon.sdf.xacro")
    return join(pkg, "worlds", "gpr_scan.sdf.xacro")


def _spawn_pose(p, scenario):
    if scenario == "polygon":
        return (
            str(p.get("sim.polygon.spawn_x", p.get("sim.spawn_x", -2.5))),
            str(p.get("sim.polygon.spawn_y", p.get("sim.spawn_y", -1.0))),
        )
    return (str(p.get("sim.spawn_x", -3.25)), str(p.get("sim.spawn_y", -1.75)))


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("gpr_bringup")
    params_file = LaunchConfiguration("params_file").perform(context) or join(
        pkg, "config", "gpr_coverage.yaml")
    p = _load_params(params_file)
    scenario_arg = LaunchConfiguration("scenario").perform(context)
    scenario = scenario_arg or p.get("scenario", "rectangle")
    world_file = _world_for_scenario(pkg, scenario)
    use_sim = _as_bool(p.get("launch.sim", True))
    use_sim_time = use_sim
    delay = float(p.get("launch.sim_startup_delay_sec", 3.0)) if use_sim else 0.0
    mission_delay = delay + float(p.get("launch.mission_startup_delay_sec", 8.0))

    spawn_x, spawn_y = _spawn_pose(p, scenario)
    spawn_yaw = str(p.get("sim.spawn_yaw", 0.0))
    spawn_z = str(p.get("sim.spawn_z", 0.01))

    actions = []
    if use_sim:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(join(pkg, "launch", "gz_sim.launch.py")),
            launch_arguments={
                "world": world_file,
                "use_sim_time": str(use_sim_time).lower(),
                "x_pose": spawn_x,
                "y_pose": spawn_y,
                "yaw": spawn_yaw,
                "z_pose": spawn_z,
            }.items(),
        ))

    if _as_bool(p.get("launch.static_tf", True)):
        # map == gazebo world. The gz DiffDrive plugin reports odom starting at the
        # spawn pose, so map->odom must equal the spawn pose (x, y, yaw).
        actions.append(TimerAction(period=delay, actions=[
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=[
                    "--x", spawn_x,
                    "--y", spawn_y,
                    "--z", "0",
                    "--yaw", spawn_yaw,
                    "--pitch", "0",
                    "--roll", "0",
                    "--frame-id", "map",
                    "--child-frame-id", "odom",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]))

    if _as_bool(p.get("launch.nav2", True)):
        # Pass nav2 params explicitly: the parent's empty "params_file" arg would
        # otherwise shadow the child default and Nav2 would start with no config.
        actions.append(TimerAction(period=delay, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(join(pkg, "launch", "nav2_gpr.launch.py")),
                launch_arguments={
                    "use_sim_time": str(use_sim_time).lower(),
                    "params_file": join(pkg, "config", "nav2_gpr_params.yaml"),
                }.items(),
            ),
        ]))

    mission_node = Node(
        package="gpr_mission",
        executable="gpr_mission_node",
        output="screen",
        parameters=[
            params_file,
            {
                "scenario": scenario,
                "oarp_lite.max_replan_generations": int(
                    LaunchConfiguration("oarp_lite.max_replan_generations").perform(context)
                    or p.get("oarp_lite.max_replan_generations", 2)
                ),
            },
        ],
    )
    actions.append(TimerAction(period=mission_delay, actions=[mission_node]))

    if _as_bool(p.get("launch.shutdown_on_mission_complete", True)) and _as_bool(
        p.get("mission.shutdown_on_complete", True)
    ):
        actions.append(RegisterEventHandler(
            OnProcessExit(
                target_action=mission_node,
                on_exit=[Shutdown(reason="GPR mission complete")],
            )
        ))

    if _as_bool(p.get("launch.rviz", True)):
        actions.append(TimerAction(period=delay, actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", join(pkg, "rviz", "gpr_scan.rviz")],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=""),
        DeclareLaunchArgument(
            "scenario",
            default_value="",
            description=(
                "Overrides scenario in gpr_coverage.yaml (rectangle | polygon/M). "
                "Leave empty to use the yaml value."
            ),
        ),
        DeclareLaunchArgument(
            "oarp_lite.max_replan_generations",
            default_value="",
            description=(
                "Max OARP rank batches per mission before giving up on replan "
                "(overrides gpr_coverage.yaml when set)."
            ),
        ),
        OpaqueFunction(function=_setup),
    ])
