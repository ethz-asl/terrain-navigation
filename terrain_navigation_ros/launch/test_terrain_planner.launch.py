import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_terrain_navigation_action(
    context: LaunchContext, *args, **kwargs
) -> [ExecuteProcess]:
    """Generate the terrain_navigation_ros launch action."""

    pkg_terrain_navigation_ros = get_package_share_directory("terrain_navigation_ros")

    # arguments.
    location = LaunchConfiguration("location").perform(context)

    # terrain navigation node
    terrain_path = os.path.join(
        pkg_terrain_navigation_ros,
        "../resources",
        location + ".tf",
    )
    terrain_color_path = os.path.join(
        pkg_terrain_navigation_ros,
        "../resources",
        location + "_color.tf",
    )
    resource_path = os.path.join(pkg_terrain_navigation_ros, "../resources")

    # debug - check the context is resolved correctly.
    # print(f"terrain_path:       {terrain_path}")
    # print(f"terrain_color_path: {terrain_color_path}")
    # print(f"resource_path:      {resource_path}")

    # Create action.
    node = Node(
        package="terrain_navigation_ros",
        executable="terrain_planner_node",
        name="terrain_planner",
        parameters=[
            {"terrain_path": terrain_path},
            {"terrain_color_path": terrain_color_path},
            {"resource_path": resource_path},
            {"minimum_turn_radius": "80.0"},
        ],
        output="screen",
    )
    return [node]


def generate_launch_description():
    """Generate a launch description."""

    pkg_terrain_planner = get_package_share_directory("terrain_planner")

    # mavros node
    # TODO include launch description 

    # px4 posix_sitl node
    # TODO include launch description 

    # terrain navigation node
    terrain_navigation = OpaqueFunction(function=generate_terrain_navigation_action)

    # rviz node
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_terrain_planner, "launch", "config.rviz")],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mav_name", default_value="tiltrotor", description="Vehicle name."
            ),
            DeclareLaunchArgument(
                "fcu_url",
                default_value="udp://:14540@127.0.0.1:14557",
                description="Flight controller address.",
            ),
            DeclareLaunchArgument(
                "gcs_url",
                default_value="",
                description="Ground control station address.",
            ),
            DeclareLaunchArgument("tgt_system", default_value="1", description=""),
            DeclareLaunchArgument("tgt_component", default_value="1", description=""),
            DeclareLaunchArgument("command_input", default_value="2", description=""),
            DeclareLaunchArgument(
                "gazebo_simulation", default_value="true", description="Run Gazebo."
            ),
            DeclareLaunchArgument(
                "log_output",
                default_value="screen",
                description="Location to log output.",
            ),
            DeclareLaunchArgument(
                "fcu_protocol",
                default_value="v2.0",
                description="Flight controller protocol version.",
            ),
            DeclareLaunchArgument(
                "respawn_mavros", default_value="false", description="Respawn mavros."
            ),
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            DeclareLaunchArgument(
                "location", default_value="davosdorf", description="Location."
            ),
            DeclareLaunchArgument(
                "gui", default_value="false", description="Open SITL GUI."
            ),
            terrain_navigation,
            rviz,
        ]
    )
