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
    minimum_turn_radius = float(
        LaunchConfiguration("minimum_turn_radius").perform(context)
    )
    alt_control_p = float(LaunchConfiguration("alt_control_p").perform(context))
    alt_control_max_climb_rate = float(
        LaunchConfiguration("alt_control_max_climb_rate").perform(context)
    )
    cruise_speed = float(LaunchConfiguration("cruise_speed").perform(context))

    # terrain navigation node
    resource_path = os.path.join(pkg_terrain_navigation_ros, "resources")
    terrain_path = os.path.join(resource_path, location + ".tif")
    terrain_color_path = os.path.join(resource_path, location + "_color.tif")
    meshresource_path = os.path.join(resource_path, "believer.dae")

    # debug - check the context is resolved correctly.
    # print(f"resource_path:              {resource_path}")
    # print(f"terrain_path:               {terrain_path}")
    # print(f"terrain_color_path:         {terrain_color_path}")
    # print(f"minimum_turn_radius:        {minimum_turn_radius}")
    # print(f"alt_control_p:              {alt_control_p}")
    # print(f"alt_control_max_climb_rate: {alt_control_max_climb_rate}")
    # print(f"cruise_speed:               {cruise_speed}")

    # Create action.
    node = Node(
        package="terrain_navigation_ros",
        executable="terrain_planner_node",
        name="terrain_planner",
        parameters=[
            {"meshresource_path": meshresource_path},
            {"minimum_turn_radius": minimum_turn_radius},
            {"resource_path": resource_path},
            {"terrain_path": terrain_path},
            {"terrain_color_path": terrain_color_path},
            {"alt_control_p": alt_control_p},
            {"alt_control_max_climb_rate": alt_control_max_climb_rate},
            {"cruise_speed": cruise_speed},
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

    # ardupilot_sitl node
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
            # terrain_planner arguments
            DeclareLaunchArgument(
                "location", default_value="davosdorf", description="Location."
            ),
            DeclareLaunchArgument(
                "minimum_turn_radius",
                default_value="80.0",
                description="Minimum turn radius.",
            ),
            DeclareLaunchArgument(
                "alt_control_p",
                default_value="0.5",
                description="Altitude controller proportional gain.",
            ),
            DeclareLaunchArgument(
                "alt_control_max_climb_rate",
                default_value="3.0",
                description="Altitude controller maximim climb rate (m/s).",
            ),
            DeclareLaunchArgument(
                "cruise_speed",
                default_value="15.0",
                description="Vehicle cruise speed (m/s).",
            ),
            # rviz arguments
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz."
            ),
            terrain_navigation,
            rviz,
        ]
    )
