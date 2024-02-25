import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate a launch description."""

    pkg_terrain_planner = get_package_share_directory("terrain_planner")

    # rviz node
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_terrain_planner, "launch", "config.rviz")],
    )

    return LaunchDescription(
        [
            rviz,
        ]
    )
