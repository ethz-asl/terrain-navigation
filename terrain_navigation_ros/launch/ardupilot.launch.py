import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description."""

    pkg_ardupilot_sitl = get_package_share_directory("ardupilot_sitl")

    # TODO: hardcoded home location (lat, lon, alt, yaw) for davosdorf example.
    latitude_deg = 46.8132056
    longitude_deg = 9.8479538
    elevation = 1562.0
    heading_deg = 0
    home = f"'{latitude_deg}, {longitude_deg}, {elevation}, {heading_deg}'"

    # ardupilot sitl node with DDS and mavproxy
    sitl_dds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "sitl_dds_udp.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "transport": "udp4",
            "refs": PathJoinSubstitution(
                [
                    FindPackageShare("ardupilot_sitl"),
                    "config",
                    "dds_xrce_profile.xml",
                ]
            ),
            "port": "2019",
            "command": "arduplane",
            "synthetic_clock": "True",
            "wipe": "False",
            "model": "quadplane",
            "speedup": "1",
            "slave": "0",
            "instance": "0",
            "defaults": os.path.join(
                pkg_ardupilot_sitl,
                "config",
                "default_params",
                "quadplane.parm",
            )
            + ","
            + os.path.join(
                pkg_ardupilot_sitl,
                "config",
                "default_params",
                "dds_udp.parm",
            ),
            "home": home,
            "sim_address": "127.0.0.1",
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
        }.items(),
    )

    return LaunchDescription(
        [
            sitl_dds,
        ]
    )
