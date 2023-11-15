import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # package directories
    pkg_mavros = get_package_share_directory("mavros")
    pkg_terrain_navigation_ros = get_package_share_directory("terrain_navigation_ros")

    # TODO: remove hardcoding

    # mavros config
    # ardupilot
    fcu_url = "udp://127.0.0.1:14551@14555"
    # px4
    fcu_url = "udp://127.0.0.1:14540@14557"

    gcs_url = ""
    tgt_system = 1
    tgt_component = 1
    fcu_protocol = "v2.0"
    respawn_mavros = False
    namespace = "mavros"

    # ardupilot
    config_yaml = os.path.join(pkg_terrain_navigation_ros, "config", "ap_config.yaml")
    pluginlists_yaml = os.path.join(
        pkg_terrain_navigation_ros, "config", "ap_pluginlists.yaml"
    )
    # px4
    config_yaml = os.path.join(pkg_mavros, "launch", "px4_config.yaml")
    pluginlists_yaml = os.path.join(pkg_mavros, "launch", "px4_pluginlists.yaml")

    # mavros node
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        namespace=namespace,
        parameters=[
            {"fcu_url": fcu_url},
            {"gcs_url": gcs_url},
            {"tgt_system": tgt_system},
            {"tgt_component": tgt_component},
            {"fcu_protocol": fcu_protocol},
            pluginlists_yaml,
            config_yaml,
        ],
        remappings=[],
        respawn=respawn_mavros,
        output="screen",
    )

    return LaunchDescription(
        [
            mavros,
        ]
    )
