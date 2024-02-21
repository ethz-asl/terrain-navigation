import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions.launch_configuration_equals import LaunchConfigurationEquals
from launch_ros.actions import Node


FLIGHT_STACK_PX4 = "px4"
FLIGHT_STACK_ARDUPILOT = "ardupilot"
FLIGHT_STACK_ARG_NAME = "flight_stack"


def generate_launch_description():
    # package directories
    pkg_mavros = get_package_share_directory("mavros")
    pkg_terrain_navigation_ros = get_package_share_directory("terrain_navigation_ros")

    FLIGHT_STACK_CONFIGS = {
        FLIGHT_STACK_PX4: {
            "fcu_url": "udp://127.0.0.1:14540@14557",
            "config_yaml": os.path.join(pkg_mavros, "launch", "px4_config.yaml"),
            "pluginlists_yaml": os.path.join(
                pkg_mavros, "launch", "px4_pluginlists.yaml"
            ),
        },
        FLIGHT_STACK_ARDUPILOT: {
            "fcu_url": "udp://127.0.0.1:14551@14555",
            "config_yaml": os.path.join(
                pkg_terrain_navigation_ros, "config", "ap_config.yaml"
            ),
            "pluginlists_yaml": os.path.join(
                pkg_terrain_navigation_ros, "config", "ap_pluginlists.yaml"
            ),
        },
    }

    flight_stack_arg = DeclareLaunchArgument(
        FLIGHT_STACK_ARG_NAME,
        default_value=FLIGHT_STACK_PX4,
        choices=[FLIGHT_STACK_PX4, FLIGHT_STACK_ARDUPILOT],
        description="Autopilot Type - See https://github.com/ROS-Aerial/aerial_robotic_landscape/blob/main/autopilots_suites.md",
    )

    gcs_url = ""
    tgt_system = 1
    tgt_component = 1
    fcu_protocol = "v2.0"
    respawn_mavros = False
    namespace = "mavros"

    condition_px4 = LaunchConfigurationEquals(
        FLIGHT_STACK_ARG_NAME, [TextSubstitution(text=FLIGHT_STACK_PX4)]
    )

    condition_ardupilot = LaunchConfigurationEquals(
        FLIGHT_STACK_ARG_NAME, [TextSubstitution(text=FLIGHT_STACK_ARDUPILOT)]
    )

    # mavros node for PX4
    mavros_px4 = Node(
        condition=condition_px4,
        package="mavros",
        executable="mavros_node",
        namespace=namespace,
        parameters=[
            {"fcu_url": FLIGHT_STACK_CONFIGS[FLIGHT_STACK_PX4]["fcu_url"]},
            {"gcs_url": gcs_url},
            {"tgt_system": tgt_system},
            {"tgt_component": tgt_component},
            {"fcu_protocol": fcu_protocol},
            FLIGHT_STACK_CONFIGS[FLIGHT_STACK_PX4]["pluginlists_yaml"],
            FLIGHT_STACK_CONFIGS[FLIGHT_STACK_PX4]["config_yaml"],
        ],
        remappings=[],
        respawn=respawn_mavros,
        output="screen",
    )

    # mavros node for ArduPilot
    mavros_ardupilot = Node(
        condition=condition_ardupilot,
        package="mavros",
        executable="mavros_node",
        namespace=namespace,
        parameters=[
            {"fcu_url": FLIGHT_STACK_CONFIGS[FLIGHT_STACK_ARDUPILOT]["fcu_url"]},
            {"gcs_url": gcs_url},
            {"tgt_system": tgt_system},
            {"tgt_component": tgt_component},
            {"fcu_protocol": fcu_protocol},
            FLIGHT_STACK_CONFIGS[FLIGHT_STACK_ARDUPILOT]["pluginlists_yaml"],
            FLIGHT_STACK_CONFIGS[FLIGHT_STACK_ARDUPILOT]["config_yaml"],
        ],
        remappings=[],
        respawn=respawn_mavros,
        output="screen",
    )

    return LaunchDescription([mavros_px4, mavros_ardupilot, flight_stack_arg])
