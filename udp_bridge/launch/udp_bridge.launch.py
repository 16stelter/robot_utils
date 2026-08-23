# udp_bridge_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(get_package_share_directory("udp_bridge"), "config", "udp_bridge.yaml"),
        description="Path to the configuration file for the UDP bridge"
    )

    receiver_node = Node(
        package='udp_bridge',
        executable='receiver',
        output='screen',
        parameters=[
            {
                'config_file': LaunchConfiguration('config_file')
            }
        ]
    )

    sender_node = Node(
        package='udp_bridge',
        executable='sender',
        output='screen',
        parameters=[
            {
                'config_file': LaunchConfiguration('config_file')
            }
        ]
    )

    return LaunchDescription([
        config_file_arg,
        receiver_node,
        sender_node
    ])