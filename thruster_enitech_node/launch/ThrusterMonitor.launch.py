import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('thruster_enitech_node'),
        'config',
        'thruster_monitor.yaml'
    )

    return LaunchDescription([
        Node(
            package='thruster_enitech_node',
            executable='ThrusterMonitorExample',
            output='screen',
            parameters=[config_file]
        )
    ])
