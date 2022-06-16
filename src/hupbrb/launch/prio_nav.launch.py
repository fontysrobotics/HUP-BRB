import os
from platform import node

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, LocalSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Paramater Setup
    namespace = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument('namespace',
        description='The name of the launching robot.')

    
    # Rviz2 remap
    scanner_node = Node(
        package='hupbrb',
        executable='id_scanner',
        namespace=namespace,
        output='screen',
    )
    priority_node = Node(
        package='hupbrb',
        executable='prio_controller',
        namespace=namespace,
        output='screen',
    )

    # Launch Description Setup
    ld = LaunchDescription()
    ld.add_action(namespace_arg)

    ld.add_action(scanner_node)
    ld.add_action(priority_node)
    return ld