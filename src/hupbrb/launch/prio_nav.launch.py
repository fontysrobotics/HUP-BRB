import os
from platform import node

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution, LocalSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Paramater Setup
    namespace_prefix = LaunchConfiguration('namespace_prefix')
    namespace_prefix_arg = DeclareLaunchArgument('namespace_prefix', 
        default_value = 'dede',
        description='The identifier of the current robot, used in its namespace and QR code.')

    node_id = LaunchConfiguration('id')
    node_id_arg = DeclareLaunchArgument('id', 
        description='The identifier of the current robot, used in its namespace and QR code.')
    
    # Rviz2 remap
    rviz_node = Node(
        package='hupbrb',
        executable='id_scanner',
        namespace=[namespace_prefix, node_id],
        output='screen',
    )

    # Launch Description Setup
    ld = LaunchDescription()
    ld.add_action(namespace_prefix_arg)
    ld.add_action(node_id_arg)
    ld.add_action(rviz_node)
    return ld