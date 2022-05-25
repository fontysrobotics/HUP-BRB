from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_projection',
            namespace='path_projection',
            executable='path_projection_node',
            name='path_projection'
        ),
        
    ])