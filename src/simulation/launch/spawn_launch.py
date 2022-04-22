import os
import sys

import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'turtlebot3_waffle_pi.urdf'
    sdf_file_name = 'turtlebot3_waffle_pi/model.sdf'
    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(get_package_share_directory('simulation'), 'urdf', urdf_file_name)

    sdf_model = os.path.join(get_package_share_directory('simulation'), 'models', sdf_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    
    ld = launch.LaunchDescription([
        Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher1',
          output='screen',
          parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc}],
          arguments=[urdf]),

        Node(
            package='gazebo_ros', 
            executable='spawn_entity.py', 
            arguments=['-entity', 'robot1', 
               '-file', sdf_model,
                '-x', '-3.0',
                '-y', '0.0',
                '-z', '0.0'],
            output='screen'),
        
        Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher2',
          output='screen',
          parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_desc}],
          arguments=[urdf]),

        Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=['-entity', 'robot2', 
            '-file', sdf_model,
            '-x', '-1.0',
            '-y', '0.0',
            '-z', '0.0'],
        output='screen',
        namespace='robot2'),

    ])
    return ld