import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Before attempting to modify this file, it is recommended to follow the core ROS2 tutorials, and specifically for this file the guide at the link:
# https://docs.ros.org/en/foxy/Tutorials/Launch/Creating-Launch-Files.html

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(get_package_share_directory('dede'), 'worlds', "base_maze.model")
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    robot_urdf = os.path.join(get_package_share_directory('dede'), 'urdf', "dede.urdf")
    
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, "verbose": 'true'}.items()
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    robot_state = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[robot_urdf]
    )

    ld = LaunchDescription()
    ld.add_action(gz_server)
    ld.add_action(gz_client)
    ld.add_action(robot_state)
    return ld