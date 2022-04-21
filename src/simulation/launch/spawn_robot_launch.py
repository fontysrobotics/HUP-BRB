import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy import parameter

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = LaunchConfiguration('robot_name', default='robot')

    urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_waffle_pi.urdf')
    assert os.path.exists(urdf), "turtlebot3_waffle_pi.urdf doesnt exist in "+str(urdf)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(package='simulation', executable='spawn_robot_srv', arguments=[urdf, robot_name], output='screen'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
    ])