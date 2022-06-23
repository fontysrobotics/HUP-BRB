import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    world_param = LaunchConfiguration('world')
    world_param_arg = DeclareLaunchArgument('world', default_value='open_space.world', description="The world to simulate")

    # File paths
    dede_share = get_package_share_directory('dede')
    world_sdf = PathJoinSubstitution([dede_share, 'worlds', world_param ])
    robot_sdf = os.path.join(dede_share, 'models', 'dede', 'model.sdf')
    robot_urdf = os.path.join(dede_share, 'urdf', "dede.urdf")
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Gazebo Client and Server
    gazebo_server= IncludeLaunchDescription(
        PythonLaunchDescriptionSource( os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_sdf, "verbose": 'true'}.items()
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Define robots
    robot_define = [
        {"name" : "dede1", "pose_x": "1.0", "pose_y": "1.0", "pose_z": "0.1", "roll": "0.0", "pitch": "0.0", "yaw": "0.0"},
        {"name" : "dede2", "pose_x": "2.0", "pose_y": "1.0", "pose_z": "0.1", "roll": "0.0", "pitch": "0.0", "yaw": "0.0"}
    ]

    # Spawn robots
    robot_group = []
    for robot in robot_define:
        robot_spawn = GroupAction([
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                namespace=TextSubstitution(text=robot["name"]),
                output='screen',
                arguments=[
                    '-entity', TextSubstitution(text=robot["name"]),
                    '-file', robot_sdf,
                    '-robot_namespace', TextSubstitution(text=robot["name"]),
                    '-x', TextSubstitution(text=robot["pose_x"]),
                    '-y', TextSubstitution(text=robot["pose_y"]),
                    '-z', TextSubstitution(text=robot["pose_z"]),
                    '-R', TextSubstitution(text=robot["roll"]),
                    '-P', TextSubstitution(text=robot["pitch"]),
                    '-Y', TextSubstitution(text=robot["yaw"])
                ]),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=TextSubstitution(text=robot['name']),
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=[robot_urdf],
                remappings= [('/tf', 'tf'), ('/tf_static', 'tf_static')]
            )
        ])
        robot_group.append(robot_spawn)

    # Launch Description Setup
    ld = LaunchDescription()
    ld.add_action(world_param_arg)

    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    for item in robot_group:
        ld.add_action(item)

    return ld