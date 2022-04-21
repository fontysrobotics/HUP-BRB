import os
from launch import LaunchDescription
import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, include_launch_description
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


WORLD_CONSTANT = 'waffle_pi'
MODEL_CONSTANT = 'waffle_pi'




def gen_robot_list(number_of_robots):

    robots = []

    for i in range(number_of_robots):
        robot_name = "robot"+str(i)
        x_pos = float(i)
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.01})

    return robots

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    package_description = "simulation"

    # We have to change this to our own package and model when we get it
    robot_urdf_name = "turtlebot3_" + MODEL_CONSTANT + ".urdf"
    robot_urdf = os.path.join(get_package_share_directory('simulation'), 'robot', robot_urdf_name)
    assert os.path.exists(robot_urdf), "turtlebot3_waffle_pi.urdf doesnt exist in "+str(robot_urdf)

    #Names and poses of the robots
    robots = gen_robot_list(4)

    #List of spawn robot commands
    spawn_robots_cmds = []
    
    #World model 
    world = os.path.join(get_package_share_directory('simulation'), 'worlds', WORLD_CONSTANT + ".model")
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    #-------------------------------------------------------------Launch gazebo------------------------------------------------------------------------#

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource( os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, "verbose": "true"}.items()
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    #-------------------------------------------------------------Launch RVIZ-----------------------------------------------------------------------------#
    # RVIZ Configuration
    """
    rviz_config_dir = os.path.join(get_package_share_directory(
        package_description), 'rviz', 'simulation.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir])
    """

    #--------------------------------------------------------------Robot spawning------------------------------------------------------------------------#
    for robot in robots:
        print("#Spawned: " + str(robot))
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(package_description, 'launch', 'spawn_robot_launch.py')),
                launch_arguments={
                    'robot_urdf': robot_urdf,
                    'x': TextSubstitution(text=str(robot['x_pose'])),
                    'y': TextSubstitution(text=str(robot['y_pose'])),
                    'z': TextSubstitution(text=str(robot['z_pose'])),
                    'robot_name': robot['name'],
                    'robot_namespace': robot['name']
                }.items()
            )
        )

    ld = LaunchDescription()
    ld.add_action(gz_server)
    ld.add_action(gz_client)   
    #ld.add_action(rviz_node) 
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)
    return ld
