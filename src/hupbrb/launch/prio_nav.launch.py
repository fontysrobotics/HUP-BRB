import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, LocalSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Paramater Setup
    namespace = os.uname().nodename
    # LaunchConfiguration('namespace')
    # namespace_arg = DeclareLaunchArgument('namespace',
    #     description='The name of the launching robot.')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Use simulation (Gazebo) clock if true')

    # Turtlebot stuff

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')
    usb_port_arg = DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            TURTLEBOT3_MODEL + '.yaml'))
    tb3_param_dir_arg = DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load')
            
    # lidar_pkg_dir = LaunchConfiguration(
    #     'lidar_pkg_dir',
    #     default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    
    
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)


    
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

    #turtlebot 3
    turtlebot3 = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        namespace=namespace,
        parameters=[tb3_param_dir],
        arguments=['-i', usb_port, '--ros-args', '--log-level', 'DEBUG'],
        output='screen')

    turtle_state_publisher =  Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf])

    lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
    LDS_LAUNCH_FILE = '/ld08.launch.py'


    # Launch Description Setup
    ld = LaunchDescription()
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ))
    # ld.add_action(namespace_arg)
    ld.add_action(usb_port_arg)
    ld.add_action(tb3_param_dir_arg)
    ld.add_action(use_sim_time_arg)

    ld.add_action(turtlebot3)
    ld.add_action(turtle_state_publisher)
    ld.add_action(scanner_node)
    ld.add_action(priority_node)
    return ld