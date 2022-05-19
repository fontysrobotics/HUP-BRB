# Mapping and Navigation for DeDe
This package uses the core Navigation2 package and the SLAM Toolbox package for ROS2 for the navigation and mapping tasks.

## Installation
In order to ensure that the package has all the dependencies when built in the development directory. The package was tested against foxy and all the dependencies present in the respective ROS2 version. The following packages should be installed:
```bash
sudo apt install ros-foxy-navigation2 
sudo apt install ros-foxy-slam-toolbox
```

## Base Mapping Launch file
The package contains a mapping file for the Dede robot using the slam toolbox package. Launching the mapping is done with the following command:
```bash
ros2 launch dede slam_toolbox.launch.py
```
**Note!** Slam toolbox depends on the simulation or the real robot to be active and publishing on the required topics. Without the topics the launch will fail and as a consequence it will error or stall in wait for the topics to start.

Once the mapping is running, the robot can be manually driven in order to create a map. At any moment a mapping file can be created on the storage with the following command, substitute **"map_name"** with the desired name:
```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename : 'map_name'}"
```
**Note!** Slam toolbox will save the map in the working directory where the launch file was executed instead of the directory where the save command is executed.

**Note!** When a file is created, slam toolbox will update the file when more progress on the map building is done. In the eventuality the mapping process results in errors, the file can be automatically updated and store the error. If the error is saved on the map file, it is required in most cases to start over with the mapping process.

## Understanding the Mapping Launch file
This subchapter will break down the launch file found in the package at the location *launch/slam_toolbox.launch.py*. The structure will be presented, followed by the node executed and the config file for it.

*Note!* Before progressing with this subchapter, is better to be familiar with the ROS2 basic launch file structure. The official tutorial can be found on the [ROS2 Wiki](https://docs.ros.org/en/foxy/Tutorials/Launch/Creating-Launch-Files.html)

### 1) Launch file structure
The launch file contains commented code segments for ease of reference. The code is grouped in segments that have a specific role in the launch file. The following segments are present:
- *Configuration paths:* This segment contains a single line of code. The purpose of it is to look up the path of the configuration file. ROS2 allows the packages to be installed in multiple locations, this makes the path of resource files be variable based on the system setup. This is mitigated by the lookup function included in ROS2, this function accesses the package registry to get the file path where the package is installed. To this path, representing the root of the package, it can be appended the desired target file or directory.
- *Parameters setup:* This segment contains the code for setting up the launch file parameters, their default values and a short description. These parameters allow the adjustment of the launch behavior without needing to modify the file itself. The following parameters are set:
    - *use_sim_time:* A boolean, defaulting to *"true"*, the parameter sets the nodes to either use the gazebo time stamp or the system time stamp. Set it to false when using a real robot.
    - *use_map_file:* A boolean, defaulting to *"false"*. When set to "*true*" the launch file expects additional parameters in order to load an existing launch file and resume mapping on it. Otherwise it will start a new map.
    - *map_file:* A string, the parameter needs to be set when resuming mapping from a file. It is recommended to use an absolute path for the map file. The file path should not contain the file extension.
    - *map_pose:* A string representing a float array of size 3. The array values represent the X, Y and theta values given by odometry. The parameter needs to be set when resuming mapping from a file. 
- *Nodes and flow control:* This section contains the node definition and the behavior control. The only node that is launched is the *"slam_toolbox"*. Using GroupAction, the same node can be configured to use different parameters based on the value set by *"use_map_file"*. There are two flows:
    - When the parameter is set to *"false"*, the slam toolbox node is set to start without any previous map data, a new map graph is created.
    - When the parameter is set to *"true"*. the slam toolbox is set to resume and update an existing map file, the file path and the position of the robot on the map needs to be set.
- *Return object:* This segment creates the return object and appends to it all the actions and parameters before exiting the function. If an action is not added to the LaunchDescription object, it won't be executed by the launch file.

### 2) Launch File Nodes
The launch file contains only a single node. The node subscribes to a scan topic for LaserScan data and odometry topic for Odometry data and publishes Map data on a map topic. As the robot moves in the world, the algorithm combines the sensor and motion data in order to create a pose graph with the aligned scan data, this is further processed into a 2D map of the environment. As the map is build, the algorithm adjusts previous pose elements based on the features detected in the environment. These adjustments change the mapped environment, shift and align walls as well as modify the current pose of the robot to correct for drift of the wheels.

### 3) Configuration Files
The node *"slam_toolbox"* has its configuration file at the location *"config/slam/mapping.yaml"*. The parameters are described on the official github repository for the [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox#configuration)

Some of the important parameters are:
- *odom_frame, map_frame, base_frame:* Transform frames published by the robot
- *scan_topic:* The topic the slam toolbox will subscribe for LaserScan Messages. This is provided by a Lidar.
- *mode:* It is either set to *"mapping"* or *"localization"*, the first mode is used for map building and file update while the second mode is used for drift correction and localization.
- *loop_match_minimum_chain_size, loop_search_space_dimension:* When the robot is mapping in environments lacking unique features, the algorithm can misinterpret the current robot location. This will result in the robot pose being modified in a erroneous way. The values of the parameters can be reduced to make the algorithm focus its localization in a smaller area and remove the sudden jumps in pose.

## Base Navigation Launch file
The package contains a navigation launch file and accompanying configuration for the Dede robot. The navigation can be launched with the the following command with the appropriate parameter values:
```bash
ros2 launch dede nav2_slam.launch.py map:='/path/to/map/file' map_pose:="[0.0, 0.0, 0.0]"
```
**Note!** Is recommended to use an absolute path for the map file. The file path must not contain the file extension.

With the navigation nodes active, the user can issue goals through topics or the RViz2 interface.

## Understanding the Navigation Launch file
This subchapter will break down the launch file found at *"launch/nav2_slam.launch.py"*. The subchapter will start with the structure of the launch file, followed by the nodes and their role. Lastly the config files for the nodes will be addressed.

### 1) Launch file structure
The launch file contains commented code segments for ease of reference. The code is grouped in segments that have a specific role in the launch file. The following segments are present:
- *Configuration Paths:* This block has the purpose to get the file paths for the configuration files and folders for the launch file. As previously noted, a package can have a variable file path where it is installed and has the resource files.
- *Node Lifecycle Order:* The nodes executed by the launch file implement a lifecycle and need to be configured and activated in a specific order. This block contains a list of the nodes, ordered from the first to last.
- *Parameters Setup:* This segment contains the code for setting up launch file parameters, their defaults and short description. The following parameters are set:
    - *map:* A string, represents the path for the map file without the file extension. Is recommended to use absolute paths.
    - *map_pose:* A string representing a float array of size 3. The array values represent the X, Y and theta values given by odometry. The parameter needs to be set when resuming mapping from a file. 
    - *use_sim_time:* A boolean, defaulting to *"true"*, the parameters sets the nodes to either use the gazebo time stamp or the system time stamp. Set it to false when using a real robot.
- *Nodes:* This segment contains the nodes and groups that are executed by the launch file. The nodes are the localization, map publishing, navigation stack and lifecycle manager. A more detailed description of each node and their role can be found further down in this document.
- *Return Setup:* This segment creates a return object and appends to it all the actions and parameters before exiting the function. If an action is not added to the LaunchDescription object, it won't be executed by the launch file.

### 2) Launch File Nodes
The launch file contains all the nodes required for the autonomous navigation and can be broken down into three categories: localization, navigation stack and lifecycle management.

The localization is provided by the same node that created the map file while using a different configuration. The node's role is to provide the links between the robot and map, to correct for drift caused by wheel's slip and publish a map that can be used by the navigation stack.

The navigation stack in ROS2 is a complex system of nodes and plugins. The full documentation of the navigation stack can be found at [ROS2 Nav2](https://navigation.ros.org/). Each node has a configuration file associated with it. The configuration file are describing the node plugins and parameters. The node's roles are described in short as following:
 - *bt_navigator:* Manages the flow of operations for navigation, the node accepts a Behavior Tree file that describes what the navigation stack needs to do when encountering a specific event or enters in a specific state. The node manages from goal handing and passing to switching from standard navigation to recovery procedure in the case of failure.
 - *planner_server:* The node has the role of computing a path in order to achieve an objective. Based on the configuration, it loads a specific algorithm plugin. The target objectives are generally classified into two groups. The first group is "plan to goal", the most common algorithms have the objective to calculate the shortest path from a robot position to a target goal. The second group is "plan to cover given space", for example this group contains algorithms that can plan how the robot can drive efficiently to reach every spot in the given area. When handling these objectives, the node makes use of the map, environment and sensor data and outputs plans for the navigation stack.
 - *controller_server*: The node has the role of controlling the robot to execute an operation based on the path computed by a planner node or based on local data. In general, the algorithms use the sensors data and odometry data in order to compute the most efficient way to meet the required tasks or plan while avoiding going into an error state. Other example tasks are docking or boarding an elevator.
 - *recoveries_server*: The node has the role to autonomously handle error states. The robot can enter an error state due to a multitude of factors. Example factors are all paths to a goal are blocked, sudden noise in the sensor data for a few seconds, external object collided with the robot from outside the sensor detection space. The algorithms can have the tasks of cleaning and resetting the sensors, take over the drive system to withdraw from the dangerous situation, execute a motion that will be normally refused by the controller in order to avoid a more dangerous situation.
 - *waypoint_follower*: The node has the role of executing a series of tasks. An example of a tasks is iterating over an array of motion goals and passing a new one every time a goal finishes. A more complex tasks is driving to a target goal and executing a multi stage operation in that position.

 The navigation stack nodes need to be started in a specific sequence as they depend on each other. This is done by the lifecycle manager node. The node has a configuration list, and moves through it. When the navigation nodes start, they are in an inactive state, the lifecycle manager moves through the list and places the nodes in a configuration state. In this state, the nodes read the configuration files and set themselves up. Errors due to out of bounds parameters or wrong file path results in the lifecycle manager to stall the node configuration. At the end of this stage the nodes will advertize the topics and services they provide and use but no data is published or read from them. When all nodes finish configuring, the lifecycle manager will go thorough the list and attempt to activate the node in the given order. Only when a node is fully active, the manager will move to the next element in the list.

### 3) Configuration Files
The localization and navigation nodes have multiple launch files associated with them. The document will provide links to the official documentation and touch on the more important parameters in each file.

The *"slam_toolbox"* node has a single configuration file found at the location *"config/slam/localization.yaml"* See the previous chapter in the document for details on the configuration of the node. The main parameter of interest is the *"mode"* that is set to *"localization"*. Effectively this parameter makes the node only read the map file, it does not modify the file. In this mode. the node, will keep track of a smaller areas and consume less resources.

The navigation stack has an associated configuration file per node. Depending on the node, the files contain only parameters or parameters and plugins. The following configuration files are used:

- *bt_navigator*: The config file is located at *"config/nav2/bt_navigator.yaml*". The official documentation is found at [Nav2 Wiki - BT Navigator](https://navigation.ros.org/configuration/packages/configuring-bt-navigator.html). When creating the config file it is important to include in the *"plugin_lib_names"* parameter all the modules used in the behavior tree.

- *planner_server*: The config file is located at *"config/nav2/planner.yaml*". This package uses NavFn as the planner algorithm. The planner contains an associated costmap. The official documentation is found at [Nav2 Wiki - NavFn Planner](https://navigation.ros.org/configuration/packages/configuring-navfn.html) and at [Nav2 Wiki - Costmap 2D](https://navigation.ros.org/configuration/packages/configuring-costmaps.html). When creating a configuration file it is important to remember:
    - Set the planner to compute through unknown space when desired by setting the parameter *"allow_unknown"* to true.
    - The planner uses a global costmap
    - Set the inflation layer radius and the robot radius to values that match the robot in the costmap. A larger inflation layer radius will keep the robot further away from obstacles.

- *controller_server*: The config file is located at *"config/nav2/controller_dwb.yaml*". This package uses DWB as the controller algorithm. The controller contains an associated costmap. The official documentation is found at [Nav2 Wiki - DWB Controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html). When creating a configuration file is important to remember:
    - Set the kinematics of the robot to match the capabilities of the robot. Sometimes velocities lower the the maximum achievable are desired.
    - Set the goal tolerances to match the desired precision.
    - The algorithm can be set to put emphasis on avoiding costmap areas by setting the parameter *BaseObstacle.scale* to a high value.
    - The algorithm can be tuned to precisely follow the global path or loosely follow it based on the parameters *"PathAlign", "GoalAlign", "PathDist"* and *"GoalDist"*
    - The controller uses a local costmap

- *recoveries_server*: The config file is located at *"config/nav2/recoveries.yaml*. The official documentation is found at [Nav2 Wiki - Behavior](https://navigation.ros.org/configuration/packages/configuring-behavior-server.html). By default the recovery behaviors are very simple but more complex plugins can be added to the node.

- *waypoint_follower*: The config file is located at *"config/nav2/waypoint.yaml*. The official documentation is found at [Nav2 Wiki - Waypoint Follower](https://navigation.ros.org/configuration/packages/configuring-waypoint-follower.html). When creating a configuration file is important to remember:
    - Decide to skip a tasks when it fails and move to the next task by setting the parameter *"stop_on_failure"* to false.
    - By default the node will pause fo a short duration between navigation goals, this behavior can be changed by choosing a different plugin.
    - The node has build-in plugins for waiting a set amount of time when reaching a goal, taking a photo when reaching a goal or waiting for a flag message on a topic before moving to the next goal.