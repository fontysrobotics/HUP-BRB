# Simulation with Gazebo
## The robot model
The package contains a gazebo robot model matching the real mobile platform. The model contains:
- Collision, inertial and visual meshes.
- Simulated differential drive module:
    - Publishes odometry data on the **/odom** topic
    - Subscribes for velocity commands on the **/cmd_vel** topic
    - Simulated wheel friction and motor torque
    - Individual wheel velocity and pose are published on **/joint_states**
- Simulated Lidar module:
    - Simulated lidar with a range of 10 meters and update rate of 10Hz
    - Publishes LaserScan data on the **/scan** topic
- Matching URDF for robot state publishing

The provided topics allow basic driving operations, mapping and navigation in a simulated environment.

## Running the robot in the provided simulated world
The package contains a simple gazebo world file with a simulated robot placed in it. Running the simulation can be done using the following command:
```bash
ros2 ros2 launch dede simulation.launch.py
```
Gazebo will be launched in verbose mode and a robot state will be published. RViz2 can be utilized to monitor the simulated robot topics.

## Running the robot in a custom simulated world
The robot can be placed in a different virtual environment. For a simple world a single static mesh file is enough but more complex worlds can use textures and dynamic objects.

### The STL file
The various entities in the world can be created using 3D primitives or mesh files. The entities represent rooms, objects or robots.

**Note!** The format used by gazebo for mesh files is STL.

When creating a mesh file for a simulated object, keep in mind the following:
- When exporting the STL file, set the axis as: Z up, X Forward
- The robot has a diameter of 0.3 Meters, any path that the robot is intended to traverse must be larger than the robot diameter
- Add thickness to the room's walls and/or objects, we recommended a value of at least 5cm. The map data is by default using cells of 5cm, thus, thinner walls might cause inconsistent behavior
- The lidar within the robot is positioned at height of 18cm relative to the ground, the robot can collide with objects lower then the lidar scan plane. This is because said objects are out of the lidar's scan plane
- Mapping and localization software uses features of the environment for drift correction. Avoid creating empty rooms or long hallways without markers.
- When creating identical rooms, it is recommended to add objects inside to create enough distinctive features. Identical rooms will cause localization software to inaccurately determine the robots location
- The lidar range is of 10 Meters, walls outside of this range will not be detected/mapped.

### The simulated object model
In ROS2, each simulated object is placed in a specific folder structure. Relative to the root of the package a *"models"* folder holds the individual world and robot models.

**Note:** The Gazebo ROS2 references the models based on the folder name rather then the package name, when creating a robot or world model give it an unique name or prefix it with the package name.

**Note:** The ROS package doesn't export the models by default. Under the *"package.xml"* the export is created by appending to the the *"export"* tag the following:
```xml
<gazebo_ros gazebo_model_path="${prefix}/models"/>
```

Each model folder contains the following:
- *meshes* : A folder containing 3D mesh files for visual representations and collision calculations
- *model.config* : The config file of the model in an XML format containing the name, short description and author of the model
- *model.sdf* : The description file for a simulated world or robot. This description is elaborated on the next paragraphs


When creating a simple room or object only the following fields need to be provided in the model descriptor file:
- *Pose:* Set to no translation or rotation: 0.0 0.0 0.0 0.0 0.0 0.0
- *Static:* The field sets the model as an immovable entity while providing collision.
- *Main Link Group*:
    - *Inertia*:
        - Required field
        - The value can be computed by various software
        - For static entity the field doesn't have any effect
    - *Collision:*
        - Provides collision for the sensor rays and robot models
        - Set to use a simple 3D mesh
        - The friction valued can be set based on the desired behavior
    - *Visual:*
        - Provides the visual representation of the world.

**Note!** The model description file specification can be found at [SDFormat](http://sdformat.org/spec).

**Note!** For reference the provided model description can be found under: *models/dede_maze/model.sdf*

### The world file
The world file combines multiple simulation model files representing light sources, static objects, dynamic objects and robots. Each individual object is given a pose relative to the world origin. The file can contain custom properties for the scene and physics engine. The world files are places in *"worlds"* folder.

A simple world file contains the following:
- *Sun entity:* Provides a light source
- *Ground plane:* Provides a plane where the dynamic objects are resting on
- *An open or enclosed room:* Provides an environment where the robot can drive around and scan
- *Simulated robot*

**Note:** For reference the provided world file can be found under: *world/base_maze.model*

The world file can be passed directly to a Gazebo server instance executed using *ros2 run* command or loaded by a launch files.

**Note:** For reference the provided launch file file can be found under: *launch/simulation.launch.py*