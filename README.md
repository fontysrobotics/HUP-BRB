# Human Understandable Priority-Based Robot Behaviour

The source code for all the packages can be found in the `src` folder. More specifically:

- `src/path_projection` contains the logic for displaying the paths of the robots onto the floor.
- `src/dede` is the [Nox dede](https://github.com/NOX-Robotics/dede) repository with some modifications to the configurations, worlds and launch files.
- `src/hupbrb` contains all the custom priority based navigation nodes.
- `src/hupbrb_msgs` contains custom `.msg` and `.srv` files used as part of the project.
- `src/priority_based_robot_costmap_plugin` creates a costmap plugin that must be used in the `Navigation2` stack to allow priority-based navigation.

Images and videos with the project can be found among the [Media Assets](https://stichtingfontys.sharepoint.com/:f:/s/DevelopersHUP-BRB/EsbArXFdLzFIiJ8f-M7v_2EBC2GRPdnjV8R0IPIHCm0GfQ?e=1zIK9L) 
