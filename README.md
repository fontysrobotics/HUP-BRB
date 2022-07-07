# Human Understandable Priority-Based Robot Behaviour

Images and videos with the project can be found among the [Media Assets](https://stichtingfontys.sharepoint.com/:f:/s/DevelopersHUP-BRB/EsbArXFdLzFIiJ8f-M7v_2EBC2GRPdnjV8R0IPIHCm0GfQ?e=1zIK9L) 

more information can be found in the `doc1 directory.

The source code for all the packages can be found in the `src` folder. More specifically:

- `src/path_projection` contains the logic for displaying the paths of the robots onto the floor.
- `src/dede` is the [Nox dede](https://github.com/NOX-Robotics/dede) repository with some modifications to the configurations, worlds and launch files.
- `src/hupbrb` contains all the custom priority based navigation nodes.
- `src/hupbrb_msgs` contains custom `.msg` and `.srv` files used as part of the project.
- `src/priority_based_robot_costmap_plugin` creates a [costmap plugin](https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html) that must be used in the `Navigation2` stack to allow priority-based navigation.

All of these packages have their own README files with more detail.

## Recommendations

Recomended directions for improvement include :
- Fix and finish the pico laser projector (on the `main` branch).
- Use actual turtlebots or dede robots with this project.
- Test and support more scenarios and more robots simulatnously.
- Add the concept of `importance` that acts as a tiebreaker when the priority of two robots is equal but ie must yield to the other.
