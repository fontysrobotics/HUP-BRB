# The priority based navigation

This package contains nodes that make navigation decissions.

These are the equal-priority scenarios and solutions that we support:
![Screenshot 2022-07-07 150511](https://user-images.githubusercontent.com/26307463/177780605-be832093-cc37-4dde-831e-3e76b4cff2bd.png)

And these are the scenarios if the priority differs:
![image](https://user-images.githubusercontent.com/26307463/177780831-0c9355df-a0e9-47ae-8b87-147a793d5302.png)
Note in case two we don't swerve always to the right, but instead towards the high priority robot, this is safer.

This is how we move the robot away from the collision point (indicated with a purple dot). Note that the costmap is not centered on the the collision point, but insteaed is shifted to the left of the robot. This is achieved by translating it into the robot frame, increasing the y coordinate and translating back. This happenes in the PriorityController.

![image](https://user-images.githubusercontent.com/26307463/177781184-92fa6350-dd93-4f94-bd74-f3d8a53b28b7.png)

