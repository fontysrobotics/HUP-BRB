# HUP-BRB
Human Understandable Priority Based Robot Behavior. This repository is part of the Holland Robotics &amp; Logistiek project.

The path projection package is created to display paths of the robots on the warehouse floor. It runs on ROS2 and is coded in python using OpenCV.
In our path projection folder (HUP-BRB/src/path_projection) we have multiple folders:
- bags
- path_projection
- resource
- test

In this package we are only really interested in the bags and the path_projection folder. 

The path_projection folder contain python code for dispaying the path of 1 robot in the path_projection_node.py file. It also contains python code
for displaying multiple paths from multiple robots on the floor. 

The bags folder contains topic messages that we can send to test our code. 

**Multi_path_projection_node.py**
file description:

This file contains a path_projection class that is a ROS node. This node subscribes to the robot_info topic. This topic is being published in all active robots
and the path projection receives this information and saves the robot data in a dictionary (type of python list) with its own name and a unique color. When saving this information we also create a subscriber for the path of the robot. This path data is also saved in the dictionary using the save_plan function (callback function for the path subscription).
```
self.robotinformation_subscription = self.create_subscription(RobotInformation, '/robot_info', self.robot_info_callback, 10)
```

This file contains a timer as well which checks if a robot has disconnected every 4 seconds by checking the time of the last message send by that robot.
If it is more then 4 seconds the path projection will forget this robot and stop displaying the path.
```
self.alive_check_timer = self.create_timer(4, self.alive_check)
```

This file also contains a timer that has a callback_function which runs every 2 seconds (self.drawer_timer) which draws the paths of the robots.
```
self.drawer_timer = self.create_timer(2, self.draw_lines)
```

When self.draw_lines() is called the code loops through all active robots with a path and saves the path coördinates in the list Points[].
These points are then converted to a np.array and displayed on a black frame. 

```
cv2.polylines(frame, [out], isClosed = False, color = color, thickness = THICKNESS) #1

cv2.line(frame, (dest_x - CROSS_SIDE, dest_y - CROSS_SIDE), (dest_x + CROSS_SIDE, dest_y + CROSS_SIDE), color=(255,)*3, thickness=THICKNESS) #2
cv2.line(frame, (dest_x - CROSS_SIDE, dest_y + CROSS_SIDE), (dest_x + CROSS_SIDE, dest_y - CROSS_SIDE), color=(255,)*3, thickness=THICKNESS) #3

cv2.circle(frame, (start_x, start_y), circle_radius, self.robots[x]["color"], THICKNESS) #4
```
The path lines are displayed by #1 with their own color and a predefined thickness.
The cross of the destonation is displayed by #2 and #3, each one line of the X.
The circle of the robot location is drawn with #4. 

The final result should look something like this:
![alt text](https://github.com/fontysrobotics/HUP-BRB/blob/multi_path_projection/src/path_projection/MicrosoftTeams-image%20(3).png)

