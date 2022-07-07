# HUP-BRB
Human Understandable Priority Based Robot Behavior. This repository is part of the Holland Robotics &amp; Logistiek project.

What we did can be found here: https://youtu.be/TVuYbK3PcoQ

All the Ros code is in master. Of example the priority controller and the navigation plugin!!!

In this branch of the git hub repository, you can find the code and the CAD files for the picolaser project. The cad files are in kicad and the code is in written in C. if you would like to continue on this project the easiest steps to take are to be on Linux and follow the install guide on the raspberry pi pico sdk. The install guide can be found with this URL:

https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf

Read page 7 of this manual. The code is not optimal. It runs on only one of two cores so that should be fixed. There is a state machine that handles the communication with the other raspberry pi 4 and does the calculations then it is displayed. 

If would would like to continue with the hardware than I would like to refer you to the picoLaser kicad files. A recommendation for the hardware is that the motors that we used don’t have the resolution nor the control that we would have liked, so you could look into a feedback system and closed loop control or change the motors completely. Also have a look at the placement of the lens on the top of the laser. The laser needs to be focused with a better solution than we have done.   
