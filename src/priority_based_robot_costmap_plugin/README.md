# The costmap plugin

This is a [Naviagtion 2 Costmap Plugin](https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html) 
that allows you to create a circular no-go zone at any point in a robots global or local costmap by sending
a message to the topic /\<robot namespace\>/global_costmap/collision_point.

