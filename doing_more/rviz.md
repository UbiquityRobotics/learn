---
layout: default
title:  "Visualize with RViz"
permalink: rviz
---

# Visualize with rviz

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

Rviz (ROS visualization) is a 3D visualizer for displaying sensor data and state information from ROS. It can show the robot in space as well as graph practically any robot parameter.

There is a nice short intro to rviz video at [this location](http://wiki.ros.org/rviz).

<!-- To start rviz, type on the workstation command line:

  `rviz`
-->
On the workstation, you can type:

  ```roslaunch magni_viz view_nav.launch``` when navigation is running  
  or  
  ```roslaunch magni_viz view_robot.launch``` when it is not.

![rviz](https://ubiquityrobotics.github.io/learn/assets/rviz_image.png)

Using rviz, you can move the robot:  
Click on "2D Nav Goal" in the menu bar.  
Click again in the black screen area.  The robot will ......

![rviz](https://ubiquityrobotics.github.io/learn/assets/rviz with nav.mp4)

#### &larr;[back](workstation_setup)- - - - - - - - - - [next](fiducials)&rarr;
