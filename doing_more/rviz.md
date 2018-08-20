---
layout: default
title:  "Visualize with RViz"
permalink: rviz
---

# Visualize with rviz

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

Rviz (ROS visualization) is a 3D visualizer for displaying sensor data and state information from ROS. It can show the robot in space as well as graph practically any robot parameter.

There is a nice short introductory rviz video at [this location](http://wiki.ros.org/rviz).

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
Click again in the black screen area and indicate what the robot is to do.

Here is a [short video](https://ubiquityrobotics.github.io/learn/assets/rviz_with_nav.mp4) showing the robot navigating with our setup. As you can see this was recorded on a workstation operating the robot. With the left side being a window that's logged in to the robot and the right window being a local terminal on the workstation. This video assumes that a standard robot has been set up according to the instructions - its probably obvious that the name of the robot has been changed, as per the instructions, in this case to 'frank'.

In the right hand window you see the steps that are carried out on the workstation including loading rviz etc. The setup is all standard; if you want to make sure that the launch and configuration files are on your workstation you can make sure everything is installed with

```sudo apt install ros-kinetic-magni-viz```  
or  
```sudo apt install ros-kinetic-magni-robot```  
on the workstation.

Once RViz is loaded you can see that the user issues a go to goal target in RViz using a few mouse clicks. She first clicks **2D Nav Goal** in the menu bar, then clicks on the desired pose. The arrow that is on the field of view is the desired location and orientation.

The system determines its location by looking at the fiducials in the environment, then the robot turns and drives to the target location.


#### &larr;[back](workstation_setup)- - - - - - - - - - [next](fiducials)&rarr;
