---
layout: default
title:  "Fiducial-Based Localization"
permalink: fiducials
---
# Fiducial-Based Localization

^^[top](main_menu)^^ - - - ^[up](ix_doing_more)^ - - - &larr;[back](rviz)- - - - - - - - - - [next](sensors)&rarr;

This is one of the ways in which a robot's position in the `world` (localization) can
be determined.  For a discussion of other approaches, see the
[Navigation and Localization Overview](../overview/overview.md).

This document assumes that you are running the software on a Ubiquity
Robotics robot base, using the supplied Raspberry Pi camera. It also assumes that you have a workstation with ROS installed, which is connected to a network in common with the robot. You will need a printer, too.

## Basic Concept

The Ubiquity Robotics localization system uses a number of fiducial markers of known size (illustrated below).  Detection of the markers
is done by using the robot's camera.  The characteristics of the images of the fiducial markers enable the robot to compute its location.

![Fiducial Markers](two_fiducials.png)

## Print Some Fiducials

A PDF file containing a range of fiducial markers can be generated on your workstation with a command such as:
```rosrun aruco_detect create_markers.py 100 112 fiducials.pdf```

The *100* and *112* specify the starting and ending numerical IDs of the
markers.  It is not important what these are (we recommend starting at 100), but it is important that each marker has a unique ID.  Print the PDF file to produce the fiducials.  
## Mount the fiducials
Affix the fiducials (in any order) to any convenient
surface, such as a ceiling, where they will be viewed by the robot's camera.
They need to be at a sufficient spacing so that more than one is visible at a time, if the robot is at least 6 feet away from the wall or ceiling holding the fiducials. Below you will see how to test that the spacing is OK.

## Running the Software
Login to the robot from your workstation using ssh.  Then execute the following command on the robot to launch the detection and SLAM nodes:

```roslaunch magni_nav aruco.launch```

It should be run for the first time with at least one marker visible.
A map (this is a file of fiducial poses) is created such that the current position of the robot is the origin.

## Using rviz to Monitor Map Creation

Use The following command on your workstation to run the
[robot visualization tool](http://wiki.ros.org/rviz), rviz.
#### [Link here to the tutorial where we explain ROS_MASTER_URI]???

```roslaunch fiducial_slam fiducial_rviz.launch```

This will produce a display as shown below.  The bottom left pane shows the
current camera view.  This is useful for determining if the fiducial density
is sufficient.  The right-hand pane shows the map of fiducials as it is being
built. Red cubes represent fiducials that are currently in view of the camera.
Green cubes represent fiducials that are in the map, but not currently
in the view of the camera. The blue lines show connected pairs of fiducials
that have been observed in the camera view at the same time.  The robustness
of the map is increased by having a high degree of connectivity between the
fiducials.

![Visualizing with rviz](fiducial_rviz.png)
#### &larr;[back](rviz)- - - - - - - - - - [next](sensors)&rarr;
