---
layout: default
title:  "The Fiducial Follow App"
permalink: fiducial_follow_app
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/) - - -&uarr;[up](ix_programming)

## The Fiducial_Follow App

With this app, the robot will follow a given marker, keeping a certain distance from it as it moves and turns.

The Ubiquity Robotics robots use a number of fiducial markers (aruco markers, illustrated below).  The fiducial_follow app takes advantage of aruco_detect (a ROS node that is part of the Magni package) to detect aruco markers in the image feed from a camera.  The characteristics of the images of the markers enable the robot to compute its location and theirs.

<img src="https://ubiquityrobotics.github.io/learn/assets/fiducial.png" />

With navigation running aruco_detect continuously searches its field of view for aruco markers. When a marker is found a `fiducial_transforms` message is published with an array of all the markers that have been found.

The fiducial_follow module subscribes to these messages.  However, it is interested in only one fiducial: the one it is following (aruco marker number 49 by default).

If the target marker is found in the array, movement commands are issued to the robot to make it move towards the fiducial.

The code for the fiducial_follow app may be found [here](https://github.com/UbiquityRobotics/demos/blob/master/fiducial_follow/nodes/follow.py).

#### Nodes
The single program follow.py

#### Parameters
target_fiducial: the fiducial we are following. The default is fid_49.

#### Publications (i.e., output)

cmd_vel(geometry_msgs/Twist): Commands to move the robot.

#### Subscriptions (i.e., input)

fiducial_transforms:(fiducial_msgs/FiducialTransformArray)

#### other
uses the fiducials package https://github.com/UbiquityRobotics/fiducials
