---
layout: default
title:  "The Fiducial Follow App"
permalink: fiducial_follow_app
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

![Fiducial Markers](two_fiducials.png)

The Ubiquity Robotics robots use a number of fiducial markers of known size (illustrated below).  The fiducial_follow demo uses aruco_detect (a ROS node that is part of the Magni package) to detect fiducials in the image feed from a camera.  The characteristics of the images of the fiducial markers enable the robot to compute its location.

<img src="https://ubiquityrobotics.github.io/learn/assets/fiducial.png" />

This is an aruco marker. Like a QR code, it contains an identifier encoded in the pattern of the marker.  With fiducial follow running the robot continuously searches its field of view for a specific aruco marker (number 49 by default). Because the marker is a pre-set size and has a pre-set orientation the robot can determine how far away it is and its own orientation!


If the target fiducial is detected, movement commands are issued to the robot to make it move towards the fiducial.

Parameters

Nodes
The single program follow.py

target_fiducial: the fiducial we are following. The default is fid_49.

Publications (i.e., output)

cmd_vel(geometry_msgs/Twist): Commands to move the robot.

Subscriptions (i.e., input)

fiducial_transforms:(fiducial_msgs/FiducialTransformArray)

uses the fiducials package https://github.com/UbiquityRobotics/fiducials
