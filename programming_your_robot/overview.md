---
layout: default
title:  "Overview"
permalink: overview
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/) - - -&uarr;[up](ix_programming)

## Overview

As many of you will know already, ROS operates with a set of nodes that are able to publish and/or subscribe to messages that are passed between the nodes by the core ROS process. The task of programming the robot to do something we want is the task of writing a node that subscribes to the data we want to use and publishes the action we want the robot to take.

In addition to the addition to the nodes that come with a standard ROS install there are 3 types of node that Ubiquity Robotics provides specifically with our robots.

1) Infrastructure Nodes  - Nodes that enable the hardware on our robot to function. A little like a hardware driver. These either gather data from sensors on the robot and publish it (for example our raspicam node publishes photographic camera data), or they subscribe to topics that cause the robot to move.

2) Capability Node - Nodes that provide an important capability that our robot uses. These nodes are not tied to specific hardware, and need more software to actually cause the robot to act. Typically these nodes convert one type of data to another.- For example our *dnn_detect* node takes photographic camera data and publishes the position of all the objects it can see in the image.

3) Action Nodes - Nodes that actually allow the robot to do something. Right now these are our demos. For example our *dnn_rotate* demo takes the data from the *dnn_detect* node and will turn the robot towards any object that you specify.

Each of the nodes that Ubiquity Robotics has developed resides in a GitHub repository (listed below). A description of each of these nodes, the topics it subscribes to and publishes as well as installation instructions can be found there:

### Infrastructure Nodes  
!https://github.com/UbiquityRobotics/ubiquity_motor    -Drives the motors
!(https://github.com/UbiquityRobotics/raspicam_node)   -Drives the camera
https://github.com/UbiquityRobotics/pi_sonar              -Drives the sonar array
https://github.com/UbiquityRobotics/bt_joystick            -Drives the included bluetooth joystick

### Capability Node
https://github.com/UbiquityRobotics/fiducials               -Detects and publishes fiducial position
https://github.com/UbiquityRobotics/move_basic        -Enables simple navigation
https://github.com/UbiquityRobotics/dnn_detect           -Publishes what it sees on camera and where it is in the field of view

### Action Nodes
https://github.com/UbiquityRobotics/demos    The set of demos that ubiquity robotics includes as a starting point for you to write your own programs.

By using a combination of nodes it is possible to get the robot to adopt any one of a number of behaviors. Our demos are used to show how the robot can operate but also form an example or a starting point for you to write your own. To see how this can be done, see our [fiducial follow](programming_your_robot/fiducial_follow_app.md) program.

### Launch Files

The launch files bring up the robot with the software needed for different configurations.  They are described in [this GitHub repo](https://github.com/UbiquityRobotics/magni_robot/blob/indigo-devel/README.md).

### Learning ROS

ROS has a wide user base and there are lots of resources to learn how to program with it.
#### ROS Texts

Some of the texts that are widely acknowledged as excellent include:

* A Gentle introduction to ROS - Free online textbook style guide

* Programming Robots with ROS - Was written by many of the original developers of ROS

* ROS Tutorials - An online resource maintained by The Open Source Robotics Foundation

#### Online lecture courses

In addition there are a number of lecture series on ROS some that can be found online include:

* Programming for Robotics - ROS From ETH Zurich

* Introduction to Robotics From Bar-Ilan University

### Starter Code

For those of you who want to dive straight in to code - we’ve provided a couple of example projects. They also provide a lot of the basic infrastructure you need and create a starting point.
#### Fiducial Follow

With this code the robot will follow a fiducial marker. If you attach the marker to your clothing the robot will follow you. The app itself contains the bindings that are needed to do useful work with fiducials.
#### Move Demo

Move demo This code simply moves the robot. It includes all the bindings to external services that both navigate and drive the robot.
#### Good Luck!!

If you have issues consider posting your question to forum.ubiquityrobotics.com. Other than that - we hope you have much success developing your applications with Magni.
