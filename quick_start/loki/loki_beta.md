---
layout: default
title:  "LOKI Beta startup"
permalink: loki_beta
---

## Getting Started with Loki for Beta Testers

Loki is a ROS controlled differstial drive robot designed for educational purposes by Ubiquity Robotics.
As such, it has code that is in many cases identical to our general purpose Magni Platform. A program written to navigate a
one meter square for Loki wil navigate an Identical one meter square for Magni

To get started, one needs to attach a Raspberry Pi 3 with an SD card image supplied by Ubiquity Robotics.
It is possible to intall the PI incorrectly which will result in a "fried Pi". Be careful!

![Fried Pi](loki_rpifatal.jpg)

The Rpi 3 pulls over 3 amps, so besure you supply at least 2.5 amps (no camera) or 3 amps if you are using an Raspicam.
using and USB accessories is not recommnded unless addional power is provided.

![Loki Top View](loki_top1.jpg)

Make sure (For rev F boards) that the J2 and J5 (circled) jumpers are installed. The robot is powered by either a switch or a jumper. When activated, initially all the LEDS on the robot should go green. If you then turn the wheels, they should turn on and off. If this doesn't happen, the board needs to have its firmware re-installed.

Up to 16 sonar sensors can optionally be installed for obstacle avoidance. These are not intended for gmapping. As of this writing, the optional raspicam is set to use ceiing mounted fiducial markers, and this may be changed for the final release.
Also, we haven't finished the final SD card image for Loki, so for now the networking and connecting sections for our Magni
robot are the best source of instructions.

## Boot up

Assuming you can connect to the robot via ssh, you can test the robot by seeing if you can teleop.
First determine if any nodes are running by typing:

rostopic list

if not, you can start the base controller by:

cd catkin_ws/src/ubiquity_launches/bin/
./loki_base

To drive the robot, ssh in a separate window and type:

rosrun teleop_twist_keyboard teleop_twist_keyboard.py

## Rviz

To run Rviz on your laptop,

you need to make sure your networking between the laptop and robot is working correctly and the clocks are synchronized.

on the robot make sure ROS_IP  and ROS_MASTER_URI are set to IP number, not localhost. on the laptop make sure the ROS_MASTER_URI matches the robot. The catkin_ws/src/ubiquity_launches files need to be installed on both systems.

cd ~/catkin_ws/src/ubiquity_launches/bin/
./loki_rviz_sonar

will bring up rviz and show any sonars if present.

If you've gotten this far, pour yourself an adult beverage, do your victory dance, and be proud of yourself!


![Loki RVIZ](loki_rviz.jpg)
