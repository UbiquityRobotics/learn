---
layout: default
permalink: read_this_first
---
## Read This First
#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

### Communications

As the robot is delivered it of course has no connection to your local network. Because of this, the robot has its own network (called an access point or AP) that enables you to connect to it directly, without connecting to your local network. For example, you should be able to drive the robot with our Android Robot commander app. You also can connect directly to the robot from a workstation, to run ROS commands such as keyboard teleoperation. However, in AP mode, the robot cannot access the rest of the Internet.

Connecting to your local network is required for programming the robot,
and is covered in a separate section.

### The Workstation

You need a workstation to control the robot. In this manual it is assumed that your workstation, whether a laptop, desktop, or virtual machine, is running Ubuntu 16.04 LTS.  ("16.04" stands for "2016, April" and "LTS" denotes "Long Term Support" which means 5 years.)  Ubiquity Robotics supports only this release.

**The workstation must have WiFi capability.**  Most laptops do, many desktops don't.

Not everyone has Ubuntu Linux installed on their machine, so we've created a virtual machine (VM) as a downloadable VirtualBox image. This is a system that allows most any computer to pretend that it is a Ubuntu Linux machine. Our VM is preconfigured with Ubuntu, ROS (Robot Operating System) and Ubiquity Robotics' workstation software.

Note that there may be three systems involved in this configuration:
* ROS under Ubuntu on the robot.
* ROS under Ubuntu 16.04 on the workstation hardware, OR
  * ROS under Ubuntu 16.04 in a VM under VirtualBox.
* If using a VM, Windows or MacOS hosting VirtualBox.

Instructions for workstation setup are given below, as they are needed.
