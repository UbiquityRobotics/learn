---
layout: default
title:  "Setting up ur50 lidar Kinetic"
permalink: ur50_lidar_setup_kinetic
---
# Setting up ubiquity robotics default lidar - Kinetic

#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/)

Ubuntu 16.04

ROS Kinetic

**This tutorial works only with magni_robot branch: indigo-devel (which should work up to and including Kinetic)**

This document will be deprecated soon as we are moving to newer versions of ROS. See Noetic instructions [here](ur50_lidar_setup_noetic.md).

In this document it is described how to set up the ur50 lidar on the Magni.

It is assumed that the ur50 lidar is already mounted and the network is setup following tutorial [Setting up magni-default lidar](/learn/doing_more/ur50_lidar_setup_common.md)



# Setup to work with RaspberryPi

## Network
The lidar should come pre-configured with static IP: 192.168.42.222 and only answering to requests coming from IP 192.168.42.125, so we need to configure that on the RPI.

A static interface needs to be set on RPIs lan port by editing the interfaces file

    sudo nano /etc/network/interfaces

we need the eth0 interface to have a static address of 192.168.42.125/24:

    auto eth0
	iface eth0 inet static
	address 192.168.42.125/24 #this IP may depend on your lidar settings
	gateway 0.0.0.0
	dns-nameservers 8.8.8.8

# Compiling

    cd ~/catkin_ws/src
    git clone https://github.com/LS-Technical-Supporter/LS-LIDAR-N301ROS.git
    cd ~/catkin_ws/
    catkin_make
    

# Running

    cd ~/catkin_ws/
    source devel/setup.bash
    roslaunch lslidar_n301_decoder lslidar_n301_config.launch device_IP:=192.168.42.222
    

# Default lidar extrinsics

The lidar position that is going to be loaded when robot starts up can be set in  `/etc/ubiquity/robot.yaml`:

    sudo nano /etc/ubiquity/robot.yaml

which should, among other things also contain:

    lidar_installed: True


# Example adding a custom lidar location

If the lidar is set in a different position, readjust its coordinates in the [urdf files](https://github.com/UbiquityRobotics/magni_robot/blob/4c43300e4fc08a37f5206c9db85a6e11105f91d6/magni_description/urdf/magni.urdf.xacro#L205). Example:

    <origin xyz="0 0.1 0.4" rpy="0 0 0"/> <!--change lidar pose here-->
