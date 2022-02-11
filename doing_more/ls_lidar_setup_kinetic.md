---
layout: default
title:  "Setting up magni-default lidar Kinetic"
permalink: default_lidar_setup
---
# Setting up ubiquity robotics default lidar - Kinetic

#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/)

Ubuntu 16.04

ROS Kinetic

**This tutorial works only with magni_robot branch: indigo-devel (which should work up to and including Kinetic)**

This document will be deprecated soon as we are moving to newer versions of ROS. See Noetic instructions [here](ls_lidar_setup_noetic.md).

In this document it is described how to set up the LS301 lidar on the Magni.

It is assumed that the LS lidar is already mounted and the network is setup following tutorial [Setting up magni-default lidar](/learn/doing_more/ls_lidar_setup_common.md)

# Default lidar extrinsics

The lidar position that is going to be loaded when robot starts up can be set in  `/etc/ubiquity/robot.yaml`:

    sudo nano /etc/ubiquity/robot.yaml

which should, among other things also contain:

    lidar_installed: True


# Example adding a custom lidar location

If the lidar is set in a different position, readjust its coordinates in the [urdf files](https://github.com/UbiquityRobotics/magni_robot/blob/4c43300e4fc08a37f5206c9db85a6e11105f91d6/magni_description/urdf/magni.urdf.xacro#L205). Example:

    <xacro:if value="${lidar_installed}">
        <xacro:hokuyo_lidar name="laser" connected_to="base_link">
        <origin xyz="0 0.1 0.4" rpy="0 0 0"/> <!--change lidar pose here-->
        </xacro:hokuyo_lidar>
    </xacro:if>

***