---
layout: default
title:  "Setting up magni-default lidar"
permalink: default_lidar_setup
---
# Setting up ubiquity robotics default lidar

#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/)

In this document it is described how to set up the LS lidar on the Magni.


# Mounting the lidar
Top plate comes with wholes ready for the lidar to be mounted. (if the wholes arent there use a double sided tape)


# Setup to work with RaspberryPi

## Location of the lidar on the robot

If you mounted the lidar on the top plate where the holes are, you dont need to worry about the urdf file, that position is set by default.

Robot must know the exact pose of where the LIDAR is mounted. This can be setup inside the magni urdf file 
https://github.com/UbiquityRobotics/magni_robot/blob/3775b000f5b9ca463a7ac4dfb4b1423b8d67f40d/magni_description/urdf/magni.urdf.xacro#L207



## Network
The lidar should come pre-configured with static IP: 192.168.42.222 and only answering to requests comming from IP 192.168.42.125, so we need to configure that on the RPI.

A static interface needs to be set on RPIs lan port by editing the interfaces file

    sudo nano /etc/network/interfaces

we need the eth0 interface to have a static address of 192.168.42.125/24. After the necesseary additions are made, the interfaces file looks something like this:

    # interfaces(5) file used by ifup(8) and ifdown(8)
    # Include files from /etc/network/interfaces.d:
    source-directory /etc/network/interfaces.d

    # The loopback network interface
    auto lo
    iface lo inet loopback

    auto eth0
    iface eth0 inet static
    address 192.168.42.1/24
    gateway 0.0.0.0
    dns-nameservers 8.8.8.8




# Running

<!-- TODO add compiling and running once we have the repo with the URLIDAR setup -->
