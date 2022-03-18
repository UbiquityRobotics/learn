---
layout: default
title:  "Setting up ur50 lidar Noetic"
permalink: ur50_lidar_setup
---
# Setting up ubiquity robotics default lidar

#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/)

Ubuntu 20.04

ROS Noetic

**This tutorial works only with magni_robot branch: [noetic-devel](https://github.com/UbiquityRobotics/magni_robot/tree/noetic-devel/)**

In this document it is described how to set up the ur50 lidar on the Magni.

It is assumed that the ur50 lidar is already mounted and the network is setup following tutorial [Setting up magni-default lidar](/learn/doing_more/ur50_lidar_setup_common.md)

# Setup to work with RaspberryPi

## Network
The lidar should come pre-configured with static IP: 192.168.42.222 and only answering to requests coming from IP 192.168.42.125, so we need to configure that on the RPI.

    sudo nano /etc/systemd/network/10-eth-dhcp.network

and replace everything in there with 

    [Match]
    Name=eth*

    Address=192.168.42.125/24
    [Network]

and then `sudo reboot` or `sudo systemctl restart systemd-networkd`. After that the IP on eth interface should always be set to 192.168.42.125 when any device is plugged into ethernet port. You can check that with

    ubuntu@pi-focal ~$ ifconfig eth0
    eth0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
            inet 192.168.42.125  netmask 255.255.255.0  broadcast 192.168.42.255
            inet6 fe80::e65f:1ff:fe33:ef3f  prefixlen 64  scopeid 0x20<link>
            ether e4:5f:01:33:ef:3f  txqueuelen 1000  (Ethernet)
            RX packets 243450  bytes 302746896 (302.7 MB)
            RX errors 0  dropped 0  overruns 0  frame 0
            TX packets 5618  bytes 2593522 (2.5 MB)
            TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0



# Compiling

    cd ~/catkin_ws/src
    git clone https://github.com/UbiquityRobotics/ls_lidar_driver
    cd ~/catkin_ws/
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
    

# Running

    cd ~/catkin_ws/
    source devel/setup.bash
    roslaunch lslidar_n301_decoder lslidar_n301_config.launch device_IP:=192.168.42.222

# Default lidar extrinsics

The system is setup so that lidar extrinsics can be set in two places with following priorities:
    
1st priority in `~/.ros/extrinsics/lidar_extrinsics_<POSITION>.yaml`
    
2nd priority in package `magni_description/extrinsics/lidar_extrinsics_<POSITION>.yaml`

Where `<POSITION>` is taken from `~/.ros/ubiquity/robot.yaml`, parameter `lidar_position`.

Example: 

This means that if in `~/.ros/ubiquity/robot.yaml` the parameter `lidar_position` is set to `top_plate`, like it is by default ([see default robot.yaml settings](https://github.com/UbiquityRobotics/magni_robot/blob/noetic-devel/magni_bringup/config/default_robot.yaml)), the system will first search for `~/.ros/extrinsics/lidar_extrinsics_top_plate.yaml`. If that does not exists it will search for `magni_description/extrinsics/lidar_extrinsics_top_plate.yaml`. If none of those can be found the lidar extrinsics will not be loaded.

The difference between the two locations is that inside `~/.ros/extrinsics/` the extrinsics files will be robot-specific and will not be overwritten by git pulls or automatic updates - its meant for users to store custom extrinsics files. Where as `magni_description/extrinsics/` location will be overwriten with both git pulls and updates - its meant as a storage of widely used default extrinsics.


# Example Adding a custom lidar location

You can mount the lidar in a custom location on the robot, but then you need to indicate the extrinsics (x, y, z, roll, pitch, yaw) of the custom location in robots configuration files. That can be easily done by:

1.) adding an additional extrinsics configuration into `~/.ros/extrinsics/lidar_extrinsics_<POSITION>.yaml`, where `<POSITION>` is an arbitrary name for the new configuration. Lets say a new lidar location with name "backward" must be added - lidar turned backwards. In this case we would:

    nano ~/.ros/extrinsics/lidar_extrinsics_backward.yaml

and into it insert the coordinates of the lidar turned by 180 degrees in yaw:

    # This file must be formated in the following way
    #
    # x: 0.0
    # y: 0.0
    # z: 0.0
    # roll: 0.0
    # pitch: 0.0
    # yaw: 0.0
    #
    # Otherwise you might experience "No such key" errors when running robot description urdfs

    x: 0.2
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 3.14 #pi

2.) now the newly created extrinsic configuration `lidar_extrinsics_backward.yaml` must be set to be used the next time rpi boots:

    sudo nano /etc/ubiquity/robot.yaml

where the `lidar_position` must be set to "backward"

    lidar_position: 'backward' # to disable insert "None"

Now raspberry pi can either be restarted OR the systemctl reloaded with:

    sudo systemctl restart magni-base.service

The lidar is now set at a custom location which can be seen in Rviz. To see all available extrinsics files both for camera and lidar please check directories `~/.ros/extrinsics/` (user specified custom extrinsics) and `~/catkin_ws/src/magni_robot/magni_description/extrinsics/` (system default extrinsics - will be overridden with every git update)

***