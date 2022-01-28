---
layout: default
title:  "Setting up magni-default lidar"
permalink: default_lidar_setup
---
# Setting up ubiquity robotics default lidar

#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/)

In this document it is described how to set up the LS301 lidar on the Magni.

# Mounting the lidar

**Before doing anything with the electronics, the robot must be powered down the batteries disconnected.**

Connect the lidar lan cable into Raspberry Pi lan port and the power supply connector into the right molex connector on the MCB board: 

![Magni lidar connection](lidar_cabling2.jpg)

Using 4xM3 screws, the lidar can be mounted on the top plate that comes with Magni robot:

TODO image

Now the batteries can be reconnected and the robot powered on.
## Default location

The default location of the lidar is on the top plate looking into the forward direction (lidar connector is turned backward). Top plate of newer robots comes ready with holes for lidar mounting. But some of the older versions dont have those holes. In that case please either contact Ubiquity Robotics to supply you with a new top plate OR use double sided tape to mount the lidar. If you chose to mount the lidar with the double sided tape please make sure its 1.) turned exactly forward - a slight offset in angle can result in inaccurate localization and 2.) its center is EXACTLY the position shown on the picture.

![Magni Laser Rviz](magni_laser_rviz.png)

TODO add an accurate picture of the lidar location

On the picture red, green, blue correspond to x, y and z axis

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
    
# IP settings

If buying a lidar from Ubiquity Robotics, the lidar will come pre-set with the IPs that were used in these tutorials:
 - LIDAR IP: **192.168.42.222**
 - IP to which lidar is connected (workstation IP): **192.168.42.125**

If you want to change those .... TODO

<!-- TODO add compiling and running once we have the repo with the URLIDAR setup -->
<!-- TODO before this tutorial is written, lidar extrinsics should be configured through robot.yaml -->