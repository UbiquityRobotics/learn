---
title: "LiDAR"
permalink: noetic_magnisilver_lidar
group: "magni silver (gen 5)"
rosver: noetic
nav_order: 6
nav_exclude: false
--- 

# LiDAR Installation Guide

We currently support two LiDARs:

## UR-50 Long Range LiDAR

![Magni lidar connection](assets/camera_sensor/n301.png)

##### [Installation guide and info](noetic_ur50_lidar)

## UR-12 Short Range LiDAR

![Magni lidar connection](assets/camera_sensor/ld06.png)

##### [Installation guide and info](noetic_ur12_lidar)

<hr>

## Extras: RPLIDAR A1

While not officialy supported, many users of the magni have set their robot up with the popular RPLidar from SlamTec. The official driver for it can be found [here](http://wiki.ros.org/rplidar).

The purpose of this note here is that you may need to adjust the lidar's TF frame ("laser" in the magni_description urdf file) to turn it upside down to conform to usual standards, for use in e.g. EZ-Map and similar in case your unit has this issue. Not relevant for the R6 units.