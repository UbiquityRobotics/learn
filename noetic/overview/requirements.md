---
title: "Requirements"
permalink: noetic_overview_need_to_know
group: overview
rosver: noetic
nav_order: 1
nav_exclude: false
---

# Requirements

This is a list of all you'll need in order to get Ubiquity robots up and running.

#### ➤ [2x 12V Lead Acid Battery](noetic_overview_batteries)

As the robot ships without batteries.

#### ➤ [CR2032 Coin Battery](noetic_overview_batteries#the-real-time-clock-battery)

To power up the real time clock and make sure robot time is always accurate.

#### ➤ [Wifi and SSH Capable Computer](noetic_quick_start_connecting)

Most of the live administration to the robot is normally done by opening an SSH console to the host computer. As such you need what's likely to be a laptop and a capability to run SSH (via Terminal on Mac and Linux or through puTTY on Windows).

### For advanced use:

#### ➤ [Linux compatible computer or virtual machine](noetic_quick_start_workstation)

The Magni robot is normally running all the software for whatever sensors and navigation mode you are using for your project, but as viewing data through a terminal is very limited, it is handy to have a proper ROS workstation so you can visualize the data on your workstation using RViz and run more demanding nodes locally that can then interact with the robot through ROS.

#### ➤ Sensors

If the software suite you intend to use requires additional sensors you'll need to obtain and mount them as well.

- [Sonars](noetic_magnisilver_sonars)

- [UR-50 Long Range LiDAR](noetic_ur50_lidar)

- [UR-12 Short Range LiDAR](noetic_ur12_lidar)