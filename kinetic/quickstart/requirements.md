---
title: "Getting Started"
permalink: kinetic_overview_need_to_know
group: "quick start"
rosver: kinetic
nav_order: 1
nav_exclude: false
---

# Getting Started

## Requirements

This is a list of all you'll need in order to get Ubiquity robots up and running.

#### ➤ [2x 12V Lead Acid Battery](kinetic_overview_batteries)

As the robot ships without batteries.

#### ➤ [CR2032 Coin Battery](kinetic_overview_batteries#the-real-time-clock-battery)

To power up the real time clock and make sure robot time is always accurate.

#### ➤ [Wifi and SSH Capable Computer](kinetic_quick_start_connecting)

Most of the live administration to the robot is normally done by opening an SSH console to the host computer. As such you need what's likely to be a laptop and a capability to run SSH (via Terminal on Mac and Linux or through puTTY on Windows). 

Note: Not required for Conveyorbot.

## EZ-Map

I've you're using EZ-Map and a LiDAR sensor, [continue with EZ-Map set up](kinetic_ezmap_intro).

## Conveyorbot

If you're using the Coveyorbot package, which includes the tower and fiducial markers, [continue with Conveyorbot set up](noetic_conveyorbot_intro).

### For advanced use:

Note: Only continue with this if you intend to get more involved with the stock magni, it's not required for the average user.

#### ➤ [Linux compatible computer or virtual machine](kinetic_quick_start_workstation)

The Magni robot is normally running all the software for whatever sensors and navigation mode you are using for your project, but as viewing data through a terminal is very limited, it is handy to have a proper ROS workstation so you can visualize the data on your workstation using RViz and run more demanding nodes locally that can then interact with the robot through ROS.