---
title: "Introduction"
permalink: noetic_conveyorbot_intro
group: conveyorbot
rosver: noetic
nav_order: 1
nav_exclude: false
---

# ConveyorBot: Ubiquity Robotics' Conveyor Robot

**ConveyorBot** is a mobile robot that can transport loads up to 60 kg / 130 lbs.
It is a software suite that runs on a Ubiquity Robotics Magni that enables a robot conveyor application, where the robot will follow a set of markers called 'Fiducials' which are placed on the floor. 
The combination of Fiducials and the built-in touchscreen makes it easy to control the navigation and path planning of ConveyorBot.
The robot optimizes the workflow and frees up human resources, thus increasing productivity and reducing logistics costs.

> ![ConveyorBot](assets/breadcrumb/Breadcrumb_main.jpg)

### Where to use ConveyorBot

The ConveyorBot is designed to automate light weight material transportation and logistics.
It can carry the packages that can be handled by regular conveyor belts:

- height and width < 70 cm
- length < 140 cm
- weight < 60 kg
- mostly human carryable small parts: screws, bolts and small electronic items

## Already taken delivery of your robot?

As the software suite is designed to work with the tower and shell, you'll need to check the [Shell and Tower Assembly Instructions](noetic_quickstart_shell_tower).

If you are unfamiliar with the Magni robot, start with the [Quick Start](noetic_overview_need_to_know) category in the navbar.

## Don't have your robot yet?

ConveyorBot is optimized to run on a Raspberry Pi 4.
If you have one (especially if you have Pi Camera for it), there are numerous things you can do easily, even if you don't have one of out Magni Robots.  It would be possible to run our software on other platforms, but it would take a lot of effort to edit the various parameters.

* [Flash the custom SD card image.](noetic_quick_start_microsd)

* [Set up ROS on your workstation.](noetic_quick_start_workstation)

* [Connect the Pi with the Conveyorbot image to your workstation](noetic_quick_start_connecting).

* [Generate and print fiducials](noetic_conveyorbot_fiducials).

* [Visualize what's going on on the Pi using Rviz and other tools](noetic_conveyorbot_advanced#visualize-with-rviz)

* [Run Gazebo Simulator on your Workstation](noetic_conveyorbot_advanced#running-in-gazebo) to see how conveyorbot works in principle.








