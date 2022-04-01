---
rosver: noetic
nav_exclude: true
---

# Ubiquity Robotics Documentation

<H3 style="color:red">Warning</H3>

The Magni robot is strong, fast, and heavy. Initially, use lumber, bricks, or whatever you have to lift the wheels free of the floor, or run it somewhere where it can't hurt anyone or anything if it surprises you. NOT ON A TABLE TOP.

<H4 style="color:red">Always remove the red battery cable for any work on boards to remove live voltage from the main board</H4>

You may visit the [Ubiquity Robotics Forum](https://forum.ubiquityrobotics.com) for assistance or more information and tips

## Preface: Ubiquity Robotics' Robot Development Platform

**The goal of this guide is to get web, mobile, and maker developers using ROS (the Robot Operating System) via Magni, the Ubiquity Robotics development platform.** We hope you will see the amazing possibilities and opportunities, dive in and never look back.

### What is Magni and What Does it Do?

> ![Magni/Loki](assets/Magni_best.png)

Magni is a hardware platform in the form of a mobile base. When powered by ROS software, it can handle vision, localization, communication and mobility. Magni is a heavyweight platform capable of moving payloads up to 100kg. It can autonomously move anything on top of it to wherever that item needs to go, avoiding obstacles along the way.

A mobile base is the heart of a modular/interoperability model of robotics. Without a shared base, parts such as robotic arms, sensors, and other tools could not find or get to their location. Even at their desired location, each would require an independent “brain” to know what to do, which would in turn require interpretation between each of the components.

If you want to build anything from laundryBots to a fleet of waiters for hire, Ubiquity's hardware, driven by the open source ROS software, is an ideal starting point for any developer transitioning into robotics.

### What is ROS?

The Robot Operating System (ROS) is an open source framework for writing robot software. In ROS, the intense research and expertise of engineers, computer scientists and more have been distilled into ready-to-use development environments and function calls. ROS has simplified the task of creating complex and robust robot behavior across a wide variety of tasks.

### Start Designing!

When a mobile base can move wherever it is needed without running into obstacles, lots of opportunities arise.

Whether you are delivering coffee, transporting people, or moving industrial equipment, progress starts with knowing where you are, what your goal is, and how to achieve it while avoiding obstacles. Magni makes learning these core concepts affordable, accessible and fun.


<!--
#### High Level Overviews
1. [Introduction To The Magni Platform](introduction)  
2. [Requirements: Batteries, Network, Workstation use & Configuration ](need_to_know)  
3. [Magni Power Controls](magni_key)  

#### Quick Start

1.  [If you don't have your robot yet](quick_start/no_robot.md)
2.  [If you are using our Raspberry Pi image on its own without a Magni](quick_start/image_no_magni.md)
3.	[Unboxing, putting in batteries and charging them](quick_start/unboxing/unboxing.md)
4.  [Connecting a workstation, starting and stopping the Robot](quick_start/connecting.md)
5.  [Driving a robot with a keyboard using teleop twist](quick_start/keyboard_teleop.md)
6.  [How to drive with optional Logitech controller](quick_start/logitech.markdown)
7.	[How to set up the camera](quick_start/camera_sensor/installation.md)
8.	[How to set up the sonar board](quick_start/camera_sensor/sonar_setup.md)
1.  [How to set up UR50 lidar](quick_start/ur50_lidar_setup_common.md)
10.	[How to make Follow Me work with a fiducial](quick_start/fiducial_follow.md)


####	Doing More With Your Robot

1.	[How to connect the robot with your network](doing_more/network_connect.md)
2.  [How to set-up a workstation with ROS and Synchronize Time](doing_more/workstation_setup.md)
3.	[Visualize With rviz](doing_more/rviz.md)
4.  [Lidar Navigation Using A Magni](doing_more/lidar_navigation.md)
5.	[Fiducials Enable You To Set Waypoints and Goals](doing_more/fiducials.md)
6. [Use A Script To Control Robot Navigation](python_script_1)
7. [The Fiducial Follow App](programming_your_robot/fiducial_follow_app.md)
8.	[Using Robot Commander Joystick and Voice Control](quick_start/Robot_Commander_AP.markdown)
9. [Running Magni in Simulation](simulation)  

####	Support and Verification Of Your Robot

1. [Burning a fresh Image, Updating Software](support/updating.md)
2. [Updating Firmware](support/firmware_upgrade.md)
3. [Verification Of Operation](support/verification.md)
4. [GPIO Line Usage And Using Them For Your Own Needs](support/GPIO_lines.md)
5. [MCB Connectors, Power Supplies, LEDs & Helpful Mods](mcb_pinouts_leds_userpower)
6. [Mechanical Drawings](support/mechanical_drawings.md)
7. [ROS Notes With A Focus On Magni Usage](programming_your_robot/overview.md)
8. [Diagnostics, Developer Tips and I2C](support/diagnostics.md)
9. [PC Board Revisions For MCB, Switch Boards & more](PC_Board_RevId)
10. [Removal And Installation Of Main PC Boards](support/board_replacement.md)
11. [Troubleshooting](support/troubleshooting.md)




12.	Writing Your First Script
13.	Creating a Map
14.	Autonomous Driving
15.	Going Forward and Avoiding Obstacles with Code
16.	Going to a Specific Location on Your Map Using Code
17.	Monitor Magni Battery Status
18.	Button Events
19.	What to Read Next

* [Software Reference](software_reference/software_reference.md)

<!--

* [Setup In Depth](setup/setup.md):

  Everything from ordering batteries and network cables to figuring out networking issues.

* [Miscellaneous](misc/misc.md)

* [CoffeeBot Challenge](ix_coffeebot)
* [Learning with Magni in Simulation](ix_simulation1)
* [Challenge in Simulation](ix_simulation2) -->
