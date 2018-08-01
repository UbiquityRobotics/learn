---
layout: default
permalink: need_to_know
---
## Requirements
#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

### About Magni

Ubiquity Robotics use Raspberry Pi 3s running Ubuntu 16.04, ROS Kinetic and custom software for the Magni platform. There are also utility programs that will enable you to connect to a local area network.

[Batteries](#Batteries)
[Communications](#communications)  
[The Workstation](#the-workstation)  
[The Configuration File](#the-configuration-file)  
[ROS Params](#ros-params)

### Batteries

The robot ships by air world wide. Because batteries are difficult to ship worldwide but are easy to source locally the robot does not come with batteries included to keep shipping costs down. Also the robot will accept different battery sizes, so the user can select batteries depending on whether they prefer a long endurance but heavier robot or a short endurance but lighter robot. In short you need to find your own batteries to put in the robot and these are commonly available online (https://www.batterysharks.com/) or in local stores that supply products for scooters, wheelchairs, uninterupped power supply systems or even automotive. The robot requires 2X 12V batteries and typically we recommend either:

12350 size. Typical capacity - 35Ah usually much bigger (and heavier) than most applications demand - recommended only for those who must have extraordinary endurance - typicaly 24 hours or more of continuous use.

1270 sized battery. Typical capacity 7AH - 10AH. The preferred and most common choice usually provides 7-8 hours of continuous use with a typical duty cycle. This size battery makes the robot light enough to pick up. 

1250 or a 1255 sized battery. Typical capacity 4-6 AH capacities provides around 4 hours of endurance. Used when portability of the robot is at a premium - for example if you are travelling by air with the robot.

In all cases we recommend a non-spill-able, deep cycle, sealed lead acid battery of either a Gel type or AGM type - although the robot can accept any type of battery pack with a voltage in the range of 21V - 30V, the provided charger is specified for lead acid batteries. We provide foam inserts with the robot to fit the above battery sizes. Do not discard these foam inserts with the packaging.

### Communications

Of course, as the robot is delivered it has no connection to your local network. Because of this, the robot has its own network (called an access point or "AP mode") that enables you to [connect to it directly](connecting), without connecting to your local network. For example, you can drive the robot with our Android Robot Commander app. You can use AP mode to connect directly to the robot from a workstation, to run ROS commands such as keyboard teleoperation. However, in AP mode, the robot cannot access the Internet.

[Connecting to your local network](connect_network) is required for programming the robot.
 After you have done this, with ROS set up on your workstation, you can issue ROS commands to execute on the workstation which then control the robot over your local network. This may improve performance if the workstation is more powerful than the robot.

The local network must support [zeroconf](https://en.wikipedia.org/wiki/Zero-configuration_networking) and operate in in PSK (Pre-Shared Key) infrastructure mode. This permits you to use names like `robot.local` instead of the IP address, which can change.

### The Workstation

You need a workstation to control the robot. In these tutorials it is assumed that your workstation, whether a laptop, desktop, or virtual machine, is running Ubuntu 16.04 LTS.  ("16.04" stands for "2016, April" and "LTS" denotes "Long Term Support" which means 5 years.)  **Ubiquity Robotics supports only this release.**

**The workstation must have WiFi capability.**  That is, it must be able to connect wirelessly to a router or access point. Most laptops do, many desktops don't.

Not everyone has Ubuntu Linux installed on their machine, so we've created a virtual machine (VM) as a downloadable VirtualBox image. This is a system that allows most any computer to pretend that it is a Ubuntu Linux machine. Our VM is preconfigured with Ubuntu, ROS (Robot Operating System) and Ubiquity Robotics' workstation software.

Note that there will be two--or maybe three--simultaneously running systems in this configuration, all sharing the same keyboard:
* Robot OS--ROS under Ubuntu on the robot.
* Workstation OS--ROS under Ubuntu 16.04 on the workstation hardware, OR
  * ROS under Ubuntu 16.04 in a VM under VirtualBox.
* Workstation Native OS--If you are using a VM, you will have Windows or MacOS hosting VirtualBox.

Be aware of this--it's easy to type a command into the wrong system.

Instructions for workstation setup are given in the sections below, as they are needed. We know you may want to start as soon as possible to get your projects underway. Since we provide downloadable software images for the Raspberry Pi 3 and for Virtual Box, it is possible to 'try before you buy.'  [Download Site](https://downloads.ubiquityrobotics.com/)

### The Configuration File

The configuration file is used to tell the robot software what options are installed.  It is located at: ```/etc/ubiquity/robot.yaml```

If sonars are installed, the config file should contain the line:
```
sonars: 'pi_sonar_v1'
```
to enable the sonars, and this to disable them:
```
sonars: None
```

<Todo: finish a description of the configuration file, and how to make use of it, and insert a link to a previously utilized configuration file>


### ROS Params

The parameter `ubiquity_robot_mode` specifies the level of capabilities available in the robot. Possible values are 'core', 'teleop', and 'navigation'. The launch file magni_bringup base.launch runs as part of the boot process and automatically sets the parameter to 'teleop'.  The launch file magni_demos simple_navigation.launch enables navigation, so the parameter is set to 'navigation'.
