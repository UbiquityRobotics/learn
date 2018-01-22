---
layout: default
permalink: need_to_know
---
## What You Need To Know
#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

### About Loki and Magni 

Ubiquity Robotics use Raspberry Pi 3s running Ubuntu 16.04, ROS Kinetic and custom software for both Loki Magni platforms. These are set up so that you can quickly get your robot working. There are also utility programs that will enable you to connect to a local area network.

### Communications

As the robot is delivered it of course has no connection to your local network. Because of this, the robot has its own network (called an access point or "AP mode") that enables you to [connect to it directly](connecting), without connecting to your local network. For example, you can drive the robot with our Android Robot Commander app. You can use AP mode to connect directly to the robot from a workstation, to run ROS commands such as keyboard teleoperation. However, in AP mode, the robot cannot access the Internet.

[Connecting to your local network](connect_network) is required for programming the robot.
 After you have done this, with ROS set up on your workstation, you can issue ROS commands to execute on the workstation which then control the robot over your local network. This may improve performance if the workstation is more powerful than the robot.

The local network must support [zeroconf](https://en.wikipedia.org/wiki/Zero-configuration_networking) and operate in in PSK (Pre-Shared Key) infrastructure mode. This permits you to use names like `robot.local` instead of the IP address, which can change.

### The Workstation

You need a workstation to control the robot. In these tutorials it is assumed that your workstation, whether a laptop, desktop, or virtual machine, is running Ubuntu 16.04 LTS.  ("16.04" stands for "2016, April" and "LTS" denotes "Long Term Support" which means 5 years.)  Ubiquity Robotics supports only this release.

**The workstation must have WiFi capability.**  That is, it must be able to connect wirelessly to a router or access point. Most laptops do, many desktops don't.

Not everyone has Ubuntu Linux installed on their machine, so we've created a virtual machine (VM) as a downloadable VirtualBox image. This is a system that allows most any computer to pretend that it is a Ubuntu Linux machine. Our VM is preconfigured with Ubuntu, ROS (Robot Operating System) and Ubiquity Robotics' workstation software.

Note that there will be two--or maybe three--simultaneously running systems in this configuration, all sharing the same keyboard:
* Robot OS--ROS under Ubuntu on the robot.
* Workstation OS--ROS under Ubuntu 16.04 on the workstation hardware, OR
  * ROS under Ubuntu 16.04 in a VM under VirtualBox.
* Workstation Native OS--If you are using a VM, you will have Windows or MacOS hosting VirtualBox.

Be aware of this--it's easy to type a command into the wrong system.

Instructions for workstation setup are given in the sections below, as they are needed. We know you may be excited and want to start as soon as possible to get your projects underway. Since we provide software images for the Raspberry Pi 3 or a Virtual Box, it is possible to 'try before you buy.'  [Download Site](https://downloads.ubiquityrobotics.com/)
