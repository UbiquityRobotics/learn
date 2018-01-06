---
layout: default
permalink: read_this_first
---
## Read This First
#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

### Communications

As the robot is delivered it of course has no connection to your local network. Because of this, the robot has its own network (called an access point or "AP mode") that enables you to [connect to it directly](connecting), without connecting to your local network. For example, you can drive the robot with our Android Robot Commander app. You can use AP mode to connect directly to the robot from a workstation, to run ROS commands such as keyboard teleoperation. However, in AP mode, the robot cannot access the Internet.

[Connecting to your local network](connect_network) is required for programming the robot.
 After you have done this, with ROS set up on your workstation, you can issue ROS commands to execute on the workstation which then control the robot over your local network. This may improve performance if the workstation is more powerful than the robot.
 
The local WiFi network must support zeroconf (the thing that allows you to find printers on your local WiFi network) and PSK (the thing that allows you to input a WiFi password). Almost all networks these days do, but if you suspect yours doesn't (for example if you can't find a WiFi connected device or you can't enter a WiFi password). Then 
you might need to either use the robot in AP mode (explained in tutorials 1-5 or upgrade your router.

*{Wayne: The wikipedia article on `zeroconf` is not very good.  I think it would be better to summarize the important
feature of `zeroconf` which is that it allows a processor that is running the `zeroconf` protocols to access the
robot and/or workstation using a name of the form `HOSTNAME.local`, where `HOSTNAME` is name specified by the
robot software developer.  This is instead of using a numeric internet address that continually changes. }*

//   [//]: # "The statement: local network must support zeroconf and operate in in PSK (Pre-Shared Key) infrastructure mode. isn't very helpful. Perhaps we should talk about this in terms of the implications as very few people are going to know the jargon or care about it. This doesn't get better by better explainations of what this is. 

You might say something like: The local WiFi network must support zeroconf (the thing that allows you to find printers on your local WiFi network) and PSK (the thing that allows you to input a WiFi password). Almost all networks these days do, but if you suspect yours doesn't (for example if you can't find a WiFi connected device or you can't enter a WiFi password). Then 
you might need to either use the robot in AP mode (explained in tutorials 1-5 or upgrade your router.

It could be even better if there was a way to test to make sure your network had these capabilities
"

### The Workstation

You need a workstation to control the robot. In this manual it is assumed that your workstation, whether a laptop, desktop, or virtual machine, is running Ubuntu 16.04 LTS.  ("16.04" stands for "2016, April" and "LTS" denotes "Long Term Support" which means 5 years.)  Ubiquity Robotics supports only this release.

**The workstation must have WiFi capability.**  Most laptops do, many desktops don't.

*{ Wayne: I think we need to describe what WiFi capability "means".  Maybe something like, the workstation needs
the ability to connected to a WiFi access point, such as a small office/home office.  I'll let you wordsmith this. }*

Not everyone has Ubuntu Linux installed on their machine, so we've created a virtual machine (VM) as a downloadable VirtualBox image. This is a system that allows most any computer to pretend that it is a Ubuntu Linux machine. Our VM is preconfigured with Ubuntu, ROS (Robot Operating System) and Ubiquity Robotics' workstation software.

Note that there may be three simultaneously operating systems involved in this configuration:
* ROS under Ubuntu on the robot.
* ROS under Ubuntu 16.04 on the workstation hardware, OR
  * ROS under Ubuntu 16.04 in a VM under VirtualBox.
* If you are using a VM, you will have Windows or MacOS hosting VirtualBox.

*{ Wayne: I recommend that we name the thes operating system here -- 1) Robot OS, 2) Workstation ?? OS, and
3) Workstation Native OS.  I don't have a good word for `??` yet. }*

Instructions for workstation setup are given below, as they are needed.
