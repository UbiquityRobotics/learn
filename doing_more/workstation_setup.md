---
layout: default
title:  "How to setup a ROS workstation"
permalink: workstation_setup
---

# How to setup a ROS workstation

A ROS workstation is a full setup of ROS (the Robot Operating System) on a desktop or laptop computer, that you can connect to your robot. ROS has a framework to share all the internal communication within the robot with a ROS workstation over your network. As such you can monitor internal robot activity, see what the robot is seeing, send commands and even offload data processing tasks from the robot on to more powerful computers. A ROS workstation is really helpful if you want to do anything beyond simply driving the robot around and getting it to do voice commands.

There are two methods to get a ROS workstation setup.

1) Use our out of the box virtualization drive
2) Install ROS on a native linux partition of your system

ROS runs on ubunutu linux. However not everyone has ubuntu linux installed on  their machine, so we've created a virtual box image. This is a system that allows most any laptop or computer to temporarily pretend that it is a ubuntu linux machine, using virtualization techniques. Our image has a full install of ubuntu, ROS and ubiquity robotics' software as appropriate for a workstation. The good news is that its quick and easy to get started this way, the downside is that the process of virtualization saps compute performance from your system so things will not be as fast as if you are running natively. Usually using the virtualization system is a good way to quickly try out having a workstation before committing to set one up on your laptop.

## 1) Using our out of the box virtualization drive

To use our virtualization box setup you need to download it from:

[Ubiquity Robotics Virtualization System](https://drive.google.com/drive/folders/0B1zeRbBVLXhzZ0Q1TkxtbUxIcEU)

<Joe you need to change this link to make it "anyone with link can view">

<Joe please add instructions for what to do with it once you've downloaded it>

## 2) Install ROS on a native linux partition of your system

As you will see when we get to ROS installation, ROS will run natively on several operating systems although ROS and Ubiquity's software is only supported on Ubuntu Linux. In general if you haven't already got a ubuntu linux partition you should set one up. Ubunutu has a detailed guide that covers installing from a DVD, installing using a USB and even installing within Windows. This guide can be found here:

[Ubuntu Linux Official Installation Guide](https://help.ubuntu.com/community/Installation)

Once you have a working ubuntu linux installation you can go and install ROS. ROS has a guide for how to do this contained here:

[ROS installation guide](http://wiki.ros.org/kinetic/Installation)

Once you've done all this and assuming you are running a Magni you'll want to install all the files that help make Magni run. You can do this by typing:

`sudo apt intall ROS-kinetic-magni-robot`

Now you should have a working local workstation that will enable you to do all kinds of cool things with your robot.
