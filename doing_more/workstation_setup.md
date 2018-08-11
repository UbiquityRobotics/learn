---
layout: default
title:  "How to set up a ROS workstation"
permalink: workstation_setup
---

# How to set up a ROS workstation
#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/)

A ROS workstation is a full setup of ROS (the Robot Operating System) on a desktop or laptop computer, that you can connect to your robot. ROS shares all the internal communication within the robot with a ROS workstation over your network. Thus you can monitor internal robot activity, see what the robot is seeing, send commands and even offload data processing tasks from the robot on to more powerful computers. A ROS workstation is needed if you are going to program the robot to perform you robotic application.

There are two methods to get a ROS workstation setup.

1. Use our out of the box virtual machine
2. Install ROS on a native Linux partition of your system

ROS runs on Ubuntu Linux. However not everyone has Ubuntu Linux installed on their machine, so we've created a virtual machine (VM) as a VirtualBox image. This is a system that allows most any laptop or computer to pretend that it is a Ubuntu Linux machine. Our VM has a full install of Ubuntu, ROS and Ubiquity Robotics' software as appropriate for a workstation. The good news is that its quick and easy to get started this way. The downside is that the process of virtualization saps performance from your system so things will not be as fast as if you are running natively. On a fast system you may not notice this. In any event, the virtualization system is a good way to try out having a workstation before committing to set one up on your laptop.

### 1) Using our out-of-the-box virtual machine

This is covered in the section above, [Connecting a Workstation for the First Time](connecting).  ROS is already set up on this virtual machine.

### 2) Install ROS on a native Linux partition of your system

Though ROS will run natively on several operating systems, ROS and Ubiquity's software is supported only on Ubuntu Linux. If you haven't already got a Ubuntu Linux partition you should set one up. Ubuntu has a [detailed guide](https://help.ubuntu.com/community/Installation) that covers installing from a DVD, installing using a USB and even installing within Windows.

**Note: Ubuntu 16.04 is the only version currently supported by Ubiquity Robotics.**

Once you have a working Ubuntu Linux installation you can install ROS. Refer to the
[ROS installation guide](http://wiki.ros.org/kinetic/Installation)

## Using Robot and Workstation Together

* Once you've got a workstation with Ubuntu and ROS, you should update the Ubiquity software. Because you have two ROS systems, you must keep their versions in sync.

To update the workstation, open a terminal window, login, and type:

  `sudo apt install ros-kinetic-magni-robot`

Then, in another terminal window, ssh into the robot and perform the update there:

    `sudo apt install ros-kinetic-magni-robot`

## Set environment variables on the workstation

* Now go to your workstation terminal window and set its environment variables. ROS assumes that the computer it is set up on is the robot. But we are running on the workstation, not the robot.  To tell ROS how to communicate with the robot, you must type:

  `export ROS_MASTER_URI=http://<robot IP address>:11311`  
  `export ROS_IP=http://<workstation IP address>`

  ROS on your workstation will use ROS_MASTER_URI to communicate with the ROS master node, which is on the robot. The setting of ROS_IP tells ROS the workstation IP address.  It may be that you can use NEWHOSTNAME.local instead of the address, but not all networks support it.

  However, environment variables set by the `export ...` method are not persistent across system boots.

  To make this environment variable persistent, we append its setting to the file called`~/.bashrc, which runs when the Ubuntu shell (called bash) is started. Use an editor, or from the command line:

  `echo "export ROS_MASTER_URI=http://<robot IP address>:11311" >> ~/.bashrc`
  `echo "export ROS_IP=http://<workstation IP address>"`

  **Warning: Don't do this if you have more than one robot. If you do, each terminal will have the same ROS_MASTER_URI and so will try to communicate with the same robot. Also watch out for changes in the IP addresses.**

  Verify that ROS is running and you are connected. On the workstation type:

  ```rostopic list```

  You should see a list of topics including /joy which means you can drive with a joystick.

  At this point you can drive the robot from your workstation's
  keyboard, just as in the Quick Start section called [Driving a Magni with a keyboard](keyboard_teleop). But now, instead of running the `teleop_twist_keyboard program` in the robot, you can run it in the workstation. The motion commands will be generated in the workstation rather than in the robot, and ROS will manage the communication between the two.

#### &larr;[back](connect_network)- - - - - - - - - - [next](rviz)&rarr;
