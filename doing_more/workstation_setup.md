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

On the workstation, we want to make sure that we have zeroconf networking enabled:

    sudo apt install libnss-mdns avahi-daemon avahi-utils

Once you've got a workstation with Ubuntu and ROS, you should update the Ubiquity software. Because you have two ROS systems, you must keep their versions in sync.

To update the workstation, open a terminal window:

    sudo apt update
    sudo apt upgrade

Then, in another terminal window, ssh into the robot and perform the update there:

    sudo apt update
    sudo apt upgrade

## Set environment variables on the workstation

Test if zeroconf greatly simplifies connecting to the robot, but it doesn't work in every environment.
On your workstation you should be able to ping the robot with `ping ROBOTNAME.local` where ROBOTNAME is the hostname of your robot.

### If zeroconf works (the ping succeeds):

Now go to your workstation terminal window and set its environment variables. ROS assumes that the computer it is set up on is the robot. But we are running on the workstation, not the robot.  To tell ROS how to communicate with the robot, you must type:

    export ROS_MASTER_URI=http://ROBOTNAME.local:11311  
    export ROS_HOSTNAME=$(hostname).local

Again replace ROBOTNAME with your robot's hostname.

However, environment variables set by the `export ...` method are not persistent across system boots.

To make this environment variable persistent, we append its setting to the file called ~/.bashrc, which runs when the Ubuntu shell (called bash) is started. Use an editor, or from the command line: **Warning: Don't do this step if you have more than one robot. If you do, each terminal will have the same ROS_MASTER_URI and so will try to communicate with the same robot. Instead, set the environment variables manually for each terminal.**

    echo "export ROS_MASTER_URI=http://ROBOTNAME.local:11311" >> ~/.bashrc
    echo "export ROS_HOSTNAME=$(hostname).local" >> ~/.bashrc

Again replace ROBOTNAME with your robot's hostname.

### If zeroconf is not working (the ping fails):

<<<<<<< HEAD
**Note, if you are using IP addresses instead of zeroconf, we highly recommend setting up static IP addresses or DHCP static assignments.**
=======
**Note, if you are using IP addresses instead of zeroconf, we highly recommend setting up static IP addresses, or DHCP static assignments**
>>>>>>> 16691276fd0c6d09b7478600c20b43ab2a7e1ed9

Now go to your workstation terminal window and set its environment variables. ROS assumes that the computer it is set up on is the robot. But we are running on the workstation, not the robot.  To tell ROS how to communicate with the robot, you must type:

    export ROS_MASTER_URI=http://<robot_ip>:11311  
    export ROS_IP=<workstation_ip>

However, environment variables set by the `export ...` method are not persistent across system boots.

To make this environment variable persistent, we append its setting to the file called ~/.bashrc, which runs when the Ubuntu shell (called bash) is started. Use an editor, or from the command line: **Warning: Don't do this step if you have more than one robot. If you do, each terminal will have the same ROS_MASTER_URI and so will try to communicate with the same robot. Instead, set the environment variables manually for each terminal.**

    echo "export ROS_MASTER_URI=http://<robot_ip>:11311" >> ~/.bashrc
    echo "export ROS_IP=<workstation_ip>" >> ~/.bashrc


## Test the connection

  Verify that ROS is running and you are connected. On the workstation type:

    rostopic list

  You should see a list of topics including /cmd_vel which means you can drive the robot.

  At this point you can drive the robot from your workstation's
  keyboard, just as in the Quick Start section called [Driving a Magni with a Keyboard](keyboard_teleop). But now, instead of running the `teleop_twist_keyboard` in the robot, you can run it in the workstation. The motion commands will be generated in the workstation rather than in the robot, and ROS will manage the communication between the two.

#### &larr;[back](connect_network)- - - - - - - - - - [next](rviz)&rarr;
