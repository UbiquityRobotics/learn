---
layout: default
title:  "How to setup a ROS workstation"
permalink: workstation_setup
---

# How to setup a ROS workstation

A ROS workstation is a full setup of ROS (the Robot Operating System) on a desktop or laptop computer, that you can connect to your robot. ROS shares all the internal communication within the robot with a ROS workstation over your network. Thus you can monitor internal robot activity, see what the robot is seeing, send commands and even offload data processing tasks from the robot on to more powerful computers. A ROS workstation is really helpful if you want to do anything beyond simply driving the robot around and getting it to do voice commands.

There are two methods to get a ROS workstation setup.

1) Use our out of the box virtual machine
2) Install ROS on a native Linux partition of your system

ROS runs on Ubuntu Linux. However not everyone has Ubuntu Linux installed on their machine, so we've created a virtual machine (VM) as a VirtualBox image. This is a system that allows most any laptop or computer to pretend that it is a Ubuntu Linux machine. Our image has a full install of Ubuntu, ROS and Ubiquity Robotics' software as appropriate for a workstation. The good news is that its quick and easy to get started this way. The downside is that the process of virtualization saps performance from your system so things will not be as fast as if you are running natively. On a fast system you may not notice this. In any event, the virtualization system is a good way to try out having a workstation before committing to set one up on your laptop.

## 1) Using our out of the box virtual machine

Download the appropriate VirtualBox software from [the VirtualBox website](https://www.virtualbox.org/wiki/Downloads) and install it.

Download the [Ubiquity Robotics Virtual Machine](https://drive.google.com/drive/folders/0B1zeRbBVLXhzZ0Q1TkxtbUxIcEU) and save it in VirtualBox's folder for virtual machines.  On Windows this is \Users\<username>\VirtualBox VMs.

Unzip the file you just downloaded. The result will be a single folder with the same name as the zip file.  Inside this folder find the file with the suffix `.vbox`, and double-click it.  This will cause VirtualBox to import the VM and open the VirtualBox Manager.

Check the following parameters of the VM:

  * System/Base Memory: at least 1024 MB, why not 4096?
  * Display/Video Memory: at least 64 MB, why not 128?
  * Network/Adapter 1: Bridged Adapter

Start the VM. Your user ID is "ubuntu", and the password is "ubuntu".
Your workstation is ready to use.


## 2) Install ROS on a native Linux partition of your system

Though ROS will run natively on several operating systems, ROS and Ubiquity's software is supported only on Ubuntu Linux. If you haven't already got a ubuntu linux partition you should set one up. Ubunutu has a detailed guide that covers installing from a DVD, installing using a USB and even installing within Windows. This guide can be found here:

[Ubuntu Linux Official Installation Guide](https://help.ubuntu.com/community/Installation)

**Note: Ubuntu 16.04 is the only version currently supported by Ubiquity Robotics.**

Once you have a working Ubuntu Linux installation you can install ROS. ROS has a guide for how to do this contained here:

[ROS installation guide](http://wiki.ros.org/kinetic/Installation)

## Using Robot and Workstation Together

* Once you've got a workstation with Ubuntu and ROS and assuming you are running a Magni you'll want to install the files that support Magni. Type:

  `sudo apt install ROS-kinetic-magni-robot`

* Before you go on, you should change the hostname of your robot,to distinguish your robot from others. Log in to the robot with ssh:

  `ssh ubuntu@ubiquityrobot.local,`
using the password which is
  `ubunutu`.

  To change the hostname you edit a single file /etc/hostname that only contains the name of your robot computer on the network. You can do this using your favorite editor (e.g. pico, nano, vi, vim) but it is convenient to just enter the following command:

    `sudo echo NEWHOSTNAME > /etc/hostname`

* The next step is to edit the file /etc/hosts to add two lines that again tell your computer what it is called. You can use your favorite editor to add the two lines but again you can just use the command line.

  `echo " 128.0.0.1 NEWHOSTNAME" >> /etc/hosts`

  `echo " 128.0.0.1 NEWHOSTNAME.local" >> /etc/hosts`

* If you now reboot the robot the new hostname will be used

  `sudo reboot`

  Now the robot will come up with the NEWHOSTNAME as

  `ssh ubuntu@NEWHOSTNAME.local`

  will show.

#### What are these 2 lines?

`run pifi`

connect to external

## Set environment variables on the workstation

* Now go to your workstation )HOW?) and set its environment variables. When you set up ROS it assumes that the computer that it is set up on is the robot. This is not what you want, you want

  `export ROS_MASTER_URI=http://NEWHOSTNAME.local:11311`

  This sets the variable ROS_MASTER_URI to the address http://NEWHOSTNAME.local:11311. ROS on your workstation will use this to communicate with the ROS master node which is on the robot.

  However, environment variables are not persistent!

#### Something is missing here !!!!!!!!!!!!!!!!
source ~./bashrc

### An alternative method

* On the workstation, append to your ~/.bashrc file

  `ip="$(hostname) -I|cut -d ' ' -f 1)"``
  `export ROS_IP=$ip`

then type

  `source ~/.bashrc`

* on the robot you need to type

  `ip="$(hostname) -I|cut -d ' ' -f 1)"`

  `export ROS_IP=$ip`

  `export ROS_MASTER_URI=http://$ip:11311`

* Then type

  `ifconfig -a`

 This gives you, among other information, the IP address of the robot which you will use in a moment.

 on the workstation, type

 `export ROS_MASTER_URI=http:// [ROBOT'S IP ADDRESS]:11311`

 where you substitute for [ROBOTS IP ADDRESS] the actual robots IP address.

 #### issue: persistence
 #### issue: user may not know how to switch between workstation and robot.

Now you should have a working local workstation that will enable you to do all kinds of cool things with your robot.
