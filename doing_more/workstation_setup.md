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

ROS runs on Ubuntu Linux. However not everyone has ubuntu linux installed on their machine, so we've created a virtual machine (VM) as a VirtualBox image. This is a system that allows most any laptop or computer to pretend that it is a Ubuntu Linux machine. Our image has a full install of Ubuntu, ROS and ubiquity robotics' software as appropriate for a workstation. The good news is that its quick and easy to get started this way. The downside is that the process of virtualization saps performance from your system so things will not be as fast as if you are running natively. On a fast system you may not notice this. In any event, the virtualization system is a good way to try out having a workstation before committing to set one up on your laptop.

## 1) Using our out of the box virtual machine

Download the appropriate VirtualBox software from [the VirtualBox website](https://www.virtualbox.org/wiki/Downloads) and install it.

Download the [Ubiquity Robotics Virtual Machine](https://drive.google.com/drive/folders/0B1zeRbBVLXhzZ0Q1TkxtbUxIcEU) and save it in VirtualBox's folder for virtual machines.  On Windows this is \Users\<username>\VirtualBox VMs.

Unzip the file you just downloaded. The result will be a single folder with the same name as the zip file.  Inside this folder find the file with the suffix .vbox, and double-click it.  This will cause VirtualBox to import the VM and open the VirtualBox Manager.

Check the following parameters of the VM:
  System/Base Memory: at least 1024 MB, why not 4096?
  Display/Video Memory: at least 64 MB, why not 128?
  Network/Adapter 1: Bridged Adapter

Start the VM. Your user ID is "ubuntu", and the password is "ubuntu".
Your workstation is ready to use.


## 2) Install ROS on a native Linux partition of your system

As you will see when we get to ROS installation, ROS will run natively on several operating systems although ROS and Ubiquity's software is only supported on Ubuntu Linux. In general if you haven't already got a ubuntu linux partition you should set one up. Ubunutu has a detailed guide that covers installing from a DVD, installing using a USB and even installing within Windows. This guide can be found here:

[Ubuntu Linux Official Installation Guide](https://help.ubuntu.com/community/Installation)

**Note: Ubuntu 16.04 is the only version currently supported by Ubiquity Robotics.**

Once you have a working Ubuntu Linux installation you can install ROS. ROS has a guide for how to do this contained here:

[ROS installation guide](http://wiki.ros.org/kinetic/Installation)

## Using Robot and Workstation Together

* Once you've done all this and assuming you are running a Magni you'll want to install all the files that help make Magni run. You can do this by typing:

  `sudo apt install ROS-kinetic-magni-robot`

* Before you go on, you should change the hostname of your robot.  This distinguishes your robot from others. Log in to the robot with ssh:

  `ssh ubuntu@ubiquityrobot.local,`
using the password which is
  `ubunutu`.

  To change this you edit a single file /etc/hostname that only contains the name of your robot computer on the network. You can change this using your favorite editor (e.g. pico, nano, vi, vim) but it is convienient to just do this by entering the following command line command:

    `sudo echo NEWHOSTNAME > /etc/hostname`

* The next step is to edit the file /etc/hosts to add two lines that tell again your computer what it is called. You can use your favorite editor to add the two lines but again you can just use the command line.

  `echo " 128.0.0.1 NEWHOSTNAME" >> /etc/hosts`

  `echo " 128.0.0.1 NEWHOSTNAME.local" >> /etc/hosts`

* If you now reboot the robot the new hostname will be used

  `sudo reboot`

  Now the robot should come up with the NEWHOSTNAME as

  `ssh ubuntu@NEWHOSTNAME.local`

`run pifi`

connect to external

## Set environment variables on the workstation

* Now go to your workstation and set its environment variables. When you set up ROS it assumes that the computer that it is set up on is the robot. Clearly if you are using the computer to see what is happening on the computer that is on the robot this is not what you want. The fixup is to tell the computer on which ROS is running which computer is the one that holds the master robot functions. You do this by setting an environment variable with the following command:

  `export ROS_MASTER_URI=http://NEWHOSTNAME.local:11311`

You then need to sync this environment variable with the system that you are running. The way to do this is to type the command.

  `source ~./bashrc `

### An alternative method

* On the workstation you append to your ~/.bashrc

`ip="$(hostname) -I|cut -d ' ' -f 1)" export ROS_IP=$ip`

then type

  source ~/.bashrc

* on the robot you need to type

  `ip="$(hostname) -I|cut -d ' ' -f 1)"`

  `export ROS_IP=$ip`

  `export ROS_MASTER_URI=http://$ip:11311`

* Then type

  'ifconfig -a`

 This gives you a set of information; in particular it will give you the IP address of the robot which you will use in a moment.

 on the workstation, type

 `export ROS_MASTER_URI=http:// [ROBOT'S IP ADDRESS]:11311`

 where you substitute for [ROBOTS IP ADDRESS] the actual robots IP address.

Now you should have a working local workstation that will enable you to do all kinds of cool things with your robot.
