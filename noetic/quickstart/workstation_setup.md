---
title: "Workstation Setup"
permalink: noetic_quick_start_workstation
group: "quick start"
rosver: noetic
nav_order: 4
nav_exclude: false
--- 

# The Workstation

A ROS workstation is a full setup of ROS (the Robot Operating System) on a desktop or laptop computer, that you can connect to your robot. ROS shares all the internal communication within the robot with a ROS workstation over your network. Thus you can monitor internal robot activity, see what the robot is seeing, send commands and even offload data processing tasks from the robot on to more powerful computers. A ROS workstation is needed if you are going to program the robot to perform you robotic application.

The prime reason a workstation or virtual machine is required in a system using a Magni is to see all the ROS activity the workstation can monitor ROS topics or other status for ROS at that time. A workstation is also used so that graphical tools such as RViz or plotting software can be run on the screen of the workstation. This is the case if the robot is to do navigation you would then run visual tools such as the RViz environment to see the robot move around or to set goals. In this mode you could configure RViz to show you the sensors such as our sonar board or even a Lidar unit.   

Another need for a workstation would be so that you can monitor values in a ROS topic either from command line tools or such as for when you want to do PID tuning it is important to see the PID error values in a graph you would tend to run rqt_plot on the workstation or just run ```image_view``` to see what the raspicam camera is seeing as the robot drives around.

We are not in this section discussing purely virtual simulation tools but those too can be run on a workstation assuming the workstation has a powerful enough configuration.  

Your ```workstation``` may be a Ubuntu Linux system of your own such as a desktop or laptop computer, or you can use a virtual machine as described below.

### Native Linux machine

If you have a native machine running the an Ubuntu LTS you should be able to install ROS on it and link it to your robot.

Though ROS will run natively on several operating systems, ROS and Ubiquity's software is supported only on Ubuntu Linux LTS versions. If you haven't already got a Ubuntu partition you should set one up. Ubuntu has a [detailed guide](https://help.ubuntu.com/community/Installation) that covers installing from a DVD, installing using a USB and even installing within Windows.

Once you have a working Ubuntu Linux installation [you can install ROS](noetic_quick_start_workstation#install-ros). 

**WARNING: Make sure you have the correct Ubuntu LTS for the ROS version you'll be installing.**

### Virtual machine

You can setup a Linux VirtualBox to communicate with the Magni robot to work around the dedicated Ubuntu machine requirement.

* Download the appropriate VirtualBox software from [the VirtualBox website](https://www.virtualbox.org/wiki/Downloads) and install it.

* Download the correct Ubuntu LTS VM, for example [Kubuntu if you're used to Windows](https://www.osboxes.org/kubuntu/) and save it in VirtualBox's folder for virtual machines. On Windows this is `\Users\<username>\VirtualBox VMs`.

* On Windows or MacOS, unzip (decompress) the file you just downloaded. The result will be a single folder with the same name as the zip file.  Inside this folder find the file with the suffix `.vdi`, and double-click it. This will cause VirtualBox to import the Virtual Machine (the VM) and open the VirtualBox Manager.

* Check the following parameters of the VM:

  * System/Base Memory: at least 2048 MB, why not 4096?
  * Display/Video Memory: at least 64 MB, why not 128?
  * Storage: at least 25GB
  * Network/Adapter 1: Bridged Adapter <- Important, so you'll be able to see your robot on LAN

The workstation is setup so the robot is the ROS master and this allows ROS running on the workstation to monitor ROS topics and inspect the state of ROS using command line tools or graphical tools.  

Note that there will be two--or maybe three--simultaneously running systems in this configuration, all sharing the same keyboard:
* Robot OS, the Ubuntu running on your robot you'll be SSH'd into
* Workstation OS, the VM you'll be using to connect to the robot with ROS
* Workstation Native OS, your VM host OS

Be aware of this--it's easy to type a command into the wrong system.

## Install ROS

For ROS 1, the currently releant Ubuntu + ROS pairs are the following:

* Ubuntu 16.04 Xenial + ROS Kinetic

* Ubuntu 18.04 Bionic + ROS Melodic

* Ubuntu 20.04 Focal + ROS Noetic

Make sure to install the correct version that's compatible with the [image you downloaded](noetic_pi_image_downloads). See the [ROS downloads page](http://wiki.ros.org/ROS/Installation) for installation instructions.

Next, add the [Ubiquity package repository](https://packages.ubiquityrobotics.com/) to add Magni packages to your apt installer.

### Setup Environment

On the workstation, we want to make sure that we have zeroconf networking enabled:

    sudo apt install libnss-mdns avahi-daemon avahi-utils

Once you've got a workstation with Ubuntu and ROS, you should update the Ubiquity software. Because you have two ROS systems, you must keep their versions in sync.

To update the workstation, open a terminal window:

    sudo apt update
    sudo apt upgrade

#### Set environment variables on the workstation

Test if zeroconf greatly simplifies connecting to the robot, but it doesn't work in every environment.
On your workstation you should be able to ping the robot with `ping ROBOTNAME.local` where ROBOTNAME is the hostname of your robot.

#### If zeroconf works (the ping succeeds):

Now go to your workstation terminal window and set its environment variables. ROS assumes that the computer it is set up on is the robot. But we are running on the workstation, not the robot.  To tell ROS how to communicate with the robot, you must type:

    export ROS_MASTER_URI=http://ROBOTNAME.local:11311
    export ROS_HOSTNAME=$(hostname).local

Again replace ROBOTNAME with your robot's hostname.

However, environment variables set by the `export ...` method are not persistent across system boots.

To make this environment variable persistent, we append its setting to the file called `~/.bashrc`, which runs when the Ubuntu shell (called bash) is started. Use an editor, or from the command line: **Warning: Don't do this step if you have more than one robot. If you do, each terminal will have the same ROS_MASTER_URI and so will try to communicate with the same robot. Instead, set the environment variables manually for each terminal.**

    echo "export ROS_MASTER_URI=http://ROBOTNAME.local:11311" >> ~/.bashrc
    echo "export ROS_HOSTNAME=$(hostname).local" >> ~/.bashrc

Again replace ROBOTNAME with your robot's hostname.

#### If zeroconf is not working (the ping fails):

**Note, if you are using IP addresses instead of zeroconf, we highly recommend setting up static IP addresses, or DHCP static assignments**

Now go to your workstation terminal window and set its environment variables. ROS assumes that the computer it is set up on is the robot. But we are running on the workstation, not the robot.  To tell ROS how to communicate with the robot, you must type:

    export ROS_MASTER_URI=http://<robot_ip>:11311
    export ROS_IP=<workstation_ip>

However, environment variables set by the `export ...` method are not persistent across system boots.

To make this environment variable persistent, we append its setting to the file called `~/.bashrc`, which runs when the Ubuntu shell (called bash) is started. Use an editor, or from the command line: **Warning: Don't do this step if you have more than one robot. If you do, each terminal will have the same ROS_MASTER_URI and so will try to communicate with the same robot. Instead, set the environment variables manually for each terminal.**

    echo "export ROS_MASTER_URI=http://<robot_ip>:11311" >> ~/.bashrc
    echo "export ROS_IP=<workstation_ip>" >> ~/.bashrc

### Synchronize Magni Time To The Workstation Time

When using the robot as the master ROS node the workstation will need to use the same time as the robot or information on the ROS topics will not be recognized properly.  If your laptop and Magni are set properly you should be able to check the time on both at the same time and verify that they are the same.  The process below is to be used only if that fails,
perhaps due to NTP config changes that have been done on your laptop or the Magni.

The Magni robot uses chrony to synchronize time so it is best if your workstation also uses chrony although not a hard requirement it may
avoid some issues.

On a laptop that has chrony installed use these commands:

    sudo chronyc -a local stratum 10
    sudo chronyc -a allow 0/0

On a console to the Magni use these commands:

    sudo systemctl stop magni-base
    sudo chronyc -a add server <yourLaptopName> iburst
    sudo chronyc -a burst 2/4
    sudo systemctl start magni-base

In order to set date and time to your timezone, use command:

    sudo dpkg-reconfigure tzdata

### Test the connection

  Verify that ROS is running and you are connected. On the workstation type:

    rostopic list

  You should see a list of topics including /cmd_vel which means you can drive the robot.

  At this point you can drive the robot from your workstation's
  keyboard, just as in the Quick Start section called [Driving a Magni with a Keyboard](keyboard_teleop). But now, instead of running the `teleop_twist_keyboard` in the robot, you can run it in the workstation. The motion commands will be generated in the workstation rather than in the robot, and ROS will manage the communication between the two.
