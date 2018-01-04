---
layout: default
title:  "Connecting a Workstation for the First Time"
permalink: connecting
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/) - - - &uarr;[up](ix_quick_start)

# Connecting a Workstation for the First Time

In this manual it is assumed that your workstation, whether a laptop, desktop, or virtual machine, is running Ubuntu 16.04 LTS.  ("16.04" stands for "2016, April" and "LTS" stands for "Long Term Support" which means that the software is supported for 5 years.)

**The workstation must have WiFi capability.**  Most laptops do, many desktops don't.

Not everyone has Ubuntu Linux installed on their machine, so we've created a virtual machine (VM) as a VirtualBox image. This is a system that allows most any computer to pretend that it is a Ubuntu Linux machine. Our VM has a full install of Ubuntu, ROS (Robot Operating System) and Ubiquity Robotics' workstation software. The good news is that its quick and easy to get started this way. The downside is that the process of virtualization saps performance from your system so things will not be as fast as if you are running natively. On a fast system you may not notice this. In any event, the virtualization system is a good way to try out having a workstation before committing to set one up on your laptop.
*{ Wayne: Shouldn't we use .ova instead of .vdi ?  That way people can use other virtual
   machine emulators to run the image.  Note to Joe--try this, and document if possible }*

### Using our out-of-the-box virtual machine workstation

* Download the appropriate VirtualBox software from [the VirtualBox website](https://www.virtualbox.org/wiki/Downloads) and install it.

*  Download the [Ubiquity Robotics Virtual Machine](https://drive.google.com/drive/folders/0B1zeRbBVLXhzZ0Q1TkxtbUxIcEU) and save it in VirtualBox's folder for virtual machines.  On Windows this is \Users\<username>\VirtualBox VMs.

   *{ TODO: We need a link and instructions for MacOS. }*

* On Windows, unzip the file you just downloaded. The result will be a single folder with the same name as the zip file.  Inside this folder find the file with the suffix `.vbox`, and double-click it.  This will cause VirtualBox to import the Virtual Machine (the VM) and open the VirtualBox Manager.

 *{ TODO:On MacOS, to be written...}

* Check the following parameters of the VM:

  * System/Base Memory: at least 2048 MB, why not 4096?
  * Display/Video Memory: at least 64 MB, why not 128?
  * Storage: at least 25GB
  * Network/Adapter 1: Bridged Adapter

* Start the VM. Your user ID is "ubuntu", and the password is "ubuntu".
Your workstation is ready to use.

In a later section we will explain how to attach the robot to an existing WiFi network and how ROS can take advantage of that to control the robot.

#### The Network, out of the box

If you have received a Magni with the Raspberry Pi already installed, or loaded the default Raspberry Pi 3 image from downloads.ubiquityrobotics.com, the robot will boot up in WiFi Access Point mode. This is a WiFi mode which provides its own network to which you can connect your workstation.  The SSID (network name) is `ubiquityrobotXXXX` where XXXX is a number letter combination; the password to connect is `robotseverywhere`.

#### Connecting a Virtual Machine

If you are running under VirtualBox, you will have installed this virtual machine with a bridged network.  Thus, the VM will see whatever network your host system is connected to. If your workstation is running, shut it down. Then connect your host system to the `ubiquityrobotXXXX network`.  Now start the workstation (that is, the Ubuntu system running under VBox); it will be connected to the robot's network `ubiquityrobotXXXX`.

#### Connecting a Linux Workstation

If you are running Ubuntu 16.04LTS natively, connect to the `ubiquityrobotXXXX` network using the Ubuntu system's facilities.

#### Connecting to the Robot and Logging In

Now that you are on the robot's network, you can connect to the robot itself. On your workstation, start a terminal window (Linux shortcut: ctrl-alt-t). In that window, log in by typing

```ssh ubuntu@ubiquityrobot.local```

You may see:

>The authenticity of host '10.42.0.1 (10.42.0.1)' can't be established.  
    ECDSA key fingerprint is SHA256:sDDeGZzL8FPY3kMmvhwjPC9wH+mGsAxJL/dNXpoYnsc.  
    Are you sure you want to continue connecting (yes/no)?

Answer `yes`

>Failed to add the host to the list of known hosts (/somepath/.ssh/known_hosts).

You will be asked for the password, which initially is "ubuntu".

```ubuntu@10.42.0.1's password:ubuntu```

Finally,

>Welcome to Ubuntu 16.04.3 LTS (GNU/Linux 4.4.38-v7+ armv7l)
  * Documentation:  https://help.ubuntu.com
  * Management:     https://landscape.canonical.com
  * Support:        https://ubuntu.com/advantage  
  0 packages can be updated.  
  0 updates are security updates.  
 Last login: Thu Feb 11 16:30:39 2016 from 10.42.0.143

<!--
This robot's clock will never have synchronized with a time-server, so disregard the date.


If you can't connect to a network, but you want to run the robot, you should synchronize dates on both the laptop and robot:


```
ssh ubuntu@ubiquityrobot.local sudo -S date -s @`( date -u +"%s" )`
```
Next, use pifi to list the nearby networks and to connect your robot to your local area network:

 ```ubuntu@ubiquityrobot:~$ pifi status```

 >Network Mangager reports AP mode support on B8:27:EB:2B:3F:6B  
 Device is currently acting as an Access Point

 ```ubuntu@ubiquityrobot:~$ pifi list seen```

>MyNetwork  
Neighbor's network  
Other Network

```ubuntu@ubiquityrobot:~$ sudo pifi add “MyNetwork”  “password”```

Note: "sudo" is a linux command that allows administrative actions.  Linux will often ask you for your password ("ubuntu", if you haven't changed it) when you use it.

Now reboot.

```sudo reboot```

The robot will reboot and attach to the “ssid” wifi network. To test,

```$ping ubiquityrobot####.local```

The ping result shows the network address of the robot:

 >PING ubiquityrobot.local (10.0.0.113) 56(84) bytes of data.  
 64 bytes from 10.0.0.113: icmp_seq=1 ttl=64 time=97.6 ms  
 64 bytes from 10.0.0.113: icmp_seq=2 ttl=64 time=5.70 ms  

Now ssh into 10.0.0.113

 ```$ ssh ubuntu@10.0.0.113```

As before:
>The authenticity of host '10.0.0.113 (10.0.0.113)' can't be established.  
ECDSA key fingerprint is SHA256:sDDeGZzL8FPY3kMmvhwjPC9wH+mGsAxJL/dNXpoYnsc.
Are you sure you want to continue connecting (yes/no)?

```yes```

>Failed to add the host to the list of known hosts (/somepath/.ssh/known_hosts).
ubuntu@10.0.0.113's password:

 ```ubuntu```

>Welcome to Ubuntu 16.04.3 LTS (GNU/Linux 4.4.38-v7+ armv7l)
  * Documentation:  https://help.ubuntu.com
  * Management:     https://landscape.canonical.com
  * Support:        https://ubuntu.com/advantage
 22 packages can be updated.
 12 updates are security updates.
 Last login: Thu Feb 11 16:31:06 2016 from 10.42.0.143

Verify that Magni is running and you are connected:

```rostopic list```

If things are ok you should see a list of topics including /joy which means you can drive with a joystick.

Now check the date.

```ubuntu@ubiquityrobot:~$ date```
 >Mon Aug 14 17:16:26 UTC 2017

Now that you have the correct date you can update the robot to get changes that have been made since the robot was manufactured.

```sudo apt-get update```  
```sudo apt-get upgrade```

This should take some time, since it may have been a while since the original image was made.


Occasionally, the update/upgrade fails to complete. Usually this is due to the presence of a 'lock' file. To fix this:

        sudo rm /var/lib/dpkg/lock

-->

#### &larr;[back](robot_commander)- - - - - - - - - - [next](keyboard_teleop)&rarr;
