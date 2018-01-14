---
layout: default
title:  "Connecting a Workstation for the First Time"
permalink: connecting
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/) - - - &uarr;[up](ix_quick_start)

# Connecting a Workstation for the First Time

Your workstation may be a Ubuntu Linux system of your own, or you can use our preconfigured virtual machine.  Read more about this [here](need_to_know).

<!--*{ Wayne: Shouldn't we use .ova instead of .vdi ?  That way people can use other virtual
   machine emulators to run the image.  Note to Joe--try this, and document if possible }*-->

### Using our out-of-the-box virtual machine workstation

* Download the appropriate VirtualBox software from [the VirtualBox website](https://www.virtualbox.org/wiki/Downloads) and install it.

*  Download the [Ubiquity Robotics Virtual Machine](https://drive.google.com/drive/folders/0B1zeRbBVLXhzZ0Q1TkxtbUxIcEU) and save it in VirtualBox's folder for virtual machines.  On Windows this is \Users\<username>\VirtualBox VMs.

  <!-- *{ TODO: We need a link and instructions for MacOS. }*-->

* On Windows, unzip the file you just downloaded. The result will be a single folder with the same name as the zip file.  Inside this folder find the file with the suffix `.vbox`, and double-click it.  This will cause VirtualBox to import the Virtual Machine (the VM) and open the VirtualBox Manager.

<!-- *{ TODO:On MacOS, to be written...}-->

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

 *{ Should we use name or 10.42.0.1 - seems to be inconsistent}

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


#### &larr;[back](robot_commander)- - - - - - - - - - [next](keyboard_teleop)&rarr;
