---
layout: default
title:  "Connecting a Workstation, Starting And Stopping the Robot"
permalink: connecting
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

# Doing a clean Shutdown Of the Magni Robot

Forgive us for first explaining how to shutdown and then power off the Magni robot but we felt this step may be missed if it were at the very end of this page.

We recommend doing clean shutdown on a Magni to lower your chances of corrupting your file system and to make your next powerup be faster by avoiding file system check next powerup.  If you have the time to properly shutdown the linux host you will not run the risk, however small, of corrupting your Micro SD card.

Once you have an open SSH console to the Magni, which is explained next on this page, you can shutdown the Magni in a clean way using these commands and actions

* Turn off the motor power by releasing the red ESTOP switch on the right and the RED motor power led will go off.
* In a SSH console session type  ```sudo shutdown -h now```
* After 10 seconds turn off the main power using the BLACK switch

If your Magni is a Magni silver and you have the sonar board you can always use the SW1 switch by holding it down for a couple seconds.  This will lead to a clean shutdown of the host Raspberry Pi CPU.

If you do not have a sonar board you can wire for yourself a reset switch that is normally open and when connected connects pin 31 of P702 on the MCB to pin 30 of that large 50 pin connector.

# Connecting a Workstation and Starting the Robot

Your workstation may be a Ubuntu Linux system of your own, or you can use our preconfigured virtual machine.  Read more about this [here](need_to_know).

This page discusses the usage of a VirtualBox so that if you do not have a linux computer available you can still use a virtual Linux machine to connect to the Magni robot.   

## Use of your own Linux machine with WiFi

If you have your own Linux laptop you can use it to connect to the hotspot on the Magni computer using standard Linux interfaces to connect to a network.  Please skip ahead to the section below on connecting to a Magni for this case.

<!-- *{ Wayne: Shouldn't we use .ova instead of .vdi ?  That way people can use other virtual
   machine emulators to run the image.  Note to Joe--try this, and document if possible } -->

## Using our out-of-the-box virtual machine workstation

You can setup a Linux VirtualBox to communicate with the Magni robot.  In this case we have a Linux image that is preloaded with a great many ROS tools so this is a nice way to get going fast if you have the resources to be able to run VirtualBox environment.

* Download the appropriate VirtualBox software from [the VirtualBox website](https://www.virtualbox.org/wiki/Downloads) and install it.

*  Download the [Ubiquity Robotics Virtual Machine](https://downloads.ubiquityrobotics.com/vm.html) and save it in VirtualBox's folder for virtual machines.  On Windows this is \Users\<username>\VirtualBox VMs.

* On Windows or MacOS, unzip (decompress) the file you just downloaded. The result will be a single folder with the same name as the zip file.  Inside this folder find the file with the suffix `.vbox`, and double-click it.  This will cause VirtualBox to import the Virtual Machine (the VM) and open the VirtualBox Manager.

* Check the following parameters of the VM:

  * System/Base Memory: at least 2048 MB, why not 4096?
  * Display/Video Memory: at least 64 MB, why not 128?
  * Storage: at least 25GB
  * Network/Adapter 1: Bridged Adapter

* Start the VM. Your user ID is "ubuntu", and the password is "ubuntu".
Your workstation is ready to use.

In a later section we will explain how to attach the robot to an existing WiFi network and how ROS can take advantage of that to control the robot.

## Connecting to The Magni Robot's network

At this point you should have either your own linux physical machine or the VirtualBox setup described above and we are ready to connect to the Magni WiFi hotspot.

If you have received a Magni with the Raspberry Pi already installed, or loaded the default Raspberry Pi 3 image from downloads.ubiquityrobotics.com, the robot will boot up in WiFi Access Point mode. This is a WiFi mode which provides its own network to which you can connect your workstation.  The SSID (network name) is `ubiquityrobotXXXX` where XXXX is a number letter combination. You should find it under network list; the password (sometimes called the security key) to connect is `robotseverywhere`. If you can't find it under available networks, try restarting the robot.

### Connecting a Virtual Machine

If you are running under VirtualBox, you will have installed this virtual machine with a bridged network.  Thus, the VM will see whatever network your host system is connected to. If your workstation is running, shut it down. Then connect your host system to the `ubiquityrobotXXXX network`.  Now start the workstation (that is, the Ubuntu system running under VBox); it will be connected to the robot's network `ubiquityrobotXXXX`. The password (sometimes called the security key) to connect is `robotseverywhere`.


## Connecting to the Robot and Logging In

Now that you are on the robot's network, you can connect to the robot itself. On your workstation, start a terminal window (Linux shortcut: ctrl-alt-t). In that window, log in by typing

```ssh ubuntu@ubiquityrobot.local```

The password initially is set to "ubuntu".

 If you have trouble using ubiquityrobot.local, use the IP address, 10.42.0.1 instead. In some environments, this works better.

You may see:

>The authenticity of host '10.42.0.1 (10.42.0.1)' can't be established.  
    ECDSA key fingerprint is SHA256:sDDeGZzL8FPY3kMmvhwjPC9wH+mGsAxJL/dNXpoYnsc.  
    Are you sure you want to continue connecting (yes/no)?

Answer `yes`

>Failed to add the host to the list of known hosts (/somepath/.ssh/known_hosts).

Ignore that. You will be asked for the password, which initially is "ubuntu".

```ubuntu@10.42.0.1's password:ubuntu```

Now you are connected and logged in.

>Welcome to Ubuntu 16.04.3 LTS (GNU/Linux 4.4.38-v7+ armv7l)
  * Documentation:  https://help.ubuntu.com
  * Management:     https://landscape.canonical.com
  * Support:        https://ubuntu.com/advantage  
  0 packages can be updated.  
  0 updates are security updates.  
 Last login: Thu Feb 11 16:30:39 2016 from 10.42.0.143

 Finally, start the robot's software by typing:

```roslaunch magni_demos simple_navigation.launch```  

At this point you will be able to control the robot from the workstation keyboard or by using Robot Commander.
