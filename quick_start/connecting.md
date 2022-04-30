---
layout: default
title:  "Connecting a Workstation, Starting And Stopping the Robot"
permalink: connecting
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

# Connecting a Workstation and Starting the Robot

The Magni robot is normally running all the software for whatever sensors and navigation mode you are using for your project but there is no practical way to see what the robot is doing within ROS because the Host Linux Cpu running the Magni itself is ```headless``` and normally only has WiFi to access.

The workstation is setup so the robot is the ROS master and this allows ROS running on the workstation to monitor ROS topics and inspect the state of ROS using command line tools or graphical tools.  

Your ```workstation``` may be a Ubuntu Linux system of your own such as a laptop, or a virtual Ubuntu machine running under another system.  

This page discusses connection of your laptop or workstation.  If you do not have a linux computer available, you can still use a virtual Linux machine to connect to the Magni robot.   

Additional information may be found [here](need_to_know).

## Reasons A Workstation Comes In Handy

This following section is optional, aimed at getting a better understanding of the power of running a workstation.

The robot itself is normally running all hardware drivers for sensors, such as camera, sonar, or even Lidar. If your system uses any form of navigation, then fiducial navigation, gmapping, move_base, move_basic or AMCL all are run on the robot itself, so they are close to the hardware.    

The reason a workstation or virtual machine is required in a system using a Magni, is so a way to get access by running a SSH terminals can start, stop and monitor software running on the robot.   A second fairly important console screen use for a workstation is it will also see all the ROS activity the workstation.

A workstation can use graphical tools such as RViz or plotting software.  

A workstation can monitor values in a ROS topic either from command line tools or such as for when you want to do PID tuning it is important to see the PID error values in a graph you would tend to run rqt_plot on the workstation or just run ```image_view``` to see what the raspicam camera is seeing as the robot drives around.

## Use of your own Linux machine with WiFi

If you have your own Linux laptop already configured with ROS running the same release as the robot, then you can use it to connect to the hotspot on the Magni computer using standard Linux interfaces to connect to a network. (Note since Ubuntu 18.04 the standard method is to use netplan)  

```Skip ahead to the next major section 'Connecting to the Robot and Logging In' for laptop if you are not going to run a VirtualBox instance.```


### Connecting a Virtual Machine

If you are running under VirtualBox, you will have installed this virtual machine with a bridged network.  Thus, the VM will see whatever network your host system is connected to. If your workstation is running, shut it down. Then connect your host system to the `ubiquityrobotXXXX network`.  Now start the workstation (that is, the Ubuntu system running under VBox); it will be connected to the robot's network `ubiquityrobotXXXX`. The password (sometimes called the security key) to connect is `robotseverywhere`.


# Connecting to the Robots Network and Logging In

At this point you should have either your own linux physical machine or the VirtualBox setup described above and we are ready to connect to the Magni WiFi hotspot or sometimes for development the robot may be wired to your LAN.

## Connecting To The Magni Robot WiFi hotspot

If you have received a Magni with the Raspberry Pi already installed, or loaded the default Raspberry Pi 3 image from downloads.ubiquityrobotics.com, the robot will boot up in WiFi Access Point mode. This is a WiFi mode which provides its own network to which you can connect your workstation.  The SSID (network name) is `ubiquityrobotXXXX` where XXXX is a number letter combination. You should find it under network list; the password (sometimes called the security key) to connect is `robotseverywhere`. If you can't find it under available networks, try restarting the robot.

There is a WIFI status led on the large MCB board if it is rev 5.2 later.  The MCB WIFI led is located on the right middle height of the MCB.  There is also this WIFI led on the optional sonar board however which does brief blinks ON at same rate.    The table below shows WiFi status indications for the led on the MCB which is mostly on with short blinks off at rates shown in the table.

| Blink Rate |	WiFi Status |
| ------- | ----------- |
| 2 per sec | WiFi is being initialized after a startup or reboot |
| 1 per sec | WiFi is operating as an Access Point (AP) and ready to be connected to by a laptop or phone |
| 1 per 2 sec | WiFi has successfully connected to your own WiFi ssid setup already |

## Open A SSH Console Once You Are On The Robot's Network

Once you are on the robot's network, via WiFi Or Wired, you can connect to the robot itself. On your workstation, start a terminal window (Linux shortcut: ctrl-alt-t). In that window, log in by typing

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

## Running ROS apps not requiring a Display

Many of the robot applications we supply as demos or some you may be writting yourself do not require a laptop if they do not need to display anything.  These types of command line applications can be run on a console to the robot.

A common simple app to control robot movement for example:

```rosrun teleop_twist_keyboard teleop_twist_keyboard.py```

Even some complex applications such as to start the robot's navigation software can be run on the robot directly in a text console.  

 ```roslaunch magni_demos simple_navigation.launch```  

We would suggest that to run simple_navigation you should see our [Fiducial-Based Localization Page](https://learn.ubiquityrobotics.com/fiducials) as our intent here is not to explain that complex mode, simply to show an example of a ROS app.

## Setup Laptop ROS Environment For Magni Control

In order to have your laptop be able to issue ROS messages to control the Magni such as running ```twist``` on the laptop or to run programs that require a graphical display such as ```rviz``` you will need to run ROS on your laptop and setup the laptop instance of ROS so that the robot is the ROS master.

Once the laptop (or VM) is configured with the robot as the ROS master you can inspect the ROS topics on the laptop and even view what the camera is seeing from your laptop or do other things like configure dynamic ROS parameters which all are done with a laptop display normally.

Rather than duplicate the required setup here you should refer to the [ROS Workstation Setup Page](https://learn.ubiquityrobotics.com/workstation_setup) to do these required configuration steps.

  * Setup the ROS version that the robot is running if not already done
  * Set environment variables so the robot is the ROS master
  * Setup the laptop and the robot so they are synchronized in time
  * Some environments may require setting up IP addresses in /etc/hosts
  * Run other programs using programs that require a graphics display.

At this point you will be able to control the robot from the workstation keyboard or by using Robot Commander.

# Doing a clean Shutdown Of the Magni Robot

Forgive us for first explaining how to shutdown and then power off the Magni robot but we felt this step may be missed if it were at the very end of this page.

We recommend doing clean shutdown on a Magni to lower your chances of corrupting your file system and to make your next powerup be faster by avoiding file system check next powerup.  If you have the time to properly shutdown the linux host you will not run the risk, however small, of corrupting your Micro SD card.

Once you have an open SSH console to the Magni, which is explained next on this page, you can shutdown the Magni in a clean way using these commands and actions

* Turn off the motor power by releasing the red ESTOP switch on the right and the RED motor power led will go off.
* In a SSH console session type  ```sudo shutdown -h now```
* After 10 seconds turn off the main power using the BLACK switch

If your Magni is a Magni silver and you have the sonar board you can always use the SW1 switch by holding it down for a couple seconds.  This will lead to a clean shutdown of the host Raspberry Pi CPU.

If you do not have a sonar board you can wire for yourself a reset switch that is normally open and when connected connects pin 31 of P702 on the MCB to pin 30 of that large 50 pin connector.
