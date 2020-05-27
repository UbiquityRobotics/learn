---
layout: default
permalink: need_to_know
---
## Requirements
#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

### About Magni

Ubiquity Robotics use Raspberry Pi 3s running Ubuntu 16.04, ROS Kinetic and custom software for the Magni platform. There are also utility programs that will enable you to connect to a local area network.

[Batteries](#batteries)  
[Communications](#communications)  
[The Workstation](#the-workstation)  
[The Configuration File](#the-configuration-file)  
[ROS Params](#ros-params)  
[Keeping Your Software Current](#keeping-your-software-current)

### Batteries

The robot ships by air worldwide. The batteries are not included in order to keep shipping costs down, as they are are difficult to ship worldwide and safety restrictions vary by destination. The recommended lead acid batteries are easy to source locally.

An added advantage of not including batteries is that the robot accepts different battery sizes so the user can select batteries depending on whether they prefer a long-endurance heavier robot or a short- endurance lighter robot. In short you need to find your own batteries to put in the robot and these are commonly available online (https://www.batterysharks.com/) or in local stores that supply products for scooters, wheelchairs, uninterupped power supply systems or even automotive.

<H4 style="color:red">AT ALL TIMES IF RUNNING OR CHARGING THE BATTERY VOLTAGE CONNECTED TO THE Main BOARD MUST REMAIN 30.0V OR LESS.</H4>

#### Specific Qualified Lead Acid Batteries

The robot requires 2X 12V lead acid style batteries and typically we recommend one of the choices in this section.

1250 or a 1255 sized battery. Typical capacity 4-6 Ah capacities provides around 4 hours of endurance. Used when portability of the robot is at a premium - for example if you are travelling by air with the robot.

1270 sized battery. Typical capacity 7AH - 10 Ah. The preferred and most common choice. Usually provides 7-8 hours of continuous use with a typical duty cycle. This size battery makes the robot still light enough to lift.

12350 size. Typical capacity - 30-35 Ah usually much bigger (and heavier) than most applications demand - recommended only for those who must have extraordinary endurance - typicaly 24 hours or more of continuous use. This sized battery makes the robot sufficiently heavy that it will be difficult for most users to lift.

In all cases we recommend a non-spill-able, deep cycle, sealed lead acid battery of either a Gel type or AGM type - although the robot can accept any type of battery pack with a voltage in the range of 22.5V - 30V.

The provided charger is specified for lead acid batteries. We provide foam inserts with the robot to fit the above battery sizes. Do not discard these foam inserts with the packaging.

#### The Size Of The Battery Compartment

We ship Magni with a foam cut-out that nicely holds two 1270 format Lead Acid batteries.   Hopefully this was not disguarded when unpacking.

The floor of the battery compartment is always at least 205mm x 258mm.  Due to manufacturing tolerances it may be larger but that cannot be guaranteed.

From the floor to the top of the top rails on the side we have 135mm of height.  Batteries can go up taller to the top flat metal plate and that would be a height of 184mm.  These measurement are intentionally meant to avoid trying to get so close on a mm of clearance as our manufacturing cannot guarantee mm specs.


#### Other information That May Help Battery Selection

We are looking into solutions that perhaps in the future may be able to support  LiFePO4 battery chemistry but we have not determined if they meet the always under 30.0V limit even when on charger and fully charged.   There are many lithium solutions that would be over 30.0V and those absolutely cannot be used.   

The stock battery charger we supply is ONLY FOR LEAD ACID batteries and will NOT work and in fact may be dangerous for other battery technologies.

### Host Linux computer

The robot ships with a built in Raspberry Pi single board computer running the Linux operating system.   Units sold in 2019 use the Raspberry Pi 3 B+ running Ubuntu Linux 16.04 and the Kinetic version of ROS (Robot Operating System).  Initial support for the Raspberry Pi 4 is currently planned for shipment on units sometime in mid 2020 timeframe.

Most of the live administration to the robot is normally done by opening an SSH console to the host computer.  If access is via a linux laptop to the built in WiFi hotspot then the following command is used to get a console screen from a console on the laptop.

    ssh ubuntu@10.42.0.1

Of course names can be used for the host but above a simple IP address is used.  The default password for shipped units is  ```ubuntu```

Although the host computer has a robust file system less chance of file system corruption is always possible if the Linux system is shutdown in a clean way.  When connected to the robot with an SSH console this is how to reboot (-r) or completely halt (-h) the robot to do a clean power off. Below is the full halt which will ask for root password that is ```ubuntu``` as shipped from factory.

    sudo shutdown -h now

There is a shutdown button feature on a standard Magni but only users with the sonar board will have the actual button.   We do a clean shutdown of the robot by connecting GPIO 6 which is on pin 31 of both the 40 pin P701 and the 50 pin P702.  Pin 30 of these two connectors is ground.  If you have a sonar board there is a button labeled ```SW1``` for shutdown as it connects to GPIO 6.

If you do not have the sonar board the pins to the 50 pin P702 are available to be connected to with push on connectors common for prototype uses.  Many users make their own shutdown button with one wire to pin 31 and one wire to pin 30.  Note that for P702 pin 1 is on the far left and the bottom row of pins.  These jacks have odd number pins on bottom and even starting with 2 on top row.

### Communications

Of course, as the robot is delivered it has no connection to your local network. Because of this, the robot has its own network (called an access point or "AP mode") that enables you to [connect to it directly](connecting), without connecting to your local network. For example, you can drive the robot with our Android Robot Commander app. You can use AP mode to connect directly to the robot from a workstation, to run ROS commands such as keyboard teleoperation. However, in AP mode, the robot cannot access the Internet.

[Connecting to your local network](connect_network) is required for programming the robot.
 After you have done this, with ROS set up on your workstation, you can issue ROS commands to execute on the workstation which then control the robot over your local network. This may improve performance if the workstation is more powerful than the robot.

The local network must support [zeroconf](https://en.wikipedia.org/wiki/Zero-configuration_networking) and operate in in PSK (Pre-Shared Key) infrastructure mode. This permits you to use names like `robot.local` instead of the IP address, which can change.

### The Workstation

You need a workstation to control the robot. In these tutorials it is assumed that your workstation, whether a laptop, desktop, or virtual machine, is running Ubuntu 16.04 LTS.  ("16.04" stands for "2016, April" and "LTS" denotes "Long Term Support" which means 5 years.)  **Ubiquity Robotics supports only this release.**

**The workstation must have WiFi capability.**  That is, it must be able to connect wirelessly to a router or access point. Most laptops do, many desktops don't.

Not everyone has Ubuntu Linux installed on their machine, so we've created a virtual machine (VM) as a downloadable VirtualBox image. This is a system that allows most any computer to pretend that it is a Ubuntu Linux machine. Our VM is preconfigured with Ubuntu, ROS (Robot Operating System) and Ubiquity Robotics' workstation software.

Note that there will be two--or maybe three--simultaneously running systems in this configuration, all sharing the same keyboard:
* Robot OS--ROS under Ubuntu on the robot.
* Workstation OS--ROS under Ubuntu 16.04 on the workstation hardware, OR
  * ROS under Ubuntu 16.04 in a VM under VirtualBox.
* Workstation Native OS--If you are using a VM, you will have Windows or MacOS hosting VirtualBox.

Be aware of this--it's easy to type a command into the wrong system.

Instructions for workstation setup are given in the sections below, as they are needed. We know you may want to start as soon as possible to get your projects underway. Since we provide downloadable software images for the Raspberry Pi 3 and for Virtual Box, it is possible to 'try before you buy.'  [Download Site](https://downloads.ubiquityrobotics.com/)

### The Configuration File

The configuration file is used to tell the robot software what options are installed.  It is located at: ```/etc/ubiquity/robot.yaml```

If sonars are installed, the config file should contain the line:
```
sonars: 'pi_sonar_v1'
```
to enable the sonars, and this to disable them:
```
sonars: None
```

<Todo: finish a description of the configuration file, and how to make use of it, and insert a link to a previously utilized configuration file>


### ROS Params

The parameter `ubiquity_robot_mode` specifies the level of capabilities available in the robot. Possible values are 'core', 'teleop', and 'navigation'. The launch file magni_bringup base.launch runs as part of the boot process and automatically sets the parameter to 'teleop'.  The launch file magni_demos simple_navigation.launch enables navigation, so the parameter is set to 'navigation'.

### Keeping Your Software Current

From time to time we update software for improvements and fixes.  The robot should be updated to stay up to date. Upgrade information is found [here](updating).

<!--
On your workstation, start a terminal window (Linux shortcut: ctrl-alt-t). In that window, log in by typing:

```ssh ubuntu@ubiquityrobot.local```  
(use the robot's name or IP address)

```sudo apt-get update```

```sudo apt-get upgrade```-->
