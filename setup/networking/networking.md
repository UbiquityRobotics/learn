# Connecting Your Robot to a Wifi Network

ROS (Robot Operating System) structures robot applications as a collection of
cooperating processes called ROS Nodes.  These ROS nodes communicate with one
another via standard internet communication standards.  One benefit of this
communication architecture is that nodes can be run more than one computer.
ROS provides a rich selection of nodes to support for robot application development
(e.g `rviz`, `rostopic`, etc.)  These robot application development nodes
are run on a robot software development machine (e.g. a desktop or laptop
computer.)  Since the robot is mobile, the only practical way for the
robot to communicate with the robot application development laptop/desktop
is via a WiFi wireless internet connection.   In this document, we describe
the required steps for configuring your robot to connect to the internet via Wifi.

This document is partitioned into the following sections:

* [Overview](#overview):
  This section provides an overview of the entire process of connecting your
  robot to the internet via WiFi.

  *{ waynegramlich: Put other sections here }*

* [Summary](#summary):
  This section summarizes the performed steps and suggests what to do next.

## Overview

<!-- Use the term robot computer instead of RasPi, so when we rush after the next
     cool single board computer, we do not have to do a massive edit of this document.
-->

All UR (Ubiquity Robotics) robots run ROS on a robot computer that is currently
a Raspberry Pi 3 single board computer.  The Robot Computer 3 is quad core
processor with an associated WiFi module that can be used to establish a wireless
internet connection.  This document describes the three basic steps required
to initially configure the robot computer.  The three basic steps are:

1. [Prepare Robot Computer for Initial Power Up](#prepare-robot-computer-for-initial-power-up):
   There is a small amount of work required before powering up the robot for the first time.

2. [Get to a Shell Prompt on the Robot Computer](#get-to-a-shell-prompt-on-the-robot-computer]):
   A term "shell prompt" refers getting to a textual command line interface
   running on the computer.  We document three different ways to get to a robot computer
   shell prompt along with the advantages and disadvantages of each different method.

3. [Initial Robot Computer Configuration](#initial-robot-computer-configuration)
   The robot computer is configured using robot compute shell prompt.
   The three sub-tasks are to:

   * configure Wifi using the `pifi` program.

   * change the computer host name (and why.)

   * create new user accounts (and why.)

The three steps are described in three separate sections below:

## Prepare Your Robot Computer for Initial Power Up:

There are three items you must have prior to powering up the robot computer.
These three items are to have a:

* Robot Computer:
  Obviously, you need a robot computer to power up.  The robot computer can be ordered
  separately from the robot.  In the United States, a separately ordered computer typically
  shows up in a day or two.  It should be mentioned that many UR robot configurations ship
  with a robot computer already installed. 

* Robot Computer Power Source:
  There are currently two ways to power up the Robot computer.  First, if you have actually
  have a robot, you can simply plug the robot computer into your robot and the robot
  batteries provide the required power to the robot computer.  Of course, this requires
  that you have previously ordered the robot batteries, they have arrived, been charged up,
  and installed into the robot.  Alternatively, you can power the Robot computer using USB
  cable that plugs into your laptop/desktop computer and micro-USB connector on the end.

* MicroSD Card with URPi Image:
  You need a Micro SD card that is at least 8GB in size and has a speed rating of class
  10 or higher.  This Micro SD card needs to have the URPi (Ubiquity Robotics Pi)
  image loaded onto it.  There is a good chance that your robot comes with a
  Micro SD card with the URPi image already installed on it.  Alternatively,
  you can purchase your own Micro SD card and install the URPi image on it.
  Please read the
  [URPi Image Installation](../urpi_install/urpi_install.md)
  document before ordering your MicroSD card.

In addition to the three items listed above, each of the shell prompt strategies
may require some additional items prior to powering up.

* Keyboard/Mouse/Display Strategy:
  The keyboard/mouse/display power up method, requires a USB keyboard, a USB mouse,
  and an HDMI display.

* Internet Cable Strategy:
  The internet cable strategy requires a standard internet cable.  It should probably
  be at least 6 feet long.

* Robot Access Point:
  The robot access point requires a laptop computer -- a desktop computer will not suffice,
  since we need to use the laptop WiFi subsystem to talk with the robot compute WiFi
  subsystem.

The details of each strategy are discussed in greater detail below.

It is important to understand that you can power up a robot computer without
actually having the robot.  You can start playing with ROS if you purchase robot
computer separately from your robot purchase.

### Get to a Shell Prompt on the Robot Computer:

*{ waynegramlich: The table below needs more work. }*

<--! Use a table to present the advantages/disadvantages. -->

There are three strategies of getting to a shell prompt:

* Keyboard/Mouse/Display:
  This method uses a USB keyboard

<Table Border="1" Summary="Advantages/disadvantages of Shell Prompt Methods">
<Caption><Em>Advantages/Disadvantages of Robot Computer Shell Prompt Access Methods</Em></Caption>
  <TR>
    <TH> Method </TH>
    <TH> Advantages </TH>
    <TH> Disadvantages </TH>
  </TR>
  <TR>
    <TH RowSpan="2"> Computer/Mouse/Keyboard </TH>
    <TD> Most Reliable <TD>
    <TD> Requires extra keyboard, mouse, and display </TD>
  </TR>
  <TR>
    <TD> Does not need desktop/laptop </TD>
    <TD> Robot needs to be opened </TD>
  </TR>
  <TR>
    <TH RowSpan="2"> Network Cable </TH>
    <TD RowSpan="2"> Reliable </TD>
    <TD> Requires Internet Cable </TD>
  </TR>
  <TR>
    <TD> Requires Desktop/Laptop </TD>
  </TR>
  <TR>
    <TH RowSpan="1"> Robot WiFi Access Point </TH>
    <TD> Least Reliable </TD>
    <TD> Laptop only; no desktop </TD>
  </TR>
</Table>

*{ waynegramlich: fill in the details }*

### Initial Robot Computer Configuration]

*{ waynegramlich: fill in the details }*

## Connecting to Robot Computer with Keyboard/Mouse/Display

*{ waynegramlich: fill in the details }*

## Connecting to Robot Computer with Network Cable

*{ waynegramlich: fill in the details }*

## Connecting to Raspberry Robot Computer via Robot Wifi Access Point

*{ waynegramlich: fill in the details }*

## Using PiFi to Configure Robot Computer Wifi

*{ waynegramlich: fill in the details }*

## Changing the Robot Host Name

*{ waynegramlich: fill in the details }*

## Creating User Accounts

*{ waynegramlich: fill in the details }*

## Development Machine Networking

*{ waynegramlich: Verify zeroconf works on development machine. }*

## Conclusion

*{ waynegramlich: fill in the details }*

## Older Documentation

{* waynegramlich:  The text below needs to be merged into the sections above. }*

If you loaded the default Robot Computer 3 image from downloads.ubiquityrobotics.com, 
or have received a Magni with the Robot Computer already installed, the Robot should boot up in WiFi access point mode. This means you should be able to begin testing your robot immediately, and be able to attach it to an existing network.  If you have a logitech controller or a fiducial marker, you should be able to drive or guide your robot once it is turned on.  The robot will broadcast it’s SSID as ubiquityrobot, and the password to connect is “robotseverywhere”

Once connected I attempt to locate the robot by typing in a terminal window:

    ping ubiquityrobot.local   
  
  (this may take a bit of time before it responds)

once it does it should display the robots IP number. I then ssh to it:

    ssh ubuntu@10.42.0.1

(type password)

    The authenticity of host '10.42.0.1 (10.42.0.1)' can't be established.
    ECDSA key fingerprint is SHA256:sDDeGZzL8FPY3kMmvhwjPC9wH+mGsAxJL/dNXpoYnsc.
    Are you sure you want to continue connecting (yes/no)? yes 
    Failed to add the host to the list of known hosts (/home/alan/.ssh/known_hosts).
    ubuntu@10.42.0.1's password: 
    Welcome to Ubuntu 16.04.3 LTS (GNU/Linux 4.4.38-v7+ armv7l)

    * Documentation:  https://help.ubuntu.com
    * Management:     https://landscape.canonical.com
    * Support:        https://ubuntu.com/advantage

    0 packages can be updated.
    0 updates are security updates.

    Last login: Thu Feb 11 16:30:39 2016 from 10.42.0.143

(this robot has no real time clock, so the date is wrong)

Next you should connect your robot to the local area network:

    ubuntu@ubiquityrobot:~$ pifi status
    Network Mangager reports AP mode support on B8:27:EB:2B:3F:6B
    Device is currently acting as an Access Point
    ubuntu@ubiquityrobot:~$ pifi list seen
    DIRECT-447301B2
    fedland
    HP-Print-72-Officejet Pro 6830
    HOME-B805-2.4
    NETGEAR37
    ubuntu@ubiquityrobot:~$ pifi add fedland simbacat
    Error writing to /var/lib/pifi/pending, make sure you are running with sudo
 
 (oops)

     ubuntu@ubiquityrobot:~$ sudo pifi add “ssid”  “passwd”

If now sudo reboot should come up on “ssid” wifi

To test connect your laptop to the local area network, and ping: 

    ping ubiquityrobotXXXX.local

    alan@anfrosbase:~$ ping ubiquityrobot.local
    PING ubiquityrobot.local (10.0.0.113) 56(84) bytes of data. 
    64 bytes from 10.0.0.113: icmp_seq=1 ttl=64 time=97.6 ms
    64 bytes from 10.0.0.113: icmp_seq=2 ttl=64 time=5.70 ms

so now ssh into 10.0.0.113


    alan@anfrosbase:~$ ssh ubuntu@10.0.0.113

    The authenticity of host '10.0.0.113 (10.0.0.113)' can't be established.
    ECDSA key fingerprint is SHA256:sDDeGZzL8FPY3kMmvhwjPC9wH+mGsAxJL/dNXpoYnsc
  
    etc.
    
    
(check the date)

    ubuntu@ubiquityrobot:~$ date
    Mon Aug 14 17:16:26 UTC 2017

Now we have the correct date  so I will update

    sudo apt-get update
    sudo apt-get upgrade


This should take some time, since it may have been a while since the original image was made.


I then check to see if magni is running by typing:

    rostopic list

If things are ok you should see topics including the /joy  which means you can drive with a joystick.
You are now networked.  If your robot can't find a network you have added it will go back to AP mode.
