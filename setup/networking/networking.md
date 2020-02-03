# Connecting Your Robot to a Wifi Network

ROS (Robot Operating System) structures robot applications as a collection of
cooperating processes called ROS Nodes.  These ROS nodes communicate with one
another via standard internet communication standards.  One benefit of this
communication architecture is that nodes can be run more than one computer.
ROS provides a rich selection of nodes to support for robot application development
(e.g `rviz`, `rostopic`, etc.)  These robot application development nodes
are run on a robot software development machine (e.g. a desktop or laptop
computer.)  Since the robot is mobile, the only practical way to communicate
between  the robot computer and he robot application development laptop/desktop
computer is via a WiFi wireless internet connection.   In this document, we describe
the required steps for configuring your robot computer to connect to the internet
via Wifi.

This document is partitioned into the following sections:

* [Overview](#overview):
  This section provides an overview of the entire process of connecting your
  robot to the internet via WiFi.

* [Prepare Your Robot Computer for Initial Power Up](prepare-your-robot-computer-for-initial-power-up):
  This section discusses what you need to do before applying power to your
  robot computer.

* [Get to a Shell Prompt on the Robot Computer](get-to-a-shell-prompt-on-the-robot-computer):
  This section describes three different ways get to a shell prompt (i.e command
  line interface) on  the robot computer.

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

1. [Prepare Your Robot Computer for Initial Power Up](#prepare-your-robot-computer-for-initial-power-up):
   There is a small amount of work required before powering up a robot computer
   for the first time.

2. [Get to a Shell Prompt on the Robot Computer](#get-to-a-shell-prompt-on-the-robot-computer):
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
  image loaded onto it. The robot comes with a Micro SD card with the URPi image already installed on it. Alternatively,
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

## Get to a Shell Prompt on the Robot Computer:

There are three strategies of getting to a shell prompt on your robot computer:

* [Keyboard/Mouse/Display Strategy](connecting-to-robot-computer-with-keyboardmousedisplay):
  This method uses a USB keyboard, a USB mouse, and an HDMI display into the robot
  computer.  This is considered the most reliable method, since the robot computer
  tends to log progress information and error conditions to the display as it is
  powering up.  To offset reliability there are the disadvantages of requiring
  extra devices and having to plug all these devices into the robot computer before
  power up.  None-the-less, if you have the devices, it is extremely likely that
  you will successfully reach a shell prompt.

* Internet Cable Strategy:
  This method uses a physical internet cable to connect between your local
  network and the robot computer. Be avare that all internet cables don't have 
  accessible internet port.

* Robot Access Point Strategy:
  This method just requires a laptop.  There are more steps involved and if something
  goes wrong during one of the steps, there is less feedback to let you know what
  went wrong.  Despite the lack of feedback, when you take the robot on the road,
  this is frequently the only configuration method available.

The reliability/advantages/disadvantages are summarized in the table below:
<--! Use a table to present the advantages/disadvantages. -->
*{ waynegramlich: The table below needs more work. }*
<Table Border="1" Summary="Reliability/Advantages/disadvantages of Shell Prompt Methods">
  <Caption>
    <Em>Reliability/Advantages/Disadvantages of Robot Computer Shell Prompt Access Strategies</Em>
  </Caption>

  <TR>
    <TH> Strategy </TH>
    <TH> Reliability </Th>
    <TH> Advantages </TH>
    <TH> Disadvantages </TH>
  </TR>

  <TR>
    <TH RowSpan="2"> Computer/Mouse/Keyboard </TH>
    <TD RowSpan="2"> High <TD>
    <TD> </TD>
    <TD> Requires extra keyboard, mouse, and display </TD>
  </TR>
  <TR>
    <TD> Does not need desktop/laptop </TD>
    <TD> Robot needs to be opened </TD>
  </TR>

  <TR>
    <TH RowSpan="2"> Network Cable </TH>
    <TD RowSpan="2"> Medium </TD>
    <TD>Small number of steps</TD>
    <TD> Requires Internet Cable </TD>
  </TR>
  <TR>
    <TD> </TD>
    <TD> Requires Desktop/Laptop </TD>
  </TR>

  <TR>
    <TH> Robot WiFi Access Point </TH>
    <TD> Low </TD>
    <TD> No extra hardware required </TD>
    <TD> Laptop only; no desktop </TD>
  </TR>
</Table>

The three sub-sections below go through the details of each strategy of achieving
a viable shell prompt.

## Connecting to Robot Computer with Keyboard/Mouse/Display

The steps for ...

1. HDMI Display:
   Plug an HDMI display into the HDMI port of the robot computer.  Power the
   HDMI display on.

2. USB Keyboard:
   Plug a keyboard into one of the 4 USB ports of the robot computer.

3. USB Mouse:
   Plug either wired mouse or a wireless mouse into one of the four robot
   computer USB ports.

4. Micro SD Card:
   Insert the Micro SD card with URPi image on it into the Micro SD card slot on
   the robot computer.  Note that this slot is on the under side of the robot computer.

5. Power Up:
   Before performing this step, be sure the previous 4 steps have been followed.
   If the robot computer is plugged into a robot, just turn on the robot power.
   Alternatively, you can plug a USB cable between a laptop/computer to the micro USB
   connector on the robot computer.  There is no power on/off switch on the robot
   computer, once you plug it in, it will power up.  You will know that power up
   has occurred when the power LED lights up on the robot computer.

6. Display Activity:
   You should see some logging information show up on the display.  This means that
   the URPi image is good and that the robot computer is booting up the Linux operating
   system.  If you see no logging information, please power down the robot computer
   by unplugging the USB cable from the step immediately above.

7. Resizing File System:
   After about a minute, a message that says that the file system is being resized
   with show up on the display.  The file system resizing operation takes 5 to 10
   minutes.  This operation occurs exactly once and does not need to be done again.
   So relax and kill some time until this operation completes.  If the operation
   has not completed in 15 minutes, some has gone wrong and you will probably have to
   [install the URPi image](../urpi_install/urpi_install.md)
   on the Micro SD card.  If it still does not work, consider getting a different
   Micro SD card.

   <!-- Note that the section below refers to these steps.  If any steps are added
        or deleted, the section below will need to be updated.
   -->

8. Login Prompt:
   The robot computer has successfully booted when you get a prompt that says:

           ubuntu login:
     
   type the account name of `ubuntu` followed by the `[Enter]` key.  Note that the
   account name must be in lower case.  If the characters are showing up in upper
   case, please toggle the `[Caps Lock]` key and try again.

9. Password Prompt:
   The next prompt will be the password prompt that looks as follows:

           password:

   Type the password `robotseverywhere` in followed by the `[Enter]` key.
   Like most passwords, the computer will will not echo the characters.
   If you mistype either the account name or the password, you will get prompted
   with `ubuntu login:` again.  If so, repeat the previous step and this one.

10. Shell Prompt:
    The shell prompt looks as follows:

           ubuntu# 

    where `ubuntu` is the computer host name and `#` means that you are logged in
    in system administrator mode (i.e. Linux "root" mode).

Upon reaching this point, you have achieved a successful shell prompt and can move
onto the other configuration steps.

## Connecting to Robot Computer with Network Cable

1. Turn on Desktop/Laptop Computer:
   Turn on your desktop or laptop computer and get yourself to a Linux shell prompt.
   If you are running Linux under some virtualization software (e.g. VirtualBox),
   you will have to start up the virtualization software bring up some sort of
   shell window.

2. Verify Zero Conf. is Up:
   The phrase "zero conf." stands for zero-configuration networking.  We use
   zero conf. to avoid having to ask you to type in really ugly internet addresses
   like `10.0.12.34`.  Ugh!.  Instead we type in slightly less ugly internet
   machine names like `ubuntu.local`.   So, please type in the following:

           ping -c 5 `hostname`.local

   This is very important, there are two single quotes on the keyboard --
   the grave accent (`\``) and the regular single quote (`'`).  The command
   above uses the grave accent (usually on the far upper left of the keyboard.)
   You should get back a response that looks like:

          PING HOSTNAME.local (IP_ADDRESS) 56(84) bytes of data.
          64 bytes from 192.168.1.5: icmp_seq=1 ttl=64 time=0.028 ms
          64 bytes from 192.168.1.5: icmp_seq=2 ttl=64 time=0.041 ms
          64 bytes from 192.168.1.5: icmp_seq=3 ttl=64 time=0.044 ms

   where `HOSTNAME` is the name of you Linux development machine hostname
   and `IP_ADDRESS` is some sort of ugly internet address number like `10.0.12.34`.
   By the way, `PING` is just the word "ping" in upper case.  Go figure.

   If you get responses that look like `64 bytes from ...`, you have working
   zero conf. working on your desktop/laptop computer.  If you get any other
   kind of message (e.g `unknown host ...` or `Destination Host Unreachable`)
   that means that zero conf. is not working and must be fixed before resuming.
   If zero conf. is working, you can proceed to the next step.

3. Micro SD Card:
   Insert the Micro SD card with URPi image on it into the Micro SD card slot on
   the robot computer.  Note that this slot is on the under side of the robot computer.

4. Power Up:
   Before performing this step, be sure the previous 4 steps have been followed.
   If the robot computer is plugged into a robot, just turn on the robot power.
   Alternatively, you can plug a USB cable between a laptop/computer to the micro USB
   connector on the robot computer.  There is no power on/off switch on the robot
   computer, once you plug it in, it will power up.  You will know that power up
   has occurred when the power LED lights up on the robot computer.

5. File System Resizing:
   The very first time you boot your robot computer, the file system on the URPi
   image will be resized.  This take 5 to 10 minutes.  There is no real way to
   figure out when it is done other than to be patient.

6. Verify Robot Computer Zero. Conf. is Working:
   On your laptop/desktop computer, please run the following command:

           ping -c 3 ubuntu.local

   If you get back message that say `64 bytes from ...`, you have established
   an internet connection between your laptop/desktop computer and the robot
   computer.  If not, wait another 5 minutes and try again.  If it still fails,
   something is wrong.  At this point we recommend that you abandon this strategy
   and try the previous keyboard/mouse/display strategy.

7. Login to Robot Computer with Secure Shell:
   The `ssh` program sets up an encrypted connection between the desktop/laptop
   computer and the robot computer.  Please run the following command:

           ssh ubuntu@ubuntu.local

   If things work, you should get a message of the form:

           The authenticity of host 'HOSTNAME.local (#.#.#.#)' can't be established.
           ECDSA key fingerprint is SHA256:pha1/FINGERPRINT.
           Are you sure you want to continue connecting (yes/no)?

   Where `HOSTNAME` is the host name of the desktop/laptop computer,  `#.#.#.#`
   is an ugly internet address (e.g. `10.0.12.34`), ECDSA is an acronym
   for Elliptic Curve Digital Signature Algorithm, SHA256 is the name
   of a cryptographic digest algorithm, and FINGERPRINT is a bunch of
   random looking characters.  Please, just type `yes` followed by the
   `[Enter]` key.

   Next, you get the following message:

           Warning: Permanently added 'HOSTNAME.local' (ECDSA) to the list of known hosts.
           Warning: the ECDSA host key for 'onkyo.local' differs from the key for the IP address '#.#.#.#'
           Offending key for IP in /home/USER/.ssh/known_hosts:58
           Are you sure you want to continue connecting (yes/no)?

   where `HOSTNAME`, `ECDSA` are as before, IP is short for Internet Protocol,
   and USER the host/laptop user name.  Again, please just type `yes` followed
   by the `[Enter]` key.

   Next you should get something like:

           ubuntu@ubuntu.local's password: 

   Now, you type in `robotseverywhere` followed the `[Enter]` key.  Lastly,
   you should get the following welcome message:

           Welcome to Ubuntu 16.04.2 LTS (GNU/Linux 4.4.0-96-generic x86_64)
           
           * Documentation:  https://help.ubuntu.com
           * Management:     https://landscape.canonical.com
           * Support:        https://ubuntu.com/advantage

           N packages can be updated.
           M updates are security updates.

           ubuntu@ubuntu:~$ 

    Do not worry about, the numbers N and M, they change all the time.

If you get to this point you have successfully, gotten to a shell prompt on the
robot computer and can proceed to the section on robot.

## Connecting to Raspberry Robot Computer via Robot Wifi Access Point

*{ waynegramlich: This section is going to be huge, since it must document
   Windows, MacOS, and native Linux.  We may be able to merge the MacOS
   and Linux sections by bringing up a shell tool for MacOS. }*

## Using PiFi to Configure Robot Computer Wifi

*{ waynegramlich: fill in the details }*

### Changing the Robot Host Name

*{ waynegramlich: fill in the details }*

### Creating User Accounts

*{ waynegramlich: fill in the details }*

### Development Machine Networking

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
