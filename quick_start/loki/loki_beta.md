---
layout: default
title:  "LOKI Beta startup"
permalink: loki_beta
---

## Getting Started with Loki for Beta Testers

Loki is a ROS controlled differential drive robot designed for educational purposes by Ubiquity Robotics.
As such, it has code that is in many cases identical to our general purpose Magni Platform. A program written to navigate a
one meter square for Loki wil navigate an Identical one meter square for Magni

To get started, one needs to attach a Raspberry Pi 3 with an SD card image supplied by Ubiquity Robotics.
It is possible to intall the PI incorrectly which will result in a "fried Pi". Be careful!

![Fried Pi](loki_rpifatal.jpg)

The Rpi 3 pulls over 3 amps, so be sure you supply at least 2.5 amps (no camera) or 3 amps if you are using an Raspicam.
using USB accessories is not recommnded unless addional power is provided.

![Loki Top View](loki_top1.jpg)

Make sure (For rev F boards) that the J2 and J5 (circled) jumpers are installed. The robot is powered by either a switch or a jumper. When activated, initially all the LEDS on the robot should go green. If you then turn the wheels, they should turn on and off. If this doesn't happen, the board needs to have its firmware re-installed.

Up to 16 sonar sensors can optionally be installed for obstacle avoidance. These are not intended for gmapping. As of this writing, the optional raspicam is set to use ceiing mounted fiducial markers, and this may be changed for the final release.
Also, we haven't finished the final SD card image for Loki, so for now the networking and connecting sections for our Magni
robot are the best source of instructions.


## Starting the Loki

Depending on the type of battery supplied, the Loki will be turned on via the power switch or jumper. Some of the 5v LiPo battery packs do not sense the switch, and need to be powered on manually. When the Loki turns on both the Rpi red LED and all the Loki LEDs should light. If firmware is working the LEDs should blink when you turn the wheels. The wheels should be able to turn freely.

## Connecting to the Loki.

When the Loki boots in a new environment, it will attempt to connect to know WiFi networks. If it can’t find any known networks it will open up a WiFi access point in a couple of minutes. To connect to the access point use your network icon and Search for “UbiquityRobot####” the “####” are the last two unique hexadecimal digit of the WiFi hardware MAC address.  The password it “robotseverywhere”  The robots IP number is 10.42.0.1.

To logon to the robot:

```ssh ubuntu@10.42.0.1`````

(password = “ubuntu”)

You can use the robot in Access Point mode, or you can connect to your local area network via the 
following commands:

```pifi list seen```

```sudo pifi add  ssid  password```   (Where ssid password or for the LAN)

Then reboot, and discover the new network IP number of your robot either by looking at your LAN’s connected 
devices list, or  trying to ping ubiquityrobot####@local.

You then should be able to ssh into your robot.


To test robot operation you can use minicom:

```sudo minicom (-D /dev/ttyAMA0)```

  e <cr>  shows encoders

m 100 100  <cr>   both wheels forward

m 0 0 <cr>      full stop.


Assuming you can connect to the robot via ssh, you can test the robot by seeing if you can teleop.
First determine if any nodes are running by typing:

```
rostopic list 
```

if not, you can start the base controller by:

```cd catkin_ws/src/ubiquity_launches/bin/```

You then can launch teleop-twist-keyboard either locally or on a remote by

```export ROS_MASTER_URI=http://’robot ip number’ :11311
rosrun teleop-twist-keyboard teleop-twist-keyboard.py```

and drive the robot.


Time stamps are important in ros if you are working in access point mode the robot and laptop clocks will not be in sync. 
The folowing script(or one liner) will fix this:

#!/bin/sh
ssh ubuntu@10.42.0.1 sudo -S date -s @`( date -u +"%s" )`
./loki_base 
```

## Rviz

To run Rviz on your laptop, you need to make sure your networking between the laptop and robot is working correctly and the clocks are synchronized.

On the robot make sure ROS_IP  and ROS_MASTER_URI are set to IP number, not localhost. on the laptop make sure the ROS_MASTER_URI matches the robot. The catkin_ws/src/ubiquity_launches files need to be installed on both systems.

```
cd ~/catkin_ws/src/ubiquity_launches/bin/
./loki_rviz_sonar 
```

will bring up rviz and show any sonars if present.

If you've gotten this far, pour yourself an adult beverage, do your victory dance, and be proud of yourself!


![Loki RVIZ](loki_rviz.jpg)

You now can start experimenting with writng your own scripts, or working with published tutorials to improve your mastery. Perhaps even work on some HBRC challenges. All Beta testers are encoraged to submit code changes and comments to the Github.

Thanks for being a Beta Loki tester





