---
layout: default
title:  "LOKI Beta startup"
permalink: loki_beta
---

## Getting Started with Loki for Beta Testers

### Note: since the release of the January SD card image, this documentation is no longer accurate. We are working to bring it up to date.

Loki is a ROS controlled differential drive robot designed for educational purposes by Ubiquity Robotics.
As such, it has code that is in many cases identical to our general purpose Magni Platform. A program written to navigate a
one meter square for Loki wil navigate an identical one meter square for Magni. Loki is designed to be a fully capable ROS robot at the absolutely lowest price possible. It allows those with extremely limited budgets to learn how to program a real robot. Of course, there are some limitations. It is dificult to add sensors because of limited power. There are no cliff detectors, so running on a table exposes the robot to a fall. The soft rubber tires can make ugly scuff marks on smooth painted surfaces. Running on a 1M by 1M sheet of plywood or foam board is recommended.

To get started, one needs to attach a Raspberry Pi 3 with an SD card image supplied by Ubiquity Robotics.
It is possible to intall the Pi incorrectly which will result in a "fried Pi". Be careful! The picture below shows one wrong way to do it. It is also possible to insert to the left or right of the socket. On flat mounted RPi's use the standoffs and be sure to attach with screws.


![Fried Pi](loki_rpifatal.jpg)

The Rpi 3 pulls over 2 amps, so be sure you supply at least 2.5 amps (no camera) or 3 amps if you are using an Raspicam.
using USB accessories is not recommnded unless addional power from a second battery is provided.  

![Loki Top View](loki_top1.jpg)

Make sure (For rev F boards) that the J2 and J5 (circled) jumpers are installed. The robot is powered by either a switch or a jumper. When activated, initially all the LEDS on the robot should go green. If you then turn the wheels, they should turn on and off. If this doesn't happen, the board needs to have its firmware re-installed.

Up to 16 sonar sensors can optionally be installed for obstacle avoidance. These are not intended for gmapping. As of this writing, the optional raspicam is set to use ceiling mounted fiducial markers, and this may be changed for the final release.
Also, we haven't finished the final SD card image for Loki, so for now the networking and connecting sections for our Magni
robot are the best source of instructions.


## Starting the Loki

Depending on the type of battery supplied, the Loki will be turned on via the power switch or jumper. (Note: some of the 5v LiPo battery packs do not sense the switch, and need to be powered on manually.) When the Loki turns on both the RPi red LED and all the Loki LEDs should light. If firmware is working the LEDs should blink when you turn the wheels. The wheels should be able to turn freely.


## Connecting to the Loki.

Loki uses basically the same software as Magni, so the sections on networking in this Qiuckstart section should apply. If the instructions below fail to work, see the sections on connecting to the robot for the first time. 
When the Loki boots in a new environment, it will attempt to connect to known WiFi networks. If it can’t find any known networks it will open up a WiFi access point in a couple of minutes. To connect to the access point use your network icon and Search for “UbiquityRobot####” the “####” are the last two unique hexadecimal digit of the WiFi hardware MAC address.  The password it “robotseverywhere”  The robot's default IP number is 10.42.0.1.

To logon to the robot:

```ssh ubuntu@ubiquityrobot.local```

or if that doesn't work:

```ssh ubuntu@10.42.0.1```

(password = “ubuntu”)

You can use the robot in Access Point mode, or you can connect to your local area network via the 
following commands:

(Useful hint:  Though almost all local networks support DHCP, rarely, it may not work. In this case helps to use IP numbers instead of names.  Do this by including the following lines in my .bashrc Ubuntu startup script:

   ip="$(hostname -I|cut -d ' ' -f 1)"
   export ROS_IP=$ip

## Testing without a LAN

It is possible to test basic operation of the robot without a network, but for running RVIZ, a network is needed.
Keyboard teleop  and speech commands are available without a network, but it recommended you connect your robot to a network first, so that you can run system updates and upgrades.


## Connecting to a Local Area WiFi Network on the Robot

```pifi list seen```

```sudo pifi add  ssid  password```   (Where ssid password or for the LAN)

Then reboot, and discover the new network IP number of your robot either by looking at your LAN’s connected 
devices list, or  trying to ping ubiquityrobot####@local.

You then should be able to ssh into your robot.


To test robot operation you can use minicom(may not be installed on all distributions):

```sudo minicom (-D /dev/ttyAMA0)```

  e <cr>  shows encoders

m 100 100  <cr>   both wheels forward

m 0 0 <cr>      full stop.


Assuming you can connect to the robot via ssh, you can test the robot by seeing if you can teleop.
First determine if any nodes are running by typing:

```
rostopic list 
```

Depending on your sd card release, rostopics may show the robot as being a magni (no sonar data). If this is the case,
stop ROS by:

```pkill roslaunch```

Now you can start the base controller by:

```roslaunch loki_bringup base.launch >/dev/null 2>&1 ```
 
 (January 13 or later distribution) or by


```
cd catkin_ws/src/ubiquity_launches/bin/

./loki_base
```
Older distribution or if built in custom catkin_ws.

if it is working rostopic list looks like this:

```
/bus_server/sensor/sonar_0
/bus_server/sensor/sonar_1
/bus_server/sensor/sonar_10
/bus_server/sensor/sonar_11
/bus_server/sensor/sonar_12
/bus_server/sensor/sonar_13
/bus_server/sensor/sonar_14
/bus_server/sensor/sonar_15
/bus_server/sensor/sonar_2
/bus_server/sensor/sonar_3
/bus_server/sensor/sonar_4
/bus_server/sensor/sonar_5
/bus_server/sensor/sonar_6
/bus_server/sensor/sonar_7
/bus_server/sensor/sonar_8
/bus_server/sensor/sonar_9
/cmd_vel
/joint_states
/odom
/rosout
/rosout_agg
/tf
/tf_static
```


You then can launch teleop-twist-keyboard either locally or on a remote by

```
export ROS_MASTER_URI=http://’robot ip number’ :11311
rosrun teleop-twist-keyboard teleop-twist-keyboard.py
```

and drive the robot.


Time stamps are important in ros if you are working in access point mode the robot and laptop clocks will not be in sync. 
The folowing script(or one liner) will fix this:
```
#!/bin/sh
ssh ubuntu@10.42.0.1 sudo -S date -s @`( date -u +"%s" )
./loki_base 
```

## Rviz

To run Rviz on your laptop, you need to make sure your networking between the laptop and robot is working correctly and the clocks are synchronized.

On the robot make sure ROS_IP  and ROS_MASTER_URI are set to IP number, not localhost. on the laptop make sure the ROS_MASTER_URI matches the robot. The catkin_ws/src/ubiquity_launches files need to be installed on both systems.

```
rosrun rviz rviz
```

The loki.rviz  file can be downloaded from here: 

wget https://raw.githubusercontent.com/UbiquityRobotics/learn/master/quick_start/loki/loki.rviz

will bring up rviz and show any sonars if present.

If you've gotten this far, pour yourself an adult beverage, do your victory dance, and be proud of yourself!

![Loki RVIZ](loki_rviz.jpg)

You now can start experimenting with writng your own scripts, or working with published tutorials to improve your mastery. Perhaps even work on some HBRC challenges. All Beta testers are encoraged to submit code changes and comments to the Github. We know a lot of work needs to be done on the code, particularly on the robot model.

I have tested the Ubiquity Robotics speech command app successfully on Loki. To do this I edited the magni_nav speech_commands.launch file on the robot, and used the Robot Commander App on my Android phone. Fiducial markers are being detected, but waypoint navigation does not work yet, due to incorrect information in the robot model.

As a next step try running some ROS tutorials.  I have had some success runing the programs from Patrick Goebel's ROS by Example E-book. I believe the Indigo code is still available via Github at PiRobot(SP?).

Thanks for being a Beta Loki tester





