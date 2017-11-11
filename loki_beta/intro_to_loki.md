---
layout: default
title:  "Loki Beta Testers Documentation"
permalink: intro_to_loki
---
# Loki Beta Testers Documentation.

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


To start the base node:


```cd /catkin-ws/src/ubiquity-launches/bin
./loki_base```


You then can launch teleop-twist-keyboard either locally or on a remote by

```export ROS_MASTER_URI=http://’robot ip number’ :11311
rosrun teleop-twist-keyboard teleop-twist-keyboard.py```

and drive the robot.


Time stamps are important in ros if you are working in access point mode the robot and laptop clocks will not be in sync. 
The folowing script(or one liner) will fix this:

#!/bin/sh
ssh ubuntu@10.42.0.1 sudo -S date -s @`( date -u +"%s" )`


