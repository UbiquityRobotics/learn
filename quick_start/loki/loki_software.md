---
layout: default2.
title:  "LOKI Beta software"
permalink: loki_software
---

## How to build a Loki SD Image

Since the release of the January image, Loki nodes are included, however since the documentation as of 01-12-2018 has not been updated, this documentation explains how to start the nodes. Also the Virtual Box image does not have the loki nodes yet, so these instructions may still be valid for a non-Raspberry Pi system.

Sarting from a standard Ubiquity Magni Image:

Set up as a standard Magni Image and then make the following changes:

1. Attach to a network following the quickstart instructions.
2. Via sftp  or 'get'  download the two *.tar.gz files located in the github 'loki' directory to catkin_ws/src
   
   wget https://raw.githubusercontent.com/UbiquityRobotics/learn/master/quick_start/loki/bus_server.tar.gz
                                                                                        ubiquity_launches.tar.gz

3. Expand via tar -xzf 

      (or by gunzip   then  tar -xf
 
4. cd ~/catkin_ws  &  catkin_make
5. Move the github 'loki-base' file to /usr/sbin
6. Edit the /etc/systemd/system file magni-base file and change /usr/sbin/magni-base to /usr/sbin/loki-base
7. sudo reboot

If everything is correct,  rostopic list should show sonar topics being echoed.  keyboard teleop (either direct or 
from a workstation, one ROS_MASTER_URI is exported should work.

To get speech to work, edit the magni-demos speech_commands launch file and remove the lines refering to magni_base  and teleop/

