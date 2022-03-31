---
layout: default2.
title:  "LOKI Beta software"
permalink: loki_software
---
Starting from a standard Ubiquity Magni Image:

Set up as a standard Magni Image and then make the following changes:

1. Attach to a network following the quickstart instructions.
2. Run `sudo systemctl disable magni-base` to stop the Magni services from starting at boot.
3. Add the github 'loki-base' file to /usr/sbin. 
`sudo wget https://raw.githubusercontent.com/UbiquityRobotics/learn/master/quick_start/loki/loki-base -O /usr/sbin/loki-base`
4. Make the file executable `sudo chmod +x /usr/sbin/loki-base`
5. Create a file `/etc/systemd/system/loki-base.service` with the following contents
```
[Unit]
After=NetworkManager.service time-sync.target
[Service]
Type=simple
User=ubuntu
ExecStart=/usr/sbin/loki-base
[Install]
WantedBy=multi-user.target
```
6. Run `sudo systemctl enable loki-base`
7. sudo reboot

If everything is correct,  rostopic list should show sonar topics being echoed.  Keyboard teleop (either direct or 
from a workstation, once ROS_MASTER_URI is exported on a workstation, RVIZ should work.

To get speech to work, edit the magni-demos speech_commands launch file and remove the lines refering to magni_base  and teleop/

