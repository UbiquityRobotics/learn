---
layout: default
title:  "Upgrading Motor Controller Firmware"
permalink: firmware-upgrade
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

# Upgrading Motor Controller Firmware

Before you can upgrade firmware, your robot needs to be connected to the 
internet. See [Connecting the Robot to Your Network](/connect_network)  

First log into the Raspberry Pi: `ssh ubuntu@YOURROBOT.local`, all of the
following commands are to be run on the Pi.

Stop all running ROS nodes:

```
sudo systemctl stop magni-base
```

Run the firmware upgrade utility:

```
rosrun ubiquity_motor firmware_upgrade.py
```

After entering your email address, you should receive an access token.
Enter this token to start the download and upgrade process.

The process should take less than a minute, please make sure that power is
not interuptted during this process. Otherwise your robot may be permanently 
bricked.

When done, reboot the robot with `sudo reboot`.

You are now on the latest version of the firmware.

