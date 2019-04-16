---
layout: default
title:  "Updating Software and Firmware"
permalink: updating
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)
# Updating Software and Firmware

From time to time Ubiquity Robotics will be updating
either the full Raspberry Pi image, the working software
on the Raspberry Pi, or the firmware on the Motor Controller Board.

#### Raspberry Pi Image Update

To check or update the
full Raspberry Pi image, visit the [Ubiquity Robotics Download](https://downloads.ubiquityrobotics.com/pi.html) page.    Updating a full image will lead to loss of any customization you have done.  Never destroy your old card till you know you have everything on the new image.


#### Linux Host Software Update

More frequently you may want to just update the software on the Linux host computer.
Use  apt-get as follows to update any new Linux software for Magni:

On your workstation, start a terminal window (Linux shortcut: ctrl-alt-t). In that window, log in by typing:

```ssh ubuntu@ubiquityrobot.local```
(use the robot’s name or IP address)

```sudo apt-get update```  
```sudo apt-get upgrade```

This may take some time, since it may have been a while since the original image was made.


#### Motor Controller Firmware Upgrade

Sometimes you may need to get a firmware upgrade for your
Motor Controller board.
This operation is less frequently needed and in general we advise customers to do the firmware upgrade only if they run into an issue we know we have addressed in an update.

To get the firmware upgrade follow
the process on [the firmware-upgrade page](https://learn.ubiquityrobotics.com/firmware-upgrade).
