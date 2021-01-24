---
layout: default
title:  "Updating Software and Firmware"
permalink: updating
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)
# Burning A Fresh Image And Updating Software

From time to time Ubiquity Robotics will be updating
either the full Raspberry Pi image, the working software
on the Raspberry Pi. This page also contains a link to the page for firmware on the Motor Controller Board.

### Raspberry Pi Image Update

To check or update the
full Raspberry Pi image, visit the [Ubiquity Robotics Download](https://downloads.ubiquityrobotics.com/pi.html) page.    Updating a full image will lead to loss of any customization you have done.  Never destroy your old card till you know you have everything on the new image.

#### Raspberry Pi 4 Supported on MCB Version 5.2 or later

In order to use the more powerful Raspberry Pi 4 on the robot you will needs to have an MCB (Main Control Board) of version 5.2 or later.  The version for rev 5.2 and beyond is in bright white large silkscreen on the left edge of the MCB board.  Earlier versions of the MCB came out before the Pi4 introduction so we have seen 3.3V power related issues that often lead to things like the network not working and in some cases failure to boot up the Pi4.  

#### Known Raspberry Pi Image Issues Of Note

Here is a list of some key issues or limitations of the raspberry Pi images that are known to effect customers. We will just show the datecode part of the image name.

|  DateInName | Issue Or Limitation |
|-------------------------|----------------------|
|  2020-11-07 |  Does not run Sonar board when CPU is a Pi4 |
|  2020-02-10 |  Ethernet and Sonars do not work when CPU is a Pi4 |
|  2019-06-19 |  Supports Pi3 but came before Pi4 support |


### Linux Host Software Update

More frequently you may want to just update the software on the Linux host computer.
Use  apt-get as follows to update any new Linux software for Magni:

On your workstation, start a terminal window (Linux shortcut: ctrl-alt-t). In that window, log in by typing:

```ssh ubuntu@ubiquityrobot.local```
(use the robot’s name or IP address)

```sudo apt-get update```  
```sudo apt-get upgrade```

This may take some time, since it may have been a while since the original image was made.


### Motor Controller Firmware Upgrade

Sometimes you may need to get a firmware upgrade for your
Motor Controller board.
This operation is less frequently needed and in general we advise customers to do the firmware upgrade only if they run into an issue we know we have addressed in an update.

To get the firmware upgrade follow
the process on [the firmware-upgrade page](https://learn.ubiquityrobotics.com/firmware-upgrade).
