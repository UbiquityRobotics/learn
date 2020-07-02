---
layout: default
title:  "Upgrading Motor Controller Firmware"
permalink: firmware-upgrade
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

# Upgrading Motor Controller Firmware

Before you can upgrade firmware, your robot needs to be connected to the
internet. See [Connecting the Robot to Your Network](/connect_network)

We upgrade firmware using a tool that will by default install the latest released firmware. If a newer version is available it will require manual version entry seen in the Version column of the table below.

Sometimes improvements to our motor controller firmware, in this case v35 or later, make it necessary to first upgrade the linux host side software before installing the newest firmware.  Our host side software supports prior firmware versions.  To update host software use a process we call 'Linux Host Software Update' seen on [this page](https://learn.ubiquityrobotics.com/updating).

## Firmware Release Versions

The table below shows the default and latest available versions of firmware.  

Also shown is the rate at which the MCB ```STAT``` led will blink which is handy to visually check. It is best to count the time of 4 or more blinks then divide by that number for more accuracy. If the blink rate is found to be other than in the table it is possible you have a beta or non-approved version that may have been recalled.

| Version |	BlinkRate | Description |
| ------- | ---- | ----------- |
| v28 | 6.0 sec | Production shipment version that does wheel movement check on startup. Users should do a firmware upgrade from this very old version |
| v32	| 5.0 sec | Production firmware used in 2019. This is default if no version is entered. Supported by any host side software |
| v35 |	4.0 sec | Production firmware with double resolution wheel encoders and many improvements.    Requires host side software update done after 11/10/2019 |
| v37 | 4.75 sec | Beta firmware. Has built in selftest. |

To see more details about our firmware as well as our hardware revisions for the motor controller please view [this page](https://github.com/UbiquityRobotics/ubiquity_motor/blob/kinetic-devel/Firmware_and_Hardware_Revisions.md).

## Checking ROS /diagnostics Topic For Firmware

The firmware version as well as the firmware release date are published by the motor_node to ROS topic  ```/diagnostics```

Use the following command for a couple seconds then use Control-C to stop the fast moving output.  Scroll back and look for ```"Firmware Version"``` and ```"Firmware Date"```

    rostopic echo /diagnostics


## Firmware Installation
To install any firmware first log into the Raspberry Pi:

    ssh ubuntu@YOURROBOT.local

all of the following commands are to be run on the Pi.  

Stop all running ROS nodes:

```
sudo systemctl stop magni-base
```
Run the firmware upgrade utility:

    rosrun ubiquity_motor upgrade_firmware.py

After entering your email address, you should receive an access token.
Enter this token and either accept the default version which is seen in the above table or specify a different version perhaps for latest features.  The download will start and upgrade the motor controller board.

Should any problems show up you are always allowed to do this firmware upgrade again to select a version you know was best for your needs.

The process should take less than a minute; make sure that power is
not interrupted during this process. Otherwise your robot may become
inoperable and require factory servicing.

When done, reboot the robot with `sudo reboot`.

You are now on the latest version of the firmware.
