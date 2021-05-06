---
layout: default
title:  "Upgrading Motor Controller Firmware"
permalink: firmware-upgrade
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

# Upgrading Motor Controller Firmware

Before you can upgrade firmware, your robot needs to be connected to the
internet or you must have the firmware file and copy it to the robot.  See [Connecting the Robot to Your Network](/connect_network).

We upgrade firmware using a tool that will by default install the latest released firmware. If an older fimware version is needed, it will require manual version entry seen in the Version column of the table below.

Improvements to out motor controller fimware (v35 and later) need the latest Linux host side software. Installing the latest Linux host side software can be done by following a process we call 'Linux Host Software Update' seen on [this page](https://learn.ubiquityrobotics.com/updating).

## Finding The Firmware Version Info For Your Robot

Here is how to get firmware version and date once you have opened a SSH window into your robot.  This works by filtering off just the 2 items of interest which are ```Firmware Version```  and  ```Firmware Date```

    rostopic echo /diagnostics | grep -A 1  'Firmware [DV]'

## Firmware Release Versions

The table below shows the default and latest available versions of firmware.  

The ```Rate``` in the table is the rate at which the MCB ```STATUS``` led will blink which is handy to visually check. It is best to count the time of 4 or more blinks then divide by that number for more accuracy. If the blink rate is found to be other than in the table it is possible you have a beta or non-approved version.

DateCode was started around version v35 and shows up in /diagnostics topic along with the version.  DateCode is in YYYYMMDD format for releases and YYMMDD for beta releases. It is the date of that particular version.  If you have a version that the date is before the date in the table it is likely a Beta or unofficial release. DateCode did not start till around v35.

| Ver |	Rate | DateCode | Description |
| ------- | ---- | ---- | ----------- |
| v28 | 6.0 | NA | Depreciated Production shipment version that does wheel movement check on startup. Users should do a firmware upgrade from this very old version |
| v32	| 5.0 | NA | Depreciated Production firmware used in 2019.  |
| v35 |	4.0 | 20190815 | Last well known good release as of Sept 2020 with double resolution wheel encoders and many improvements.    Requires host side software update done after 11/10/2019 |
| v37 | 4.75 | 20200620 | Use in manufacturing only.  1st with a selftest. |
| v38 | 5.25 | 20201006 | Depreciated Beta release for some fixes. |
| v39 | 5.5 | 20201129 | Non-Magni release for in development 4wheel drive unit |
| v40 | 5.75 | 20201209 | Magni current release for most recent release candidate.  This is the default if just an enter is done when asking for version |

To see more details about our firmware as well as our hardware revisions for the motor controller please view [this page](https://github.com/UbiquityRobotics/ubiquity_motor/blob/kinetic-devel/Firmware_and_Hardware_Revisions.md).

## Checking ROS /diagnostics Topic For Firmware

The firmware version as well as the firmware release date are published by the motor_node to ROS topic  ```/diagnostics```

Use the following command for a couple seconds then use Control-C to stop the fast moving output.  Scroll back and look for ```"Firmware Version"``` and ```"Firmware Date"```

    rostopic echo /diagnostics


## Firmware Installation
To install any firmware first log into the Raspberry Pi:

    ssh ubuntu@YOURROBOT.local

all of the following commands are to be run on the Pi.  

### First Prevent The motor_node From Talking to the MCB

There are now two ways to prevent the ROS motor_node from constand query and commands to the MCB board over the serial connection.  
For a standard Magni configuration it is best to completely stop the Ubiquity Software and then do the firmware upgrade.
As of our motor_node available as of April 2021 you can also do a new method that does not shutdown everything but only impacts the motor node by a disable or relinquish of the serial port to allow direct serial control which in this case would be for the firmware upgrade.   We suggest for a standard Magni use the full system stop.

#### Full Stop Of the Magni Software If Best Method

The following command fully stops standard Magni software but may not fully stop other configurations for some new Magni applications to be announced in 2021

    sudo systemctl stop magni-base

Once you have completed a firmware upgrade you are best off to reboot when using this method.   

    sudo shutdown -r now

#### A soft disable of motor_node control of the MCB Is Now Possible
As of about April users who have very current ubiquity_motor repository code are able to do a softer stop of the motor node and then later re-enable the motor node.   We suggest use of the prior method but show this to be complete.

    rostopic pub /system_control std_msgs/String "motor_control disable"

When using this method you may after the firmware upgrade either reboot or used

    rostopic pub /system_control std_msgs/String "motor_control enable"

### After The MCB is Free over serial do the upgrade_firmware

You can use the firmware upgrade utility that must have web access to fetch the most current released firmware OR you can load a version you have fetched yourself and placed in a file.

#### Upgrading Firmware From The Web If You Have A connection.
To use this method you would generally connect the robot to your own WiFi or use an ethernet cable into the Pi to your own network.  

Run the firmware upgrade utility:

    rosrun ubiquity_motor upgrade_firmware.py

After entering your email address, you should receive an email with an access token.
Enter this token and either accept the default version which is seen in the above table or specify a different version perhaps for latest features or perhaps an older trusted release (eg. specify version 35 by typing in 'v35'). The download will start and upgrade the motor controller board.

Should any problems show up you are always allowed to do this firmware upgrade again to select a version you know was best for your needs.

The process should take less than a minute; make sure that power is
not interrupted during this process. Otherwise your robot may become
inoperable and require factory servicing.

When done, reboot the robot with `sudo shutdown -r now` if you used

You are now on the latest version of the firmware.

#### Firmware Installation From A File

In some support situations you may be working with the development team here and be given a beta version of software in the form of an encrypted file.

Upgrade Firmware from a file that you place on your system:

    rosrun ubiquity_motor upgrade_firmware.py --file  betaFirmwareFileName

For beta firmware if you are told to use a given version then you may not be able to request a beta version so you would have to check our please visit  [OUR REPOSITORY](https://github.com/UbiquityRobotics/ubiquity_motor)  and navigate into the firmware folder.

#### Firmware Installation Using Non Standard Serial support

In some situations the serial port used for controlling the robot may not be the default port on the Raspberry Pi host computer attached to the MCB.

To specify a different serial port such as one plugged into usb:

    rosrun ubiquity_motor upgrade_firmware.py --device /dev/ttyUSB0
