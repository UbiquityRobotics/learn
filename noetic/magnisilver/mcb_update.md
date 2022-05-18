---
title: "Firmware Upgrade (MCB)"
permalink: noetic_magnisilver_mcb_upgrade
group: "magni silver (gen 5)"
rosver: noetic
nav_order: 4
nav_exclude: false
--- 

# Firmware Upgrade

Sometimes you may need to get a firmware upgrade for your Motor Controller board, i.e. the MCB.
This operation is less frequently needed and in general we advise customers to do the firmware upgrade only if they run into an issue we know we have addressed in an update.

Before you can upgrade firmware, your robot needs to be connected to the
internet or you must have the firmware file and copy it to the robot.  See [Connecting the Robot to Your Network](noetic_quick_start_connecting).

We upgrade firmware using a tool that will by default install the latest released firmware. If an older fimware version is needed, it will require manual version entry seen in the Version column of the table below.

Improvements to out motor controller fimware (v35 and later) need the latest Linux host side software. Installing the latest Linux host side software can be done by following a [guide here](noetic_quick_start_microsd).

##### See: [Motor Control Board (MCB)](noetic_magnisilver_mcb) for more info about the MCB.

##### See: [Firmware Revision History](https://learn.ubiquityrobotics.com/noetic_magnisilver_mcb_revisions#firmware-revisions).

## Installation
To install any firmware first log into the Raspberry Pi:

    ssh ubuntu@YOURROBOT.local

all of the following commands are to be run on the Pi.  

## First prevent the motor_node from talking to the MCB

There are now two ways to prevent the ROS motor_node from constant queries and commands to the MCB board over the serial connection.  
For a standard Magni configuration it is best to completely stop the Ubiquity Software and then do the firmware upgrade.
As of our motor_node available as of April 2021 you can also do a new method that does not shutdown everything but only impacts the motor node by a disable or relinquish of the serial port to allow direct serial control which in this case would be for the firmware upgrade.   We suggest for a standard Magni use the full system stop.

### Full stop of the Magni software is the best method

The following command fully stops standard Magni software but may not fully stop other configurations for some new Magni applications to be announced in 2021

    sudo systemctl stop magni-base

Once you have completed a firmware upgrade you are best off to reboot when using this method.   

    sudo shutdown -r now

### A soft disable of motor_node control of the MCB is now possible
As of about April 2021 users who have very current ubiquity_motor repository code are able to do a softer stop of the motor node and then later re-enable the motor node.   We suggest use of the prior method but show this to be complete.

    rostopic pub /system_control std_msgs/String "motor_control disable"

After the firmware upgrade you may still just reboot OR use this command

    rostopic pub /system_control std_msgs/String "motor_control enable"

## After the MCB is free over serial do the upgrade_firmware

You can use the firmware upgrade utility that must have web access to fetch the most current released firmware OR you can load a version you have fetched yourself and placed in a file.

### Upgrading firmware from the web if you have an internet connection
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

### Firmware installation from a file

In some support situations you may be working with the development team here and be given a beta version of software in the form of an encrypted file.

Upgrade Firmware from a file that you place on your system:

    rosrun ubiquity_motor upgrade_firmware.py --file  v40_20201209_enc.cyacd

The name above is an example file for released v40 firmware. For beta firmware if you are told to use a given version then you may not be able to request a beta version so you would have to check our please visit  [OUR REPOSITORY](https://github.com/UbiquityRobotics/ubiquity_motor)  and navigate into the firmware folder.

As of mid 2021 we have decided on a naming convention for our firmware which will start the name with   vXX_YYMMDD  where XX is the firmware rev and YYMMDD is they daycode.  We use a daycode so that we can have 'beta' or early release tests.  When we do have a beta firmware we will try to end the name in  beta and may say other text.  So the vXX_YYMMDD is the only fixed format specified but other characters may be after that format.   

### Firmware installation using non-standard serial support

In some situations the serial port used for controlling the robot may not be the default port on the Raspberry Pi host computer attached to the MCB.

To specify a different serial port such as one plugged into usb:

    rosrun ubiquity_motor upgrade_firmware.py --device /dev/ttyUSB0
