---
title: "Tips and Tricks"
permalink: noetic_quick_start_tips_and_tricks
group: "quick start"
rosver: noetic
nav_order: 11
nav_exclude: false
---

# Software Tips and Tricks

A collection of various tips for general software diagnostics, troubleshooting, etc.

Also see [the hardware troubleshooting page](noetic_magni_silver_diagnostics_and_troubleshooting).

## Sanity Checks

* When magni is on, the wheels should have strong resistance to movement.  

* The command `sudo systemctl status magni-base`should show that the magni-base service is up and running

* The command `rosnode list` should show `motor_node` and other nodes. The command `rostopic list` should show many ROS topics, including `/battery_state` and `/cmd_vel`.

* After the command `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`, pressing the 'i' key should move the robot.

* To change the robot's hostname (a must if you will ever have a second robot), see the instructions in [Connecting the Robot](connect_network).
Instructions for joining your local area network are in the same place.

* To check the battery level type: `rostopic echo /battery_state`

  This should show you various information about the battery. If the battery voltage is somewhere between 21-28V that would be normal. Obviously the lower the voltage the less charge the batteries have.

## Handy Tips for Developers

* The main config file for enabling the raspicam camera and sonars and more can be edited as root and is `/etc/ubiquity/robot.yaml`  Details on this are located in the sections discussing camera and sonars and other sections. Beware that the format is very specific. Do not use tabs and pay attention to where spaces are required.

* Another important configuration file `base.yaml` is found at: `/opt/ros/$ROS_DISTRO/share/magni_bringup/param/base.yaml`. N.B. This file will be rewritten when the base is upgraded, as by apt=get.  The most likely things a user would change are being moved to the above robot.yaml file if not already there.

* To find the firmware version while the system is running run this then stop
    `rostopic echo /diagnostics`
  In the busy output the firmware revision and if recent code the daycode will be in the /diagnostics topic

* To disable magni startup: `sudo systemctl disable magni-base` and reboot so magni-base will be inactive.

    To get back to normal behavior you will need later a `sudo systemctl enable magni-base` and then a reboot.   

* To find the firmware version when  magni-base is stopped
    `rosrun ubiquity_motor probe_robot -f`  
    For even more fun try **-a** instead of **-f**.

    If the robot is running a program, run  
    `sudo systemctl disable magni-base.service`  
    and then reboot before trying the probe_robot command again.

    To return the robot to normal startup you will then need to use   `sudo systemctl enable magni-base.service` and reboot again.  

    To find the version number of the most important ubiquity software, type:

    `dpkg-query --showformat='${Package}\t${Version}\n' --show ros-$ROS_DISTRO-magni-robot ros-$ROS_DISTRO-ubiquity-motor ros-$ROS_DISTRO-fiducials ros-$ROS_DISTRO-raspicam-node ros-$ROS_DISTRO-pi-sonar`

*  To run magni services after a 'systemctl disable' line as shown above:    `roslaunch magni_bringup base.launch`.

* The launch file used by the magni-base.service is `/opt/ros/$ROS_DISTRO/share/magni_bringup/launch/base.launch`.  
This launch file does `rosrun controller_manager spawner ubiquity_velocity_controller ubiquity_joint_publisher`.  
This invokes the ROS controller_manager; for more detail see `wiki.ros.org/controller_manager`.

* Much magni software will be found in ```/opt/ros/$ROS_DISTRO/share/magni_*``` In particular, magni_demos/launch has ROS launch files.

## Rviz Troubleshooting

Some users have reported that, when running on a virtual machine workstation, it is necessary to turn off hardware acceleration.

## Finding Robot Firmware Version Info

Should you have to get back to report behavior issues with the robot where the robot can move that means a great deal of the system is operational.  It is important to report to the support group the firmware version and its date which is done as follows if the host Raspberry Pi cpu is able to communicate with the main MCB board.

    rostopic echo /diagnostics | grep -A 1  'Firmware [DV]'

Report the version of the MCB main board if any tests indicate it has a fault.

## Troubleshooting Lack Of Robot Movement

The most basic way to do a test to verify the robot can move is to be on an SSH console window on the robot and see if you can use keyboard to control the robot at all where ```Forward``` is just tapping the ```i``` key.  

    rosrun teleop_twist_keyboard telelop_twist_keyboard.py

Before we dig into some detailed troubleshooting below on lack of movement be aware you can also review some key high level reasons for lack of movement that we had in a post on our forum from some time back that you should review just in case it is one of these issue then this page will elaborate.    

Feel free to read that post if this page does not resolve your problem quickly.
Older troubleshooting post:  [Magni Does Not Move issue on our forum](https://forum.ubiquityrobotics.com/t/magni-does-not-move/98)  

## Guidelines for Usage Of The I2C Bus

#### The I2C devices Ubiquity Robotics reserves:

Addresses given in 7-bit form so on the I2C bus they appear shifted up by 1 bit.

| | |
|---|---|
|Device| I2C Address|
|SSD1306 OLED Display|0x3C
|PCF8574|0x20|
|MCP7940 RT Clock|0x6f|

 * If you stop Magni with `sudo systemctl stop magni-base.service` you can scan the I2C bus for all devices using `sudo i2cdetect -y 1` if i2c-tools was installed using apt-get. The i2c-tools package also has `i2cget` and `i2cset` so look them up if curious.

 * The OLED display was only added to shipment boards as of MCB version 5.2 but was possible to load starting from version 5.0 boards.

* Because the MCP7940 is owned by the kernel the i2cdetect tool will show it at address 0x6F as  ```UU```.  This indicates it was seen.  If the kernel recognized the RTC properly there will be a  /dev/rtc0 device for final confirmation.

* If there is a production issue where a PCF8574A was incorrectly purchased then you would see address 0x38.  This is considered a misload in production.


#### Tips and Guidelines For any I2C usage On the Magni Platform:

The I2C is the main 3.3V Raspberry Pi I2C on pins 3 and 5 with Raspberry Pi as the master.
The Rev 5.0 board has a 4-pin jack that brings out I2C and 3.3V with a ground.

* Use the I2C ONLY from within a ROS node to avoid conflict on the I2C bus.
* Keep the I2C lines to your device under 60mm from board to your device
* Only use devices for short data accesses and space out your accesses by at least 50msec