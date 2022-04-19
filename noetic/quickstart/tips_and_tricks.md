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

## Sanity Checks
* When magni is on, the wheels should have strong resistance to movement.  

* The command `sudo systemctl status magni-base`should show that the magni-base service is up and running

* The command `rosnode list` should show `motor_node` and other nodes. The command `rostopic list` should show many ROS topics, including `/battery` and `/cmd_vel`.

* After the command `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`, pressing the 'i' key should move the robot.

* To change the robot's hostname (a must if you will ever have a second robot), see the instructions in [Connecting the Robot](connect_network).
Instructions for joining your local area network are in the same place.


* To check the battery level type: `rostopic echo /battery`

  This should show you various information about the battery. If the battery voltage is somewhere between 21-28V that would be normal. Obviously the lower the voltage the less charge the batteries have.

## Handy Tips for Developers

* The main config file for enabling the raspicam camera and sonars and more can be edited as root and is `/etc/ubiquity/robot.yaml`  Details on this are located in the sections discussing camera and sonars and other sections. Beware that the format is very specific. Do not use tabs and pay attention to where spaces are required.

* Another important configuration file `base.yaml` is found at: `/opt/ros/kinetic/share/magni_bringup/param/base.yaml`. N.B. This file will be rewritten when the base is upgraded, as by apt=get.  The most likely things a user would change are being moved to the above robot.yaml file if not already there.

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

    `dpkg-query --showformat='${Package}\t${Version}\n' --show ros-kinetic-magni-robot ros-kinetic-ubiquity-motor ros-kinetic-fiducials ros-kinetic-raspicam-node ros-kinetic-pi-sonar`

*  To run magni services after a 'systemctl disable' line as shown above:    `roslaunch magni_bringup base.launch`.

* The launch file used by the magni-base.service is `/opt/ros/kinetic/share/magni_bringup/launch/base.launch`.  
This launch file does `rosrun controller_manager spawner ubiquity_velocity_controller ubiquity_joint_publisher`.  
This invokes the ROS controller_manager; for more detail see `wiki.ros.org/controller_manager`.

* Much magni software will be found in ```/opt/ros/kinetic/share/magni_*``` In particular, magni_demos/launch has ROS launch files.

## Keeping your battery charged

By FAR the number 1 issue we see time and again is a weak or discharged battery. We have protections to shutdown things but as the battery gets weak many other issues show up.  We are continuously making strides to better inform customers of dangerously low batteries. In versions of the product shipping in 2022 we have the battery state indicated on the recently added OLED display to greatly help make this issue very visible to users of the robot.  The battery voltage and if it is too low will show up in 2022 current systems.

If you are 100% sure your battery is delivering over 23 volts to the MCB board large power connectors as the robot runs then you can skip this section and move to the next section.

Make sure your battery is installed correctly, with all the contacts
fully attached and the batteries are fully charged.
A pair of fully charged Lead Acid batteries should give around
26-27V - if you don’t have a voltmeter and the robot has enough charge to run then the robot can self report
battery voltage which is covered later on in this message (point 5).

A good way to make sure the batteries are fully charged is by
plugging in the provided charger. If it switches off automatically
then the the batteries are fully charged.

Make sure the 14 pin connector that connects the switch board to the main board has not pulled out and if it has push it in again.

Make sure that both push buttons on the front of the robot are out
all the way (the red push button de-energizes the motor circuit
as an emergency stop). Both blue and red LED on the small ```switch board``` PCB that has the switches should be illuminated.


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