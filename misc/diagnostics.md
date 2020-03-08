---
layout: default
title:  "Diagnostics"
permalink: diagnostics
---
#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

## Diagnostics

### Sanity Checks
* When magni is on, the wheels should have strong resistance to movement.  

* The command `sudo systemctl status magni-base`should show that the magni-base service is up and running

* The command `rosnode list` should show `motor_node` and other nodes. The command `rostopic list` should show many ROS topics, including `/battery` and `/cmd_vel`.

* After the command `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`, pressing the 'i' key should move the robot.

* To change the robot's hostname (a must if you will ever have a second robot), see the instructions in [Connecting the Robot](connect_network).
Instructions for joining your local area network are in the same place.


* To check the battery level type: `rostopic echo /battery`

  This should show you various information about the battery. If the battery voltage is somewhere between 21-28V that would be normal. Obviously the lower the voltage the less charge the batteries have.

### Handy Tips for Developers

* The important configuration file `base.yaml` is found at: `/opt/ros/kinetic/share/magni_bringup/param/base.yaml`. N.B. This file may be rewritten when the base is upgraded, as by apt=get.

* To disable magni startup: `sudo systemctl disable magni-base` and reboot so magni-base will be inactive.

    To get back to normal behavior you will need later a `sudo systemctl enable magni-base` and then a reboot.   

* To find the firmware version: when the robot is idle,  
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

* Much magni software will be found in `/opt/ros/kinetic/share/magni_*`` In particular, magni_demos/launch has ROS launch files.

### Motor Controller Board Pinouts

The pinouts for the many connectors on the main Motor Controller Board can be found [HERE](https://learn.ubiquityrobotics.com/Magni_MCB_pinout.pdf).

### Motor Controller Revisions

A list of firmware and master controller board revisions can be found [HERE](https://github.com/UbiquityRobotics/ubiquity_motor/blob/indigo-devel/Firmware_and_Hardware_Revisions.md).

### Guidelines for Usage Of The I2C Bus
#### The I2C devices Ubiquity Robotics reserves:
Addresses given in 7-bit form so on the I2C bus they appear shifted up by 1 bit.

* MCP7940 RTC 0x6F Realtime clock for Linux when no NTP can be contacted
* PCF8574 IO Expander 0x20 IO Expander for Ubiquity Robotics U3 on 5.x MCB.
* PCF8574A IO Expander 0x38. We have had this loaded by accident for U3    
* SSD1306 OLED Disp 0x3C 8 line by 15 character small display on rev 5.x boards

#### Tips and Guidelines For any I2C usage On the Magni Platform:
The I2C is the main 3.3V Raspberry Pi I2C on pins 3 and 5 with Raspberry Pi as the master.
The Rev 5.0 board has a 4-pin jack that brings out I2C and 3.3V with a ground.

* Use the I2C ONLY from within a ROS node to avoid conflict on the I2C bus.
* Keep the I2C lines to your device under 60mm from board to your device
* Only use devices for short data accesses and space out your accesses by at least 50msec
* If you stop Magni with `sudo systemctl stop magni-base.service` you can scan the I2C bus for all devices using `sudo i2cdetect -y 1` if i2c-tools was installed using apt-get. The i2c-tools package also has `i2cget` and `i2cset` so look them up if curious.
