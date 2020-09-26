---
layout: default
title:  "Diagnostics"
permalink: diagnostics
---
#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

# Diagnostics And Technical Details

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

## Motor And Wheel Encoders

We limit the motor speed by default to 1 M/sec.  It is possible to make this faster but perhaps contact us for guidance.

The standard motor/wheel unit we ship have these characteristics

    - 3 phase brushless motors with internal gearing.
    - The Wheels are spaced 0.33 meters apart.
    - The wheels have a circumference of 0.64 meters
    - The 3 phase magnetic encoders produce 43 pulses each phase per revolution
    - We use both edges of the 3 non-overlaping phases for 258 tics per rev
    - This translates to about 2.5mm as the rough linear travel per enc tick
    - This also translates to 1.4 degrees wheel rotation per encoder tick

## Motor Controller Board Pinouts

The pinouts for the many connectors on the main Motor Controller Board can be found [HERE](https://learn.ubiquityrobotics.com/Magni_MCB_pinout.pdf).

## Motor Controller Revisions

You can see and download the released MCB board firmware revisions [HERE](https://learn.ubiquityrobotics.com/firmware-upgrade)

A list of firmware and master controller board revisions can be found [HERE](https://github.com/UbiquityRobotics/ubiquity_motor/blob/kinetic-devel/Firmware_and_Hardware_Revisions.md).

## The MCB Serial Protocol and Commands
A baud rate of 38400 is used with one stop bit for communications with between the host cpu (normally raspberry Pi) and the MCB, Motor Control Board.

The Motor Control Board, sometimes called main control board, uses a protocol where a packet with checksum is sent and if a reply is required the reply will come back in the same binary protocol.

By default the raspberry pi default host cpu single serial port on the 40 pin connector.  The MCB constantly transmits status and this goes to the 40 pin connector pin 10 which is the host Receive.  The raspberry pi host sends commands on it's Transmit using 3.3V signals and this arrives at the MCB on pin 8 of the 40 pin connector.

Although the MCB sends status immediately on power up the host takes sometimes a minute or so to start all the nodes and then start sending commands to the MCB.  On revision 5.2 and later boards there are two blue leds that show both the directions of serial traffic.  Magni will not really be running till both of these are seen to be blinking very fast.

For MCB boards prior to revision 5.2 the MCB serial conversion circuits required a 3.3V power supply to appear on pin 1 of the 40 pin connector.

#### MCB Serial Protocol Details
For details see  [The Magni Serial Protocol Spec](https://github.com/UbiquityRobotics/ubiquity_motor/blob/indigo-devel/Serial_Protocol.md)

#### Standalone Test Program To Control The Magni_MCB
A standalone test program used in our tests and development.  You can get the source to this program on github  [HERE](https://github.com/UbiquityRobotics/ubiquity_motor/blob/kinetic-devel/scripts/test_motor_board.py)

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
