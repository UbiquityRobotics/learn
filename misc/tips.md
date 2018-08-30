---
layout: default
title:  "Tips and Tricks"
permalink: tips
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)
<!--
I think it would be good for Joe to try these things and for example say you should see about X lines for the 'rosnode list' and Y lines for  'rostopic list' for example.

Many of these are geared towards technical people who are working with the bot to develop things and need to stop and start nodes and so on but they are 'gold' for those people.

Suggest now that the 'pifi' line and change hostname items be way up as items 4 and 5 for example.  Then the 'technical ones' could be left off for now or put in as 'And for all the technical developers these are handy'.  -->
## Tips and Tricks

### Sanity Checks
* When magni is on, the wheels should have strong resistance to movement.  

* The command `sudo systemctl status magni-base`should show that the magni-base service is up and running

* The command `rosnode list` should show `motor_node` and other nodes. The command `rostopic list` should show many ROS topics, including `/battery` and `/cmd_vel`.

* After the command `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`, pressing the 'i' key should move the robot.

* To change the robot's hostname (a must if you will ever have a second robot), see the instructions in [Connecting the Robot](connect_network).
Instructions for joining your local area network are in the same place.


* To check the battery level type: `rostopic echo /battery`

  This should show you various information about the battery. If the battery voltage is somewhere between 21-28V that would be normal. Obviously the lower the voltage the less charge the batteries have.

### Handy Tips for Developers Only

* The important configuration file `base.yaml` is found at: `/opt/ros/kinetic/share/magni_bringup/param/base.yaml`.

* To disable magni startup: `sudo systemctl disable magni-base` and reboot so magni-base will be inactive.

    To get back to normal behavior you will need later a `sudo systemctl enable magni-base` and then a reboot.   

* To find the firmware version: when the robot is idle,  
    `rosrun ubiquity_motor probe_robot -f`  
    For even more fun try **-h** instead of **-f**.

    If the robot is running a program, run  
    `sudo systemctl disable magni-base.service`  
    and then reboot before trying the probe_robot command again.

    To return the robot to normal startup you will then need to use   `sudo systemctl enable magni-base.service` and reboot again.  

    To find the version number of the most important ubiquity software type:

    `dpkg-query --showformat='${Package}\t${Version}\n' --show ros-kinetic-magni-robot ros-kinetic-ubiquity-motor ros-kinetic-fiducials ros-kinetic-raspicam-node ros-kinetic-pi-sonar`

*  To run magni services after a 'systemctl disable' line as shown above:    `roslaunch magni_bringup base.launch`.

* The launch file used by the magni-base.service is `/opt/ros/kinetic/share/magni_bringup/launch/base.launch`.  
This launch file does `rosrun controller_manager spawner ubiquity_velocity_controller ubiquity_joint_publisher`.  
This invokes the ROS controller_manager; for more detail see `wiki.ros.org/controller_manager`.

* Much magni software will be found in `/opt/ros/kinetic/share/magni_*`` In particular, magni_demos/launch has ROS launch files.
