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

### Handy Tips for Developers

* The important configuration file `base.yaml` is found at: `/opt/ros/kinetic/share/magni_bringup/param/base.yaml`.

* Disable/Enable magni startup:    sudo systemctl disable magni-base    OR stop it:  sudo systemctl stop magni-base.service

* To find the firmware version:  
    `sudo systemctl stop magni-base.service` **(why?)**  
    `rosrun ubiquity_motor probe_robot -f`  
    For other useful information use -h instead of -f.   

*  Manually run magni services after a 'systemctl disable' line as earlier shown:    roslaunch magni_bringup base.launch

* Tech Detail: launch file launched by the magni-base.service and launches /opt/ros/kinetic/share/magni_bringup/launch/base.launch
        Launch does 'rosrun controller_manager spawner ubiquity_velocity_controller ubiquity_joint_publisher  

        see wiki.ros.org/controller_manager
         magni_demos teleop.launch in /opt/ros/kinetic/share/magni_teleop/launch

* TechDetail: /opt/ros/kinetic/share/magni_* are all sorts of magni software and magni_demos/launch has ROS launch files
