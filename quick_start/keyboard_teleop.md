---
layout: default
title:  "Driving a Magni with a keyboard"
permalink: keyboard_teleop
---
# Driving a Robot with a Keyboard

From your workstation, connect to your robot and log in. (see connecting to your robot for the first time).

To drive the robot from the workstation, run the keyboard teleop program:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Alternatively from a second terminal window on your workstation:

    export ROS_MASTER_URI=http://(robots IP number):11311
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

When teleop is launched the following screen appears:

>Reading from the keyboard and Publishing to Twist!

>Moving around:

>| | | |
|----|----|----|
|  u | i  | o  |
|  j | k  | l  |
|  m | ,  | .  |
For Holonomic mode (strafing), hold down the shift key:

>| | | |
|----|----|----|
|  U | I  | O  |
|  J | K  | L  |
|  M | <  | >  |

>t : up (+z)  
b : down (-z)  
anything else : stop

>q/z : increase/decrease max speeds by 10%  
w/x : increase/decrease only linear speed by 10%  
e/c : increase/decrease only angular speed by 10%  

>CTRL-C to quit

currently:	speed 0.5	turn 1

Please note Holonomic mode does not apply to a Magni or Loki, as the robots are differential drive. The robot will respond to keystrokes as follows:

| | |
|--|--|
| u | circle left |
| I | straight ahead |
| o | circle right |
| j | rotate counter clockwise |
| k | all stop |
| l | rotate clockwise |
| m | circle backwards left |
| , | straight back |
| . | circle backwards left |

## DO WE WANT TO INCLUDE THE FOLLOWING?

To stop the running programs you need to find out their process IDs (PIDs)

ps -eaf|grep roslaunch

ubuntu 5101 4504 1 08:48 pts/1 00:00:04 /usr/bin/python /opt/ros/kinetic/bin/roslaunch magni_bringup base.launch

so pkill roslaunch would stop ROS.
