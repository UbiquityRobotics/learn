---
layout: default
title:  "Driving a Magni with a keyboard"
permalink: keyboard_teleop
author: Alan N. Federman
---
# Driving a Robot with a Keyboard

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/) 

Connect to your robot and log in, following the instructions in [Connecting to your robot for the first time](connecting). Log in to the robot from an ssh window, as before.

After logging in, check to see if the ROS magni_base program is running:

    rostopic list

Next, start the keyboard teleop program.  In the ssh window, type:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

This will launch the teleop program in the robot. The following screen will appear:

<!--Alternatively from a second terminal window on your workstation, you can run the program in the workstation and let it communicate with the robot:
    export ROS_MASTER_URI=http://ubiquityrobotXXXX.local:11311
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
-->    

>Reading from the keyboard and Publishing to Twist!

>Moving around:

| | | |
|---|---|---|
|  u | i  | o  |
|  j | k  | l  |
|  m | ,  | .  |

The robot will respond to keystrokes as follows:

| | |
|---|---|
| u | circle left |
| I | straight ahead |
| o | circle right |
| j | rotate counter clockwise |
| k | all stop |
| l | rotate clockwise |
| m | circle backwards left |
| , | straight back |
| . | circle backwards left |

<!--
>For Holonomic mode (strafing), hold down the shift key:

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

Please note Holonomic mode does not apply to a Magni or Loki, as the robots are differential drive.
-->

You can also run teleop-twist keyboard with the program running in your workstation. To do this you must connect the robot to your local WiFi network, and set up ROS on the workstation.  These are discussed under [Doing More](ix_doing_more), below.

<<[back](connecting)- - - - - - - - - - [next](camera_sensors)>>
