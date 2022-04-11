---
title: "Driving with a keyboard"
permalink: noetic_quick_keyboard_driving
group: "quick start"
rosver: noetic
nav_order: 5
nav_exclude: false
--- 

# Driving a Robot with a Keyboard

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

Connect to your robot and log in, following the instructions in [Connecting to your robot for the first time](connecting). Log in to the robot from an ssh window, as before.

After logging in, the  magni_base program will be running under ROS. To verify this, type:

    rostopic list

Next, start the keyboard teleop program.  In the ssh window, type:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

This will launch the teleop program in the robot. There will be a screen that shows the commands in a sort of crude graphical way.

Because the robot has a 'dead man timer' you must rapidly continue to press keys or the robot will automatically stop as a safety measure.  Sometimes users hit the keys too slow and will see the robot do jerky movements but that is the deadman timer kicking in for too slow of a rate of keypresses.

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
| . | circle backwards right |

You can increase or decrease the speed that will be used also from keys on the keyboard but it is often desired to first stop the robot and then when you issue the increase or decrease the robot will not keep moving.  You may want to increase speed AS the robot is moving and that is ok as well.

| | |
|---|---|
| k | optionally stop the robot |
| q | increase speed |
| z | decrease speed |

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

Please note Holonomic mode does not apply to a Magni, as it uses differential drive.
-->

# Running teleop-twist from a workstation

You can also run teleop-twist keyboard from a properly setup ROS workstation that has the Magni robot setup as it's ROS master. To do this you can connect your configured ROS workstation to the network of the robot (which can be to connect to the robot Access point OR have the robot connect to your wireless network along with your workstation).  

Take a look a the first two links about connecting to a network and setting up a ROS workstation [Doing More](ix_doing_more).

## Must Have teleop-twist setup on your ROS workstation

Install teleop-twist on your workstation which will be setup to have the robot be the ROS master.   The program can be run as discussed above because the ```cmd_vel``` topic will be fed movement commands by the teleop-twist program on your workstation and the robot will then move just as it would if the teleop-twist were run on the robot itself.

     sudo apt install ros-kinetic-teleop-twist-keyboard
