---
title: "Driving with a keyboard"
permalink: noetic_quick_keyboard_driving
group: "quick start"
rosver: noetic
nav_order: 5
nav_exclude: false
--- 

# Driving a Robot with a Keyboard

Connect to your robot and log in, following the instructions in [Connecting the Robot to Your Network](noetic_quick_start_connecting). Log in to the robot from an ssh window, as before.

After logging in, the  magni_base program will be running under ROS. To verify this, type:

    rostopic list

Next, start the keyboard teleop program.  In the ssh window, type:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

This will launch the teleop program in the robot. There will be a screen that shows the commands in a sort of crude graphical way.

Because the robot has a 'dead man timer' you must rapidly continue to press keys or the robot will automatically stop as a safety measure.  Sometimes due to spotty wifi connectivity the received commands may be too slow and you may see the robot do jerky movements but that is the deadman timer kicking in for too slow of a rate of keypresses.


Moving around:

|  u | i  | o  |
|  j | k  | l  |
|  m | ,  | .  |

The robot will respond to keystrokes as follows:


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


| k | optionally stop the robot |
| q | increase speed |
| z | decrease speed |

<br>

## Running teleop-twist from a workstation

As of right now you're running the teleop node on the actual robot, and as such are sending keypress packets through the SSH tunnel to the robot. To run the node on your workstation and send messsages through ROS sockets, then you'll need to first [connect to the robot as a workstation and set it up as a ROS master](noetic_quick_start_workstation).

You'll also need to install the actual teleop node in order to run it:

     sudo apt install ros-kinetic-teleop-twist-keyboard

There is no major difference in running the node locally, however the principle is a good starting point and test for more advanced usage.
