---
layout: default
title:  "Driving a Magni with a keyboard"
permalink: keyboard_teleop
---
# Driving a Robot with a Keyboard

For a standard Magni equiped with a Raspberry Pi 3 you need a laptop or some way to open a terminal session to the Rpi controlling the robot. If the proper software is installed, You can drive the robot from a remote terminal. Power up the robot (Rpi power light is on and red). Assuming the robot is networked locally, or if you are using the default IP address (see connecting to your robot for the first time.)

On the laptop open up a terminal window with ctrl-alt-t

See if the robot is there: ping robotname.local

the robot responds by giving you its IP number.

Then use the following command to login to the robot for example:

ssh ubuntu@10.0.0.117

You may get asked to authorize your key, say yes.

You next should be prompted for your password, type it in and you’ll be logged in

upon success:

Welcome to Ubuntu 16.04.2 LTS (GNU/Linux 4.4.43-v7+ armv7l)

* Documentation: https://help.ubuntu.com
* Management: https://landscape.canonical.com
* Support: https://ubuntu.com/advantage

1 package can be updated.
0 updates are security updates.

Last login: Sat May 6 15:57:19 2017 from 10.53.5.19

If you have booted from our RPi image, the robot is already for accepting 
commands. To check if this is true:

rostopic list

should show a list of topics. If you get an error message, you need to
launch the robot base:

roslaunch magni_bringup base.launch >/dev/null 2>&1 & 
	(the /dev/null 2 >&2 & puts into background and stops output to screen)

You can launch keyboard teleop directly on the robot:


rosrun teleop_twist_keyboard teleop_twist_keyboard.py


or remotlely from a second terminal window on your laptop:

export ROS_MASTER_URI=http://(robots IP number):11311

rosrun teleop_twist_keyboard teleop_twist_keyboard.py


to see if the topics are correct for a magni, rostopic list should show:

'
/cmd_vel
/joint_states
/left_error
/motor_node/parameter_descriptions
/motor_node/parameter_updates
/right_error
/rosout
/rosout_agg
/tf
/tf_static
/ubiquity_velocity_controller/cmd_vel
/ubiquity_velocity_controller/odom
'

once teleop is launched the following screen appears:


Reading from the keyboard and Publishing to Twist!

Moving around:
  ______
  u i o
  
  j k l
  
  m , .
  ______

For Holonomic mode (strafing), hold down the shift key:

  ______
  U I O
  
  J K L
  
  M < >
  ______

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

currently:	speed 0.5	turn 1

Please note Holonomic mode does not apply to a Magni or Loki, as the robots are differential drive.

u circle left
I straight ahead
o circle right
j rotate counter clockwise
k all stop
l rotate clockwise
m circle backwards left
, straight back
. circle backwards left



To stop the running nodes you need to find out their process IDs (PIDs)

ps -eaf|grep roslaunch

ubuntu 5101 4504 1 08:48 pts/1 00:00:04 /usr/bin/python /opt/ros/kinetic/bin/roslaunch magni_bringup base.launch


so pkill roslaunch would stop ROS.


