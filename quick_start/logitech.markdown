---
layout: default
title:  "Using the Optional Logitech Controller"
permalink: logitech
---
# Using the Optional Logitech Controller

For a standard Magni equipped with a Raspberry Pi 3 
you need a laptop or some way to open a terminal session 
to the RPi controlling the robot. If you booted up with a 
Ubiquity Roboitcs RPi image, and have the Logitech controller 
dongle installed, the joystick should start to work in a couple of minutes.

### Trouble Shooting

If nothing happens, you'll need to log on to the robot.
(see connecting to your robot for the first 
time.)

Connect either by AP mode directly, or from your laptop.

On the laptop open up a terminal window with ctrl-alt-t

See if the robot is there:

	ping robotname.local



the robot responds by giving you its IP number.

If your Robot is equiped with a Raspberry Pi 3 Ubiquity image, the Joystick program should already be running.

you can test this by using the Joystick and seeing if the robot responds to commands. For safety sake, put the robot up on blocks first.(SAFETY TIP).



Then use the following command to login to the robot for example  from a laptop networked:

	ssh ubuntu@10
	
or directly:

	ssh ubuntu@10.42.0.1


You may get asked to authorize your key, say yes.

You next should be prompted for your password, type it in and you’ll be logged in

upon success:

	---------
	Welcome to Ubuntu 16.04.2 LTS (GNU/Linux 4.4.43-v7+ armv7l)


	* Documentation: https://help.ubuntu.com

	* Management: https://landscape.canonical.com

	* Support: https://ubuntu.com/advantage


	1 package can be updated.

	0 updates are security updates.


	Last login: Sat May 6 15:57:19 2017 from 10.53.5.19

To see if the joystick is running, type:

rostopic list

If you see /joy  you are good to go!

If you don't see any topics you need to launch the logitech on the robot:


	roslaunch magni_bringup base.launch >/dev/null 2>&1 & 

(the /dev/null 2 >&2 & puts into background and stops output to screen)


	roslaunch magni_teleop logitech.launch >/dev/null 2>&1 &




The folowing command

	rostopic list

	/cmd_vel
	/diagnostics
	/joint_states
	/joy
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

Shows everything is running. You now can use the controller to drive the robot.

If we were using keyboard teleop our keys would be arranged like this:


	u i o
	j k l
	m , .


	u circle left
	i straight ahead
	o circle right
	j rotate counter clockwise	
	k all stop
	l rotate clockwise
	m circle backwards left
	, straight back
	. circle backwards left

## Logitech Controller

![Logitech Controller](https://ubiquityrobotics.github.io/learn.magni.com/assets/joystick.jpg)

<!--

Other possible way to display an image

![1038 image](https://ubiquityrobotics.github.io/learn.magni.com/assets/joystick.jpg)-->

<!--div class="image-wrapper">

		1038 raw
    
        <img src="https://ubiquityrobotics.github.io/learn.magni.com/assets/imag1038.jpg?raw=true" />
		
		1038
     
        <p class="image-caption">A Basic Multimeter</p>
    
</div-->
The Left joystick controls the direction and pseed of motion, the left button is the 'dead man' switch, and needs to be held down simulatanously with the joystick.  

