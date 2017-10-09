---
layout: default
title:  "Using the Optional Logitech Controller"
permalink: logitech
author: Alan Federman
---
# Using the Optional Logitech Controller

A standard Silver or Gold Magni equipped with a Raspberry Pi 3 
with a Ubiquity Robotics SD card image is by default set up to work with a Logitech Controller.
If you booted up with a Ubiquity Roboitcs RPi image, and have the Logitech controller 
dongle installed, the joystick should start to work in a couple of minutes.

You can test this by using the Joystick and seeing if the robot responds to commands. For safety sake, put the robot up on blocks first.(SAFETY TIP).

If the wheels don't move, check the troubleshooting section below


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

![Logitech Controller](https://ubiquityrobotics.github.io/learn/assets/joystick.jpg)

<!--

Other possible way to display an image

![1038 image](https://ubiquityrobotics.github.io/learn.magni.com/assets/joystick.jpg)-->

<!--div class="image-wrapper">

		1038 raw
    
        <img src="https://ubiquityrobotics.github.io/learn.magni.com/assets/imag1038.jpg?raw=true" />
		
		1038
     
        <p class="image-caption">A Basic Multimeter</p>
    
</div-->

To operate the controller, hold down the deadman button and move the joystick. 

Up - Forward, Down - Back, Left - rotate counter clockwise, Right - rotate clockwise

The controller is set to be very slow. You can increase the speed by using the button below the deadman, this will increase the maximum speed by about 25%.  Moving the joytick to a position between full up and full right will move the robot in a curved path.



### Troubleshooting

If nothing happens, you'll need to go back to the unboxing section and check to see
if the robot is gettting power, the MCB is working and you can connect to the robot via a network
(see connecting to your robot for the first time.) Check to see that your dongle is installed in the Raspberry Pi, there are fresh batteries in the Logitech Controller, and there is power going to both the Raspberry Pi and the Motors.

Next, see if you can connect to your robot.

	ping robotname.local

the robot responds by giving you its IP number.


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

Shows everything is running. You now should be able use the controller to drive the robot.

