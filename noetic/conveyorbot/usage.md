---
title: "Basic Usage"
permalink: noetic_conveyorbot_usage
group: conveyorbot
rosver: noetic
nav_order: 2
nav_exclude: false
---

# Basic Usage

Before using Conveyorbot, it is necessary to [set up routes](noetic_conveyorbot_setup). If routes are already set up and you just want to run the robot on a particular route, this page should provide enough information to do so. It is also recommended that you first read this page before setting up routes, so that you understand the basic operation of the robot.

## Turning Conveyorbot on and off

On the right side of the shell of the robot, a bit above its right wheel, there is a black power switch and a red emergency stop push button. To start up the robot, you should switch the power switch to the "on" state and wait for the boot process to finish.
To shut down the robot, you should press the Power off button in the navigation bar of the robot's touchscreen and wait for the shut down process to finish. After that, you can switch the power switch to the "off" state.
If you need to stop the robot in an emergency situation, you can push the emergency stop button. This will cause the robot to stop moving (but won't turn off the robot). If you want it to start moving again, rotate the emergency stop button a bit to release it. 
Note that it might also happen that the robot won't move, because the emergency stop button is pressed without you being aware of it. In this case, release it.

## Starting the robot

Initially when the robot boots up, the touchscreen display shows a **Loading screen**. After the system is ready, you should be able to see a screen with a **START** button and a drop-down list from which a desired route can be selected.

<img src="../../assets/breadcrumb/control_panel_screen.png" >

You should first select a route and then the Conveyorbot will wait for the user to press the START button. This will cause it to start driving on that route.
It is important that the robot initially sees (detects) one marker which is a part of the selected route, otherwise it will not move and a warning will be shown on the screen. To make sure that the robot sees a particular marker, you can open the **Joystick control screen** with the monitor-shaped button in the navigation bar. There you will see an image of what the robot currently sees, and currently detected markers will be outlined with a green outline. If the marker from which you wish to start the robot is not in the camera's field of view, you can move the robot with the joystick on the right side of that screen so that the marker will be in the field of view. It is also required that if you want to start the robot from a green "GO" marker, the robot should be directed in the same direction as the GO marker, since oppositely directed GO markers are ignored by default. This means that the robot has to be on the "other side" of the GO marker (on the side without the arrow) and needs to be facing towards the marker (and have it in the camera's FOV). If you are starting the robot from a purple "BIDIRECTIONAL" marker, note that the robot will start driving in one of the two directions to which the arrows of the marker are pointing - in the direction which requires less robot rotation. [Here](noetic_conveyorbot_fiducials) is more information about each type of markers.

## Navigation and stopping

<video style="display: block; margin-left: auto; margin-right: auto;" width="75%" controls autoplay>
  <source src="assets/breadcrumb/Ubiquity_Turn_Cutted.mov" type="video/mp4">
  Your browser does not support the video tag.
</video>

<br>

Conveyorbot will smoothly navigate between markers, where each marker arrow should point in the direction of the next marker. It might also happen that a marker points to a cluster of blue TURN markers (a crossroad), each of which points in a different direction. In this case, the robot will decide which of those markers to follow based on which marker is next in the route you selected.

Since Conveyorbot has advanced Lidar-based collision avoidance software, it will stop if an obstacle or another robot is in its planned path and could cause collision. After the obstacle moves away, the robot will start driving again automatically. This also enables multiple Conveyorbots to drive on the same "marker setup" simultaneously. 

A "marker setup" usually means all of the markers which form the "main circular route" and all of the "branches" connected to it. This is the recommended structure of markers in order for multiple robots to be able to run on the same marker setup. Detailed information about this structure is in Route Setup section, but in short, the "main circular route" (MCR) is a common "looping" route, which is a part of all of the possible routes on a particular marker setup. Out of the MCR, a "branch" can be created, so that some of the robots can go to another part of the facility in which they operate (by following this branch) and then return back onto the MCR. Having multiple such branches enables some robots to go to one part of the facility and some other robots to go to another part. 
A branch can either have distinct "outbound" and "inbound" "marker paths" (each of them made of basic one-directional markers), or both of the outbound and inbound paths can be on the same, so-called "bidirectional" marker path, made of BIDIRECTIONAL markers.
A "route" specifies the exact path that the robot will take through these branches (which of them will the robot drive on).
Note that **it is not possible for two Conveyorbots to drive simultaneously on a route which contains the same branch, if that branch contains BIDIRECTIONAL markers**. This is because the robots might get stuck on such a branch, if they meet head to head when one of them is driving in the inbound and the other in the outbound direction. Thus, only one robot can drive on each bidirectional branch at once.

If the robot encounters a STOP marker (<img src="assets/breadcrumb/stop_marker.jpg" alt="" width="35">), it will stop on it and turn in the direction of the arrow.
Once the robot is on a STOP marker, it will wait until the **CONTINUE** button is pressed on the touchscreen and then continue driving in the direction of the arrow. It is also possible to specify a custom timeout for each individual STOP marker, after which the robot will automatically continue driving (to specify that timeout, refer to the [route setup page](noetic_conveyorbot_setup)).
Note that you can also stop (pause) the robot at any time by clicking on the red STOP button on the touchscreen and then resume it by clicking on the green START button again.

<video style="display: block; margin-left: auto; margin-right: auto;" width="75%" controls autoplay>
  <source src="assets/breadcrumb/Ubiquity_Start_Stop.mov" type="video/mp4">
  Your browser does not support the video tag.
</video>

<br>

## Battery charging

In the route setup, a **charging marker** can be specified. This is a TURN marker on the main circular route (MCR) which is followed only when the robot's battery is low and the robot should charge. This marker redirects the robot on a branch which contains a STOP marker on which the robot always stops. In this way, the robot with low battery doesn't stop on the marker setup where also other robots drive, so that it doesn't obstruct them. A charger should be plugged into the robot(s) waiting on the charging branch. When the robot is full, CONTINUE button should be pressed and the robot will re-enter the MCR and continue driving.

## Alternative manual navigation

Robot can also be driven with an optional [Logitech controller](noetic_quick_keyboard_driving#using-the-optional-logitech-controller) or through the touchscreen UI (with the software joystick on the **Joystick control screen** mentioned before).