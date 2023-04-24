---
title: "Basic Usage"
permalink: noetic_conveyorbot_usage
group: conveyorbot
rosver: noetic
nav_order: 2
nav_exclude: false
---

# Basic Usage

Before using Conveyorbot, it is necessary to [set up routes](noetic_conveyorbot_setup).

Initially when robot boots up, the touchscreen display should show a **Loading** screen. After the system is ready you should be able to see a screen with a **START** button.

<img src="../../assets/breadcrumb/control_panel_screen.png" >

From the initial pose, it will wait for the user to press START and move to towards the detected marker.
It is important that the robot sees the first marker, otherwise it will not move and a warning will appear on the screen.

<video style="display: block; margin-left: auto; margin-right: auto;" width="75%" controls autoplay>
  <source src="assets/breadcrumb/Ubiquity_Turn_Cutted.mov" type="video/mp4">
  Your browser does not support the video tag.
</video>

<br>

ConveyorBot will smoothly navigate between markers, where each marker arrow should point in the direction of the next marker.
If the robot encounters a STOP marker (<img src="assets/breadcrumb/stop_marker.jpg" alt="" width="35">), it will stop on it and turn in the direction of arrow.
Once the robot is on the STOP marker, it will wait until the **CONTINUE** button is pressed on the touchscreen, and then continue driving in the direction of the arrow.

<video style="display: block; margin-left: auto; margin-right: auto;" width="75%" controls autoplay>
  <source src="assets/breadcrumb/Ubiquity_Start_Stop.mov" type="video/mp4">
  Your browser does not support the video tag.
</video>

<br>

Robots can also be driven with an optional [Logitech controller](noetic_quick_keyboard_driving#using-the-optional-logitech-controller) or through the touchscreen UI.
