---
layout: default
title:  "Adding Hardware the MC Board"
permalink: mc_board
---

# Adding Hardware - the Motor Controller Board

We developed Magni for people who wanted to build their own robot for a variety of purposes. 
As such, we wanted it to be easy to add powered accessories or sensors to our robot. The heart of Magni is 
the Motor Controller board, which we refer to as the "MC"." While the MC's main purpose is to control
the two drive wheels, it also provides dual 5v and 12v power supply circuits, each is rated at 7amps.
There are a variety of ways to connect to these power sources, including PC style Molex and USB jacks, so if your ever 
have a black out, you can charge your mobile device from your robot(Just kidding.)
 
Additionally, the MC contains a real time clock (RTC). We included this because our default CPU, the Raspberry Pi 3, 
does not have one, and time sync is important for ROS when working with multiple robots and computers.
The MC has a lot of safety features including over and under voltage and current protection, automatic redundancy on the 5v
power suppy, reversed polarity. While the default Raspberry Pi 3 connects directly to the MC, an independent connection for 
a USB to TTL dongle allows you to connect any Ubuntu computer.

Recent additions include a way to attach a 'cliff' sensor, and a built-in self-test (BIST) routine to quickly 
diagnose any issues.

Magni uses brushless hub motors. each motor ahs three sets of magnetic coils that control the motor. In addition there are three maagetic sensors called "Hall Effect Sensors" that sense the position of the motor and hence the wheel. The MC can read the position of the motor and control its speed. It does this by changing the amount and direction of current on the power leads. The control is done via a PSOC (Programable Software on a Chip) various power transistors (MOSFETs) and other components.

Specialy designed algorithyms allow Ubiquity Robotics to anaylze the HAll Effect Sensor data to get exceptionally accurate odometry data from our Magni robot. The MC and the custom designed wheels are designed specifically to work together, and can not be replaced by a similar item.
