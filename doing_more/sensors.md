---
layout: default
title:  "Try out sensors"
permalink: sensors
---
#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/)

### To build:

cd ~/catkin_ws/src
git clone https://github.com/UbiquityRobotics/pi_sonar.git
cd ..
catkin_make
source devel/setup.bash

The node needs to run as the user root to access GPIO, hence the following unconventional steps:

sudo chown root ~/catkin_ws/devel/lib/pi_sonar/pi_sonar
sudo chmod 4755 ~/catkin_ws/devel/lib/pi_sonar/pi_sonar

### To run:

roslaunch pi_sonar pi_sonar.launch

### Operation

The sonar node (https://github.com/UbiquityRobotics/ubiquity_sonar) publishes a sensor_msgs/Range message for each sonar reading.

rviz can visualize these messages as cones.  There are launch files to do this in:
https://github.com/UbiquityRobotics/magni_robot (the source package, not the binary packages)

move_basic http://wiki.ros.org/move_basic uses these messages to determine proximity to obstacles.   publishes a 2D visualization of the sonar.
<!-- todo complete this page! -->
#### &larr;[back](waypoints)- - - - - - - - - - [up](ix_doing_more)&rarr;
