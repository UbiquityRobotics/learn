---
layout: default
title:  "Try out sensors"
permalink: sensors
---
#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/)

<!--
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

-->
### To install sonars,

`sudo apt update`  
`sudo apt upgrade`  
`sudo apt install ros-kinetic-pi-sonar`

Then, to enable the sensors, edit the file as root using

    `sudo nano /etc/ubiquity/robot.yaml`  

  or some other editor of your choice, so that

`sonars: 'pi_sonar_v1'`
is uncommented and `sonars: None` is commented. Be sure there is a space just after the `sonars:` and before the single quote.

Like so:
```
# Robot Configuration
#sonars: None
sonars: 'pi_sonar_v1'
```
### Operation

[The sonar node](https://github.com/UbiquityRobotics/ubiquity_sonar) publishes a `sensor_msgs/Range message` for each sonar reading.
Rviz can visualize these messages as cones.  There are launch files to do this in:  
https://github.com/UbiquityRobotics/magni_robot (the source package, not the binary packages)

The [move_basic node](http://wiki.ros.org/move_basic) uses the messages published by the sonar node to determine proximity to obstacles.

| | |
|---|---|
|Topic|                Direction|
|/pi_sonar/sonar_0|   Far right|
|/pi_sonar/sonar_1|   45 degrees to the left|
|/pi_sonar/sonar_2|   45 degrees to the right
|/pi_sonar/sonar_3|   Front|
|/pi_sonar/sonar_4|   Far left|  

<!-- todo complete this page! -->
