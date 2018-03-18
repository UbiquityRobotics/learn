
# Software Reference

This page provides a reference to the software used on Ubiquity Robotics
robots.

## Nodes

The various nodes used are listed below, in the form _package_ _node name_,
along with a brief description and a link to more detailed information.

### aruco\_detect aruco\_detect

[Documentation](http://wiki.ros.org/aruco_detect)

`aruco_detect` detects fiducial markers in an image stream and publishes
their pose in terms of the camera's frame of reference.

### bt\_joystick bt\_joystick.py

[Documentation](https://github.com/UbiquityRobotics/bt_joystick/blob/master/README.md)

The `bt_joystick` node receives input from a Magicsee R1 joystick via
Bluetooth Low Energy and publishes motor commands.

### dnn\_detect dnn\_detect

[Documentation](http://wiki.ros.org/dnn_detect)

`dnn_detect` is a node that detects objects in an image stream using a
Deep Neural Network. It publishes information about them, including their
bounding box and object class name.

### dnn\_rotate rotate.py

[Documentation](https://github.com/UbiquityRobotics/demos/blob/master/dnn_rotate/README.md)

`rotate.py` is demo node showing how to use actionlib and service calls
to interact with other nodes.  This node invokes `dnn_detect` to find
objects in images from a camera and `move_basic` to move towards them.

### loki\_base\_node bus\_server.py

[Documentation](https://github.com/UbiquityRobotics/loki_base_node/blob/indigo/README.md)

`bus_server.py` is the interface between ROS and the micro controller on
Loki robots. It subscribes to motor commands and publishes odometry and 
sonar range messages.

### fiducial\_follow follow.py

[Documentation](https://github.com/UbiquityRobotics/demos/blob/master/fiducial_follow/README.md)

`follow.py` is node that receives the position of fiducial markers in
an image stream from `aruco_detect` and issues motor commands for the
robot to follow a particular marker.

### fiducial\_slam fiducial\_slam

[Documentation](http://wiki.ros.org/fiducial_slam)

### move\_base move\_base

[Documentation](http://wiki.ros.org/move_base)

The `move_base` node provides a ROS interface for configuring, running,
and interacting with the navigation stack on a robot. 

### move\_basic move\_basic

[Documentation](http://wiki.ros.org/move_basic)

`move_basic` is a node that provides a simple arc-tangent planning navigation
node. The plan for moving towards a goal consists of rotating to face it,
driving straight towards it, and rotating again to achieve the final pose.

### move\_demo move.py 

[Documentation](https://github.com/UbiquityRobotics/demos/blob/master/move_demo/README.md)

The `move_demo` package contains a node `move.py` that takes command line
arguments instructing it to issue movement commands for the robot to rotate
or move forwards or backwards.

### Robot\_Commander

[Documentation](https://github.com/UbiquityRobotics/Robot_Commander/blob/master/README.md)
 
`Robot_Commander` is an Android app that allows control of a robot, including
tele-operation and setting and navigating towards waypoints.  It interacts
with ROS via `rosbridge_websocket`.  The interaction includes sending of
motor commands, and requesting `move_basic` to navigate to a goal.

### raspicam\_node raspicam\_node

[Documentation](https://github.com/UbiquityRobotics/raspicam_node/blob/indigo/README.md)

`raspicam_node` is a node which captures images from the Raspberry Pi camera
and publishes them as a compressed image stream.

### rosbridge\_websocket rosbridge\_websocket

[Documentation](http://wiki.ros.org/rosbridge_suite)

`rosbridge_websocket` is part of the rosbridge suite, which provides
an Application Programming Interface to ROS from JavaScript.

### rviz rviz

[Documentation](http://wiki.ros.org/rviz)

`rviz` is a 3D visualization tool for ROS.  It is used to show the sensor
data and state of a robot.

### rqt\_image\_view rqt\_image\_view

[Documentation](http://wiki.ros.org/rqt_image_view)

`rqt_image_view` is a plugin for `rqt` that displays images.

### rqt\_reconfigure rqt\_reconfigure

`rqt_reconfigure` is a plugin for `rqt` that allows paramaters in nodes
to be configured dynamically.

[Documentation](http://wiki.ros.org/rqt_reconfigure)

### ubiquity\_motor ubiquity\_motor

[Documentation](https://github.com/UbiquityRobotics/ubiquity_motor/blob/indigo/README.md)

`ubiquity_motor` is the interface between ROS and the motor controller
board on Magni robots.  It subscribes to motor speed commands, and publishes
odometry and battery state.

### ubiquity\_sonar

[Documentation](https://github.com/UbiquityRobotics/ubiquity_sonar/blob/indigo/README.md)

The `ubiquity_sonar` node publishes sonar range messages from sonar sensors
connected to a Raspberry Pi's General Purpose Input Output (GPIO) pins.
