---
layout: default
title:  "The Fiducial Follow App"
permalink: fiducial_follow_app
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/) - - -&uarr;[up](ix_programming)

## The Fiducial_Follow App

With this app, the robot will follow a given marker, keeping a certain distance from it as it moves and turns.

The Ubiquity Robotics robots use a number of fiducial markers (aruco markers, illustrated below).  The fiducial_follow app takes advantage of aruco_detect (a ROS node that is part of the Magni package) to detect aruco markers in the image feed from a camera.  The characteristics of the markers' images enable the robot to compute its location and that of the markers.

The app is written in Python. The code for the fiducial_follow app may be found [here](https://github.com/UbiquityRobotics/demos/blob/master/fiducial_follow/nodes/follow.py).  This document does not refer the code elements by line number because line numbers may change, but the reader can follow these references by use of the Search function.

<img src="https://ubiquityrobotics.github.io/learn/assets/fiducial.png" />

With navigation running aruco_detect continuously searches its field of view for aruco markers. When a marker is found a `fiducial_transforms` message is published with an array of transforms for all the markers that have been found.

The fiducial_follow module subscribes to these messages, using the rospy.Subscriber function.  

```rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.newTf)```


You can see that this function takes arguments indicating the message type that will be received `(FiducialTransformArray)` and the function to be called when a message is received `(self.newTf)`.

Now, this app is interested in only one fiducial marker: the one it is following (aruco marker number 49 by default).  So it is the job of the newTf routine to find the target marker. Nonetheless, it first publishes a transform for each fiducial, whether it is the target one or not.

    self.br.sendTransform(t)

This causes rviz (assuming it is being used) to show the location of each of the markers. The function self.br is declared at initialization to be a tf2_ros.TransformBroadcaster().

If the target marker is found in the array, its coordinates are saved in the variables self.x and self.y and the variable self.got_fid is set True. This notifies the run procedure (search for "def run") that it has something to do--it's been looping at 20Hz ever since the program was started (search for node.run). But until now it has issued no commands to the robot.

Now the run function calculates the forward and turning speed needed (both may be zero) to move towards the marker and issues the needed commands to the robot. It does this by publishing a standard ROS message (known for some reason as a Twist message) that specifies the velocity and direction to move.

    twist = Twist()
    twist.angular.z = angSpeed
    twist.linear.x = linSpeedself.cmdPub.publish(twist)

If both the twist parameters are zero, the robot will stop.

Publishing this message is done by the cmdPub function, which has been declared to be of type rospy.Publisher.

<img src="https://ubiquityrobotics.github.io/learn/assets/rosgraph.svg" />


#### Nodes
The single program follow.py

#### Parameters
target_fiducial: the fiducial we are following. The default is fid_49. Numerous other parameters may be found in the initialization section. Search for "rospy.get_param" to find them.

#### Publications (i.e., output)

cmd_vel(geometry_msgs/Twist): Commands to move the robot.

#### Subscriptions (i.e., input)

fiducial_transforms:(fiducial_msgs/FiducialTransformArray)

#### Other
uses the fiducials package https://github.com/UbiquityRobotics/fiducials
