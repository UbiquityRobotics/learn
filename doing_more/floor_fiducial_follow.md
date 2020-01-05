---
layout: default
title:  "Using Floor Fiducials To Guide Your Robot"
permalink: floor_fiducial_follow
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

## Using Floor Fiducial Fiducials To Guide The Robot
We are adding a powerful ability to our fiducial follow application that will allow users to
write a python or other program to send fiducial follow as well as basic movement
commands to the follow.py application through a ROS topic. We hope to officially
release this code in the first quarter of 2020.

This page describe usage of a launch file that once released places the robot into a
mode where it can be issued commands to find and approach or move about in simple
movements. Contact us on the forum if you require this ability prior to introduction and
we can discuss Beta testing this ability. We call this launch file
fiducial_floor_follow.launch and it assumes fiducials of size 110mm because this size
seems a good choice for floor based fiducials given camera field of view and so on.

### A Few Words To Help Visualize How This Mode Enables User Applications
The enhanced follow.py app is written in Python. The existing code for the fiducial_follow
app may be found on [**THIS_PAGE**](https://github.com/UbiquityRobotics/demos/blob/master/fiducial_follow/nodes/follow.py) .

Several commands can be used in
applications where floor-attached fiducials are an acceptable solution. The commands allow functions such as the following:
• Change current drive speed or rotational speed to new values
• Look for and then approach a fiducial such as #105 then stop
• Once having found a fiducial such as #150, drive on top of it
• Drive forward or reverse for a given time at the current drive speed
• Rotate right or left for a given time at the current rotation speed

With these commands some sequential factory robotic applications can be implemented
without full implementation of the much more complex full room navigation methods
utilizing our MoveBasic command modes. In this way a robot can follow a trail of
fiducials on the floor based on a user application sending commands as markers are
reached.

This provides a simpler mode that may in many cases be all that is required.

The ROS topic of /follower_commands is issued commands with a name and
parameters depending on the action. ROS topic /follower_status will offer feedback to
the user program. So a program can say to drive to a fiducial then wait for positive
status before issuing another command. In this way a user application can have it's own
logic and let the Magni be the 'server' to perform the movement operations.
A user program once at a given location could for example then tell it's own custom
hardware to do some sort of load or unload operation using the custom user hardware
that may have been designed.

We offer a simple python script that can talk to and listen to this new follow.py node and
we call that script follower_controller.py which is very basic but perhaps a good head
start to develop your own specific control required to suit your end application.

### Setting Up a Downward Facing Camera to Use This App
By default we will specify that a downward facing RaspiCam will be used and will be
mounted 60cm above the floor and tilted 35 degrees downward ahead of the robot (this
number may change). We also are thinking floor fiducials should be 110mm across for
many reasons but if required users can modify this as suit their needs.

We plan on making camera mounting hardware to properly release this mode. A user
will have to edit /etc/ubiquity/robot.yaml as root and change position to 'downward'
raspicam:
position: 'downward'

### The Git Code Branch for the Fiducial Floor Follow App
This mode of fiducial follow as of the start of 2020 is not yet released but will be under
our 'demos' git repository in a git branch called
'followPlusCommands'. Also some code in our magni_robot repository will be required.
If desired users who are familiar with the git revision control system can contact us and
we can offer direction on how to use this mode as a Beta Site until it becomes released
in first quarter of 2020 (our goal).

### Running the Fiducial Floor Follow App
The Ubiquity Robotics robots use fiducial markers (aruco markers). The
fiducial_floor_follow launch for the follow.py app takes advantage of aruco_detect (a
ROS node that is part of the Magni package) to detect aruco markers in the image feed
from a camera. The characteristics of the markers’ images enable the robot to compute
its location and that of the markers.

To launch this application the following line will be used in a console

    roslaunch magni_demos fiducial_floor_follow.launch

While this launch file is running the console will show that aruco_detect continuously
searches its field of view for aruco markers. When a marker is found and reported to the
follow.py app the app knows or rather 'sees' the fiducial.
You can use RVIZ to see what the camera sees and adjust lighting if required so that
you can verify that the fiducial is really being seen by messages on the console.

Documents that describe in the level of detail required to develop or use this new mode are in our 'demos/fiducial_follow'
repository in branch 'followPlusCommands'. Refer to [**THIS LOCATION**](https://github.com/UbiquityRobotics/demos/tree/followPlusCommands/fiducial_follow).

This is an exciting new mode that will offer the ability for users to develop custom
applications more easily. We have an example python client that shows how to drive this code over the ROS topic
and it is called follower_controller.py found in above repository and git branch.
