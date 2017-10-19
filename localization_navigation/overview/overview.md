
[This document is being edited by Jim 10/18/17]

# Autonomous Navigation

In order to have a robot be able to respond to a command such as
`go to the kitchen`, we must address the following three requirements:

* The current position of the robot must be known. This is refered to as
  *localization*
* We must have some representation of our environment, so that a path
  can be planned from the robot's current position to the destination.
* There is some way to deal with unexpected obstacles on the way.

Much literature has been published on these taks, and there is plenty of
open source ROS software available.  We provide a overview
of the approaches below, and then discuss the 
*Ubiquity Robotics Approach*

## Localization

![Lidar map](lidar_map.png)


## Navigation

## Obstacle Avoidance

[To be written]

## The Ubuiquity Robotics Approach

In the *Ubiquity Robotics Approach*, we try to provide a rewarding robotics
experience, by applying some constraints to make the taks more achievable.
The intention is that you will be able to get something working, and can then
choose to move on to use more sophisticated approaches.

We did this for two reasons, first we observed that people new to robotics
would struggle to configure tune navigation software. and second we wanted
to provide a low-cost robot base, so we avoided costly LIDAR sensors.
Of course, you are free to add sensors to your robot base, if you so desire.


### Fiducial-Based Localization

See [Fiducial Based Localization](../fiducials/fiducials/md)

### Simple Navigation with move_basic

See [Simple Path Planning](../move_basic/move_basic.md)

## Bibliography

[Do we want one bibliography for the whole documenentation?]

