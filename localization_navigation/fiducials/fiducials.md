
# Fiducial-Based Localization

In this section, we describe our approach to fiducial-based localization.c
This is one of the ways in which a robot's position in the `world` can
be determined.  For a discussion of other approaches, see the
[Navigation and Localization Overview](../overview/overview.md).

This document assumes that you are running the software on a Ubiquity
Robotics robot base, using the supplied Raspberry Pi camera.

## Basic Concept

The Ubiquity Robotics localization system uses a number of fiducial markers
of known size to determine the robot's position.  Detection of the markers
is done by the [aruco_detect node](http://wiki.ros.org/aruco_detect).
For each marker visible in the image, a set of vertices in image co-ordinates
is produced.  Since the *intrinsic parameters* of the camera and the size
of the fiducial are known, the pose of the fiducial relative to the camera can
be estimated. Note that if the camera intrinsics are not known, they can
be determined using the process described in the
[camera calibration tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).


![Fiducial co-ordinate system](fiducial.png)

T<sub>fid_cam</sub>

![Fiducial co-ordinate system](two_fiducials.png)

## Printing Fiducials



## Using rviz to monitor map creation



