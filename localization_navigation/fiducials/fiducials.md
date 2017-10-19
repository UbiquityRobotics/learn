
# Fiducial-Based Localization

In this section, we describe our approach to fiducial-based localization.c
This is one of the ways in which a robot's position in the `world` can
be determined.  For a discussion of other approaches, see the
[Navigation and Localization Overview](../overview/overview.md).

This document assumes that you are running the software on a Ubiquity
Robotics robot base, using the supplied Raspberry Pi camera.

## Basic Concept

If you're eager to start localizing your robot, and don't wish to read about
how fiducial localization works, you can skip reading this sub-section.

The Ubiquity Robotics localization system uses a number of fiducial markers
of known size to determine the robot's position.  Detection of the markers
is done by the [aruco_detect node](http://wiki.ros.org/aruco_detect).
For each marker visible in the image, a set of vertices in image coordinates
is produced.  Since the *intrinsic parameters* of the camera and the size
of the fiducial are known, the pose of the fiducial relative to the camera can
be estimated. Note that if the camera intrinsics are not known, they can
be determined using the process described in the
[camera calibration tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration).

The diagram below shows the coordinate system of a fiducial marker, which has a
length of *2d*.  The image coordinates *(x, y)* of each vertex correspond
to a ray from the camera.  The pose estimation code solves a set of linear
equations to determine the world *(X, Y, Z)* coordinate of each of the
vertices. From this, we obtain the *transform* of the fiducial's coordinate
system to the camera's coordinate system *T<sub>fid_cam</sub>*. This
represents the *pose* of the marker in the camera's coordinate system.  Since
we know the camera's pose in the coordinate system of the robot,
*T<sub>base_cam</sub>*, we can combine the two to determine the robot's pose
in the map, *T<sub>map_base</sub>*.

![Fiducial coordinate system](fiducial.png)

Construction of the map is performed by the
[fiducial_slam node](http://wiki.ros.org/fiducial_slam). It performs
Simultaneous Localization and Mapping (SLAM) based on the fiducial transforms.
The mapping part requires that more than one marker is visible in an image.
In the diagram below, two fiducials, *fid1* and *fid2* are shown. If *fid1*
is at a known pose in the world, *T<sub>map_fid1</sub> and we know the
marker to camera transforms for both markers, we can compute the pose of
*fid2* thus:

*T<sub>map_fid2</sub> = T<sub>map_fid1</sub> * T<sub>cam_fid2</sub> * T<sub>fid1_cam</sub>*

![Fiducial coordinate system](two_fiducials.png)

## Printing Fiducials



## Using rviz to monitor map creation

![Visualizing with rviz](fiducial_rviz.png)

