---
title: "Beginner Tutorial"
permalink: noetic_ezmap_beginner_tutorial
group: ez-map
rosver: noetic
nav_order: 2
nav_exclude: false
---

# Beginner Tutorial

(This is a noetic page, sanity check for evaluation)

## Setup

### Sensors

You should have two sensors installed on the robot before starting up the software:

- Pi Camera Module v2

- Leishen N301 Lidar (shown below)

> ![](assets/ezmap/lidar.png)

When the robot first starts you’ll be asked to provide a location and orientation for both, for best results. The menu can also be skipped and later accessed in the calibration menu.

It’s possible to run the suite with other lidars sensors, however the ROS driver for it has to be additionally installed and properly launched instead of the default one.


### Flashing the SD card

The first step is taking the downloaded SD card image (image.img.xz) and flashing it onto an SD card of at least 16GB in size. We recommend using [etcher](https://www.balena.io/etcher/) to flash the image on most systems or [Win32DiskImager](https://win32diskimager.download/) as an alternative on Win10 (requires .xz files to be extracted first. Under Ubuntu Linux you can also use the GNOME Disks tool to flash images. If you haven't installed it, simply run sudo apt install gnome-disk-utility. Then you can double click on the downloaded image file, the GNOME Disks tool will automatically come up, and you can direct it to expand the image onto an SD card drive attached to your computer.


## Video Streaming

Assuming the robot has a Pi Camera V2 installed and the ezpkg_video_screen package, then video streaming will be shown as the default view.

Depending on device capability, the system will automatically switch between fast WebRTC streaming and fallback HTTP streaming.

> ![](assets/ezmap/ezmap_video.png)


## Mapping

To use the robot’s autonomous functions, first switch to the mapping view if multiple views are installed. The interface shown should look as follows, the mapping should start automatically:

### Landmarks
The hybrid particle filter SLAM also supports landmarks as a way to improve localization accuracy. These are by default set up to be aruco markers that can be printed out and laid out around in view of the camera.

Once located with reasonable accuracy they will be implemented into the map and rendered as shown on the right.

In order to get positional data correct it is crucial to set up the correct camera position. This can be done in /etc/ubiquity/robot.yaml 

