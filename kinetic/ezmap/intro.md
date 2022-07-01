---
title: "Introduction"
permalink: kinetic_ezmap_intro
group: ez-map
rosver: kinetic
nav_order: 1
nav_exclude: false
---

# Introduction

> ![](../../assets/ezmap/ezmap_logo.png)

Welcome to EZ-Map. Following this wiki should allow you to to map a room with a lidar and set up routes to execute, or drive the robot around using video streaming and remote control.

<iframe width="640" height="360" src="https://www.youtube-nocookie.com/embed/r7kaECd3c0o" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<hr>

## Setup

Make sure you have all of these items assembled and ready to go before continuing.

### Magni robot

You'll need a robot to run the software, so if you are unfamilliar with the basics start with the [Overview](kinetic_overview_need_to_know) category in the navbar to get up to speed on how to use and [assemble](kinetic_quickstart_unboxing) your robot.

If you intend to use EZ-Map with the optional **Tower & Shell**, make sure you install that as well using the [assembly instructions](kinetic_quickstart_shell_tower).

### LiDAR sensor

EZ-Map requires a lidar for mapping, so one will need to be mounted before you can start.

- [UR-50 Long Range LiDAR](kinetic_ur50_lidar)

- [UR-12 Short Range LiDAR](kinetic_ur12_lidar)

While those two are supported out of the box, it’s possible to run the suite with other lidars sensors, however the ROS driver for it has to be additionally installed and properly launched instead of the default one. Please contact ubiquity support for help with that.

**NOTE:** Most of the documentation on this wiki (outside the EZ-Map section) is intended for people working with the a bare bones robot with a stock vanilla image, so while following assembly guides is fine, keep in mind that the manual software configurations listed should be ignored for EZ-Map, since the setup may be slightly different and you should be able to make changes from a GUI and that will likely overwrite your manual file edits.

### Pi Camera

The camera used for remote viewing and fiducial marker detection that should already be included with the Magni, see [the setup instructions page](kinetic_magnisilver_camera) on how to mount it. 

### EZ-Map software image

Usually delivered via email sendout. Once you have your img.xz file downloaded see the [guide on how to flash one it to a micro SD card](kinetic_quick_start_microsd).

See the [changelogs page](ezmap_changelogs) to verify you have the latest version for your platform)


## Next up?

Once you have all that set up and the robot boots, continue on [the getting started page](kinetic_ezmap_gettingstarted).

