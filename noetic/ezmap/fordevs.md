---
title: "Advanced Documentation"
permalink: noetic_ezmap_dev_docs
group: ez-map
rosver: noetic
nav_order: 3
nav_exclude: false
---

# Advanced Documentation

(This is a noetic page, sanity check for evaluation)

## Views and Widgets

EZ-Map at it's core consists of a system that loads custom plugins, which in turn add the main functionality and allow for high customisability.

> ![](assets/ezmap/ezmap_core.png)

## Remote Control

Joysticks for remote control are always present and offer a few different modes:

- **Portrait Arrows**: tap arrows to move the robot in a specific direction, much like Robot Commander

- **Portrait Joystick**: drive the robot using one single joystick (must be enabled in the settings menu)

- **Landscape Mode**: Allows driving with two joysticks, one for forward movement and one for turning (as seen on picture above)

- **Desktop Mode**: available on non-touchscreen systems with a physical keyboard and allows driving with WASD or IJKL keys

## Widget Plugins

Plugins usually fit into two groups: windows or widgets. Widgets appear as buttons on the screen and usually open a specific menu or run an action. 

#### Default:

- <img src="assets/ezmap/settings.svg" alt="" width="70"> Settings
- <img src="assets/ezmap/viewswitch_landscape.svg" alt="" width="70"> View Switch

#### Optional:

- <img src="assets/ezmap/100.svg" alt="" width="70"> Battery
- <img src="assets/ezmap/calibrations.svg" alt="" width="70"> Calibrations
- <img src="assets/ezmap/record_off.svg" alt="" width="70"> Telemetry Record
- <img src="assets/ezmap/photo.svg" alt="" width="70"> Photo

---

The same installed widgets will also appear on the portrait mode button bar.

<img src="assets/ezmap/portrait.png" alt="" width="250">
