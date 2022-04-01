---
title: "Advanced Documentation"
permalink: ezmap_dev_docs
group: ez-map
rosver: kinetic
nav_order: 3
nav_exclude: false
---

# Advanced Documentation

(This is a kinetic page, sanity check for evaluation)

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

#### Required:

- ![settings.svg](assets/ezmap/settings.svg) Settings
- ![viewswitch_landscape.svg](assets/ezmap/viewswitch_landscape.svg) View Switch

#### Optional:

- ![100.svg](assets/ezmap/100.svg) Battery
- ![calibrations.svg](assets/ezmap/calibrations.svg) Calibrations
- ![record_off.svg](assets/ezmap/record_off.svg) Telemetry Record
- ![photo.svg](assets/ezmap/photo.svg) Photo


---

The same installed widgets will also appear on the portrait mode button bar.

<img src="/assets/ezmap/portrait.png" alt="" width="250">
