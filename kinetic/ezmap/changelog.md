---
title: "EZ-Map Changelogs"
permalink: ezmap_changelogs
group: ez-map
nav_exclude: true
---

# EZ-Map Changelogs

### 2022-02-16-ezmap-xenial-lxde
```
- fixed the issue where the Pi cannot connect to the MCB every so often, which required a reboot
- reverted to a more stable version of the move_basic navigation node (may reduce reliability of rotational actions)
- more reliable default PID params
- goal lock now renders properly on first load
- undo button now has a 40 second timeout
- fixed velocity smoothing reaching target values
- fixed wheel calibration widget
- rosbag topics are now alphabetically ordered
- added new control schemes, selectable in the settings menu instead of mobile/desktop autodetect
- various minor tweaks and fixes
- native support for LD06 and LS-N301 lidar swapping in the calibration menu
- added api_demo.py script to demo ROS topic and service API functionality
- fixed robot localization jumping in certain cases
- tab title now includes a low battery warning
```

### 2021-12-23-ezmap-xenial-lxde
```
- major code refactor to allow for modular extensions and modifications
- increased move_basic precision, adjusted PID params to reduce tail wagging at all loads
- low battery detection reworked, it now flashes the battery icon when under 50% and does not auto shut down the robot to allow for recovery
- lidar calibration screen is no longer mirrored vertically
- better screen orientation change handling
- various UI adjustments and fixes

- robot movement is now much smoother
- added desktop support (i.e. WASD or IJKL keyboard control)
- landscape joysticks have been redesigned

- route screen now works even without a map (i.e. without a lidar if needed)
- laserscan rendering is now more accurate
- goal lock function now works properly
- fixed orientation of origin arrows
- fixed robot jumping around sometimes
- robot rotation no longer wraps around weirdly every so often
- global triggers being enabled/disabled is now persistent on reboot
- map loader no longer crashes when trying to load corrupted maps, but displays a warning instead
```


### 2021-10-06-ezmap-xenial-lxde

```
- the robot now loads the last used map on boot along with the position it was last at, allowing for - fast resuming of work after a reboot
- added a metric grid background
- added a map origin (0,0) indicator with the red axis showing the X direction and green showing the direction
- switched laser scan display to median downsample, which makes it more stable
- fixed robot flickering issue
- goal lock now persists as a saved setting
- calibration splash screen now also gets hidden for good if restart or the side X is pressed
- added proper versioning, accessible by running rosversion ezmap_web in a terminal
- linear and angular speed settings now also double as the default autonomous navigation speed, not - just remote control
- if goals are locked and the robot is above a goal it's now possible to drag and drop it, as - - previously the goal would consume the mouse press
- other minor tweaks and fixes
```