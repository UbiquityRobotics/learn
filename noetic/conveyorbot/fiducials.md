---
title: "Fiducial Markers"
permalink: noetic_conveyorbot_fiducials
group: conveyorbot
rosver: noetic
nav_order: 3
nav_exclude: false
---

# Fiducial Markers

> *"A fiducial marker or fiducial is an object placed in the field of view of an imaging system, for use as a point of reference or a measure. It may be either something placed into or on the imaging subject, or a mark or set of marks in the reticle of an optical instrument."*

Fiducial markers (also known as AR markers) are a type of robust QR code that only encodes a simple number, and are used by ConveyorBot as landmarks, which it can use to navigate the environment of your choosing.

## Marker Types

ConveyorBot uses several types of fiducials:

|  🟩 **Go marker**  has a single arrow which point to the direction where the ConveyorBot will drive. | <img src="assets/breadcrumb/go_marker.jpg" >  |
|  🟥 **Stop marker** is made to halt the ConveyorBot in that spot. After the ConveyorBot moves on top of the Stop marker it rotates in a direction of a marker arrow. While ConveyorBot waits on a Stop marker the user can load or unload the packages. The robot will resume following markers when user presses the CONTINUE button on the touchscreen of the ConveyorBot. | <img src="assets/breadcrumb/stop_marker.jpg" >  |
|  🟦 **Turn marker** is made for creating crossroads. Crossroad is a couple of markers in the same location (they must not overlap!), where each is pointing in an arbitrary/different direction. Purpose of Turn markers and crossroads is to change the driving direction of the ConveyorBot. | <img src="assets/breadcrumb/turn_marker.jpg" >  |
|  🟪 **Bidirectional marker** is the only marker with 2 arrows in it. ConveyorBot drives allong the arrow that requires less robot rotation. Bidirectional marker is good for two-way routes where ConveyorBot requires to move in both directions for example narrow aisles where there is not enough space for both a forward and return path. | <img src="assets/breadcrumb/bidirectional_marker.jpg" >  |

<br>

ConveyorBot can be used either with STag or ArUco markers (default and recommended being STag). The analogy of marker types is the same for both.

Each marker has a unique numeric ID, which is written on it. This way marker types are correctly distinguished which correspond to the correct ConveyorBot manevers.

### How to place the markers

The most important things to remember before setting up the route of markers are:
- Each marker should point in the direction of the next one
- TURN markers should be used to change driving direction (In the crossroads - multiple markers at one position pointing in different directions)
- BIDIRECTIONAL markers should be used for two-way routes, where robot can drive in both directions and not for changing the driving direction. In other words, robot should never come sideways to the BIDIRECTIONAL marker, since it may turn in the wrong direction as intended.
- GO marker should be used for one-way routes, where robot can drive only in one direction


Markers are sticky stickers that can be peeled off and placed on another floor location. But it is not recommended to peel it off too many times as the glue on the marker can dry out.

An example of a simple marker layout is shown in the figure below.

<img src="assets/breadcrumb/Map_example1.png" >


## Generating Fiducials

We've provided a selection of scripts that offer a simple terminal UI, which enables user to re-configure fiducials according to their need.

Said scripts can be found in the ConveyorBot repository in the `ConveyorBot/breadcrumb_detect/scripts` directory:

- `ArUco/gen_markers.py` -> generate Aruco markers with frames
- `STag/gen_markers_autogen.py` -> generate STag markers with frames by generating also the inner part.
- `STag/gen_markers.py`  -> generate STag markers with frames by getting the inner part from downloaded image library. Download the required images from [here](https://drive.google.com/drive/folders/0ByNTNYCAhWbIV1RqdU9vRnd2Vnc).

Script were tested using python2.7.

### ArUco Dictionary ID

You can generate ArUco markers from any dictionary you prefer.
Make sure you set the corresponding `dictionary` parameter in launch file.
We performed most of the test with dictionary with **ID 7**.
For more information about ArUco refer to [`ArUco`](http://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html).

### STag Library HD

You can generate STag markers from any library you prefer.
Make sure you set the corresponding `libraryHD` parameter in launch file.
We performed most of the test with library **HD15**, which suited our needs.
For more information about STag refer to [`STag`](https://github.com/usrl-uofsc/stag_ros).

### Packs

Generally each pack should correspond to a standalone route, but you can generate yourself packs according to your preference.
Take note that each marker has a unique id, while a number is unique only to the pack.
