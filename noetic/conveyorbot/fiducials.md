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
|  🟪 **Bidirectional marker** is the only marker with 2 arrows on it. ConveyorBot drives along the arrow that requires less robot rotation. Bidirectional marker is good for two-way routes where ConveyorBot requires to move in both directions for example narrow aisles where there is not enough space for both a forward and return path. | <img src="assets/breadcrumb/bidirectional_marker.jpg" >  |

<br>

ConveyorBot can be used either with STag or ArUco markers (default and recommended being STag). The analogy of marker types is the same for both. Each marker has a unique numeric ID, which is written on it. This way marker types are correctly distinguished which correspond to the correct ConveyorBot maneuvers. 

ConveyorBot markers specifically, are rugged prefabricated sticky vinyl stickers that can be peeled off and placed on another floor location. However, it's not recommended to peel them off too many times as they'll stick less each time. In fact, we recommend that while initially setting up the markers, you shouldn't unpeel the stickers, but just stick them to the ground with a tape which doesn't leave traces on the ground or on the marker, and is easy to unpeel (we recommend masking tape). This is so that it is easier to correct a marker's position on the ground if you realize afterwards that the way in which you initially positioned the marker is not ideal. Once you are happy with the position, you can stick the marker on the ground with the glue on its back.

## Generating Markers

It might happen that you run out of markers which are provided with your Conveyorbot.
Right now, to obtain additional markers, we recommend you contact UbiquityRobotics and we will print new markers for you. If you are interested in printing them on your own, because maybe you want to experiment with different materials of markers, you can also ask us to send you a PDF with the markers. We are also working on a webpage where you will be able to generate them on your own. Note that you need to be careful to print them exactly in the size in which they are in the PDF (a side of the inner black square which contains the white shape inside of it has to be 18cm long). 

### Packs

A "pack" is a number on each marker which says to which "marker setup" a marker belongs.
A marker setup is a group of all of the markers on which a robot can drive. In other words, robots can't go from one marker setup to another by following any marker sequence. A marker setup usually means all of the markers which form the "main circular route" and all of the "branches" connected to it.
It is not necessary to use this number (you might just use number 1 for all marker setups), but it is a convenient way of differentiating between different marker setups if you want to do so. 
It is a rule that each marker on a marker setup has to be unique (has to have a unique ID). If you have 2 or more marker setups, you can use a marker with the same ID on each of them (but only once on each). More information about route setup will be in Route Setup section.