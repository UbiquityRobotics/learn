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

Fiducial markers, also known as augmented reality (AR) markers, function as robust QR codes that encode a simple number. ConveyorBot uses these markers as landmarks to navigate through various environments.

## Marker Types

ConveyorBot utilizes several fiducial markers:

|  🟩 **Go marker** displays a single arrow, directing the ConveyorBot's path. | <img src="assets/breadcrumb/go_marker.jpg" > |
|  🟥 **Stop marker** instructs the ConveyorBot to halt at a specific location. Once the robot stops on this marker, it rotates in the direction indicated by the arrow. Users can load or unload packages while the robot remains stationary. To resume operation, press the CONTINUE button on the ConveyorBot's touchscreen. | <img src="assets/breadcrumb/stop_marker.jpg" > |
|  🟦 **Turn marker** should be used to create crossroads by combining multiple markers in the same area without overlap. Each marker points in a different direction, allowing the ConveyorBot to change its driving path. | <img src="assets/breadcrumb/turn_marker.jpg" > |
|  🟪 **Bidirectional marker** is unique, featuring two arrows. The ConveyorBot follows the arrow that requires less rotation. This marker is ideal for two-way routes, such as narrow aisles where separate forward and return paths are impractical. | <img src="assets/breadcrumb/bidirectional_marker.jpg" > |

<br>

ConveyorBot works with both STag and ArUco markers, though STag is the default and recommended option. All marker types function the same way for both systems. Each marker has a unique numeric ID that ensures the correct ConveyorBot maneuvers are executed.

Specifically designed for ConveyorBot, these markers are durable, prefabricated vinyl stickers that can be peeled off and repositioned as needed. However, their adhesive strength decreases with each reapplication. Initially, we suggest using removable tape, like masking tape, to secure the markers. Once you're satisfied with their placement, use the adhesive backing to permanently affix them to the floor.

## Obtaining Additional Markers

If you run out of the markers provided with your ConveyorBot, you can request additional ones from UbiquityRobotics. We can either print new markers for you or send a PDF for you to print independently. Please note that when printing, the markers must be the exact size specified in the PDF (the side of the inner black square containing the white shape should be 18 cm long).

### Packs

A "pack" refers to the number on each marker, indicating the "marker setup" to which it belongs. A marker setup is a group of markers that form a complete navigation path for the robot. Robots cannot move between different marker setups by following any marker sequence. Typically, a marker setup consists of all markers that create the main circular route and its connected branches.

Using pack numbers is not mandatory, but it can be a convenient way to differentiate between multiple marker setups. Each marker within a setup must have a unique ID. If you have two or more setups, you can use a marker with the same ID in each, but only once per setup. The Route Setup section will provide more details on this topic.