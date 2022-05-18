---
title: "Unboxing"
permalink: kinetic_quickstart_unboxing
group: "magni silver (gen 5)"
rosver: kinetic
nav_order: 1
nav_exclude: false
---

# Unboxing and Assembling a Magni Silver

<iframe width="640" height="360" src="https://www.youtube.com/embed/pF38kFOl0Ic" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

The Magni comes almost ready to run. Two 12v SLA (Sealed Lead Acid) batteries should be purchased separately. **A CR2032 coin cell battery is required for proper operation** and must be installed on the back of the main controller board which will be shown below.

A 4mm Allen wrench (for M6 screws) is included in the shipping box. In addition a small Phillips (cross point) screw driver may be needed for mounting the Raspberry Pi camera.

## Opening The Box And Inspecting The Contents

### Step 1 - open the box

Inside the box you will find the battery cables, brackets for the cover plate, fasteners, and a cover plate.

![Shipping Box](assets/unboxing/unb1.JPG)

**IMPORTANT! DO NOT DAMAGE THE THICK STYROFOAM THAT IS IN THE
BOTTOM OF THE CHASSIS, THIS IS LATER USED TO HOLD THE BATTERIES**

![Components](assets/unboxing/unb2.JPG)

After removing the robot, note the cover plate which is stored at the bottom of the shipping box.

![Cover plate](assets/unboxing/unb4.JPG)

The Raspberry Pi 3 + SD image card can be installed if you have your own image with your own software (Silver and Gold). However a default image may already have been installed in the factory.
![Parts](assets/unboxing/unbparts.JPG)

In the small parts bags, you will find fastners and  M4 and M2 Allen wrenches that fit them. The additional sensors (Silver and Gold versions) are wrapped separately.

<!--
 ![RPi 3 installed](unb7.JPG)

 The front and back brackets can be installed using the brackets require a M4 hex Allen wrench. We suggest an extra long (6 cm). The other included fasteners are M3 (M2 hex wrench) and a small Phillips screwdriver for Raspi Cam attachment. [See the detailed section on camera and sensor installation.](camera_sensors)
-->

### Bracket Installation

![Motors to MC](assets/unboxing/unb5.JPG)

The picture above shows a Magni as shipped without the 2 brackets. Take time to ensure that the two Motors are connected, which should have been done at the factory. If they are detached, there are arrows on the connectors that (-> <-) show the alignment.
These connectors are sometimes hard to insert
and separate, because it’s hard to grip them.
Each motor attaches to the black
motor cable that comes from the nearest side of the main PC board to that motor.

![Front](assets/unboxing/MagniUnboxedWithoutTopBrackets.jpg)

The picture above shows a Magni without the front bracket.  In this picture the Raspberry Pi camera cable is attached to the Raspberry Pi itself which is part of setup for the camera. Decide which camera configuration you will want on your Magni. You should now take a detour to look at [**THIS_PAGE**](kinetic_magnisilver_camera)  and decide how you want to mount the camera. Once you decide, use the camera setup page in combination with looking at the pictures on this page about bracket mounting.

![](assets/unboxing/MagniUnboxedAttachingTopFrontBracket.jpg)
Front Bracket

Note that the front and back brackets are different.  The front bracket is the one with a shelf for mounting the Raspberry Pi camera. Using 3 of the M6 flat head hex drive screws attach the bracket.  The Allen wrench will go through the top side of the bracket to reach the screw.    In this case the forward mounted camera was selected and the ribbon cable routed to the camera.  Again, see the camera setup page.

![](assets/unboxing/MagniUnboxedAttachingTopRearBracket.jpg)
Back Bracket Viewed From Behind

The back bracket attachment also uses 3 M6 flat head hex drive screws. Here we see the 3 screws securing the back bracket to the main chassis.

### The Mostly Assembled Magni Prior To Battery Install
![](assets/unboxing/MagniUnboxedFrontViewNoBatteriest.jpg)

The Front Bracket with power switch board and Camera mounted is shown above.


The Main Power switch is the black switch to the left above the first “U” in Ubiquity.  On recent Switchboards it will say 'Main Power' next to the blue LED.

The Motor Power is the red switch for the power to the wheels is above the “y" of Ubiquity printed on the chassis. On recent Switchboards it will say 'Motor Power' next to the red LED.  

For both power switches the 'ON' position is when the switch is out and when pushed in the switch will be  'off'.

The charging port is between the two switches.  

**The next step will be to install the batteries.   At this time push both of the switches IN which will turn all power off as you connect the batteries.**


## Main Power Battery And Wheel Cables Installation

First a picture of a fully assembled Magni using 2 of the 7 ampHour batteries and having the motor cables attached for both wheels.

![](assets/unboxing/MagniBatteryInstallation.jpg)

Use the thick styrofoam cutout piece that came with your Magni in the bed of the chassis. It holds the most common battery types in place even if the robot bumps things or is moved around.  

The picture above shows proper cable connections for the batteries and wheels.  

The wheels should be properly connected from the factory.
As seen in this picture notice that the cable attached to the two green terminal strips seen on the right of the back of the main MCB board goes to the right wheel.   The cable that comes from the two green terminal strips seen on the left of the back of the MCB board goes to the left wheel.   

### Battery Power cable Connectors

The normal MCB power cables attached at the factory are setup to connect to SLA (Sealed Lead Acid) batteries using a F2 (6mm - 1/4 inch) male spade or flat connectors.  Some smaller batteries may use the F1 (3/16 inch) male flat connector and the cables we normally attach will work on those as well.     We also include alternate power cables with 6mm loop connections for larger high capacity batteries with bolts.  Below is a picture of both types of connectors that a battery may required

![Two types of MCB to Battery Cable Connectors](assets/unboxing/PowerCablesWithHeatShrink.jpg)

### Battery to MCB Power Cable details

For the main power cables, the red power cable goes to positive of battery on the right.  The yellow cable connects the positive of the left battery to the negative of the battery shown on the right. The black cable goes from the negative terminal of the battery on the left to ground on the robot.

There are cables for both spade type and screw type battery terminals. A 24 volt battery charger is included in the package (Photo not available). The recommended batteries are of the type specified by UB12xxx where xxx specifies Amp Hours. Commonly UB1250, UB1290, or UB12150 are used. Since it is unknown what size and shape the batteries will be it is the user’s responsibility to see they are secured in the chassis by the use of straps or packing material.

### The Motor cables to the Wheels

The wheels require the use of a high current cable that also holds the wheel encoder wires.   This cable can be very difficult to detact and only a little bit easier to install because it has a very tight fit.    Below is a picture of the male pin end and below that is properly connected motor cable.

Take note of the small arrows which can be hard to see but mark the key location and the cables will only fit together if the two sides align the arrow markings.

![Motor Cable Connectors](assets/unboxing/MotorCables.jpg)



<!-- *{TODO: Somewhere there needs to be a discussion of what size batteries to use.  The spade connector sizes need
to be specified.  The user should be prepared for a current inrush spark? (not sure that this still occurs on initial battery insertion)  Is there a strap to hold the batteries down?  How is it installed? }*

![Final](assets/unboxing/unb-bat.JPG)
-->
## The Real-Time Clock Battery

Make sure to install [the CR2032 real time clock battery](kinetic_overview_batteries#the-real-time-clock-battery).

## Flat Top Plate Install

The top plate should be the last thing attached, using 6  M6 machine screws. Notice that there is one 10mm or so hole in one corner of the plate that is meant to allow the camera if in 'upward' position to see the ceiling so be aware of that as you put the top plate on the robot. 

Note that the countersunk holes should be on the top.

## Power Switches

Now you can turn your robot on by pressing the ON switch (the one coloured BLACK) and follow [the guide on how to connect to it](kinetic_quick_start_connecting).

<img src="assets/Magni_Front_View_2.jpg">