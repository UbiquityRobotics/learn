---
layout: default
title:  "PC Board Revision Identification"
permalink: PC_Board_RevId
---
#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/)

## PC Board Revision Identification
This page describes how to identify Magni PC board revisions of the ```Main Control Board (MCB)``` and the ```switch board``` for versions were the rev does not print on the board or is very hard to see due to poor contrast.

### Main Control Board Identification

All of the Main Control Boards, also called MCB, have a version number on the top of the board that is printed along the left edge of the PC board on the top copper layer.   

Because prior to rev 5.2 the version number is very hard to read we will supply pictures to help with board version identification prior to rev 5.2

A detailed list of main MCB hardware changes as well as firmware revisions can be found on our [Firmware_Hardware_Revisions](https://github.com/UbiquityRobotics/ubiquity_motor/blob/kinetic-devel/Firmware_and_Hardware_Revisions.md) github page.

#### Revision 5.2 Main Control boards

Starting with version 5.2 the large text for the board revision will be up to mid left of the board and printed in bright white silkscreen.  This page will not describe rev 5.2 boards because the revision is very easy to see.


#### Revision 5.1 Main Control Board

Starting with revision 5.1 the rev is also shown on the top silkscreen under the large text of Ubiquity Robotics.  This text is normally under the Raspberry Pi controller and so was hard to see.

The following items identify a rev 5.1 board.

* The white label with board serial number on the top will start with 39 (2019)
* Mid right edge of the board will have a 1.4 inch square outline for a display
* The P2 jack in mid right will be black plastic female 4-pin jack
* A thick white strip is on the right edge for notes as required.
* The large black MosFet transistor in lower right will fit the pads on the PC board.

<img src="https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/MagniIdentificationForMcbRev5p1.jpg
" />

#### Revision 5.0 Main Control Board

These items identify a rev 5.0 board besides the top copper board rev in bottom left.

* The white label with board serial number on the top will start with 38 (2018)
* Mid right edge of the board will have a 1.4 inch square outline for a display
* The P2 jack in mid right will be a 4-pin male header
* A thick white strip is on the right edge for notes as required.
* The upper left of the board will have a large capacitor on its side with white glue

<img src="https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/MagniIdentificationForMcbRev5p0.jpg
" />  

#### Revision 4.9 Main Control Board

These items identify a rev 4.9 board besides the top copper board rev in bottom left.

* It has no serial number stick-on tag like all rev 5.x boards will have on the MCB
* No white stripe of top silkscreen along the right edge and it has not P2 there either
* The top layer text on the left will be bordered by top layer full copper PC layer.

<img src="https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/MagniIdentificationForMcbRev4p9.jpg
" />  

#### Revision 4.7 Main Control Board

These items identify a rev 4.7 board besides the top copper board rev in bottom left.
THE REV 4.7 BOARD WAS A PRE-PRODUCTION BOARD FOR EVALUATION

* It has no serial number stick-on tag
* No white stripe of top silkscreen along the right edge and it has not P2 there either
* The board has not top layer of copper so large areas much darker than other areas will show through the top of the board and the text on the far left will not seem to be 'boxed' in copper.
* The 14-pin jack that holds the switch board had to be cut to not hit a large transistor.
* The large 50-pin jack in upper right will be a female jack for this pre-production board

<img src="https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/MagniIdentificationForMcbRev4p7.jpg
" />  


### The Power Switch Board Revisions

There were several versions of switch boards from pre-production through first shipment of units using the rev 4.9 MCB. The revision number only started to appear on rev 2.0 switch boards shipped at the time of the rev 5.0 MCB boards.


#### Switch Boards With Remote Switch Connectors

In order to support user needs to place the main power switch and/or the ESTOP switch in a location that is on their robot cover or perhaps is more accessible due to the customer physical additions we developed the revision 2.2 switch board seen below

<img src="https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/SwitchBoardRev2p2.jpg
" />  

The revision 2.2 board has P202 seen in the back right that is wired in series with the red keycap ESTOP switch on the board.   Our plan is we will ship the connector that mates with P202 that has one piece of wire sorted to itself.  In this way P202 is shorted from the factory and a user may remove this jack and put two wires going to his own ESTOP switch for his own robot needs.

On the back left you see P201 which is wired in parallel with the black main power switch.  The thought here is users who want a remote main power switch connect a connector to two wires and then leave the installed main power switch OFF or pushed in.  The customer switch will then be the power switch.

#### Switch Boards To Support Rev 5.x MCB Boards

* The main thing to watch for is if the board has 3 resistors it is old for rev 4.9 or earlier MCB boards.

* A REV 5.x MCB  REQUIRES A 4 RESISTOR DESIGN AND BE LABELED REV  2.x or later.

* A rev 2.x switch board with 4 resistors can be used with earlier MCB boards.

<img src="https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/MagniIdentificationOfSwitchBoards.jpg
" />  

#### Very early pre-production Switch boards

In early prototypes there were switch boards with large white switches that had green leds in them to show the state.  These should not be in production units unless some sort of replacement had to happen early in first production units.
