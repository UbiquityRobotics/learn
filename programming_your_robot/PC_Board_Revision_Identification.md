---
layout: default
title:  "PC Board Revision Identification"
permalink: PC_Board_RevId
---
#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/)

## PC Board Revision Identification
version: 20191103

### Main Control Board Identification

All of the Main Control Boards, also called MCB, have a version number on the top of the board that is printed along the left edge of the PC board on the top copper layer.   Because prior to rev 5.2 the version number is very hard to read we will supply pictures to help with board version identification.

Starting with revision 5.1 the rev was shown on the top silkscreen under the large text of Ubiquity Robotics.  This text is normally under the Raspberry Pi controller and so was hard to see.

Starting with version 5.2 the large text for the board revision will be up to mid left of the board and printed in bright white silkscreen.  So starting with version 5.2 identification will be quite easy.

A detailed list of MCB hardware changes as well as firmware revisions can be found [here](https://github.com/UbiquityRobotics/ubiquity_motor/blob/indigo-devel/Firmware_and_Hardware_Revisions.md).

### Revision 5.1 Main Control Board

The following items identify a rev 5.1 board.

* The white label with board serial number on the top will start with 39 (2019)
* Mid right edge of the board will have a 1.4 inch square outline for a display
* The P2 jack in mid right will be black plastic female 4-pin jack
* A thick white strip is on the right edge for notes as required.
* The large black MosFet transistor in lower right will fit the pads on the PC board.

<img src="https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/MagniIdentificationForMcbRev5p1.jpg
" />

### Revision 5.0 Main Control Board

These items identify a rev 5.0 board besides the top copper board rev in bottom left.

* The white label with board serial number on the top will start with 38 (2018)
* Mid right edge of the board will have a 1.4 inch square outline for a display
* The P2 jack in mid right will be a 4-pin male header
* A thick white strip is on the right edge for notes as required.
* The upper left of the board will have a large capacitor on its side with white glue

<img src="https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/MagniIdentificationForMcbRev5p0.jpg
" />  

### Revision 4.9 Main Control Board

These items identify a rev 4.9 board besides the top copper board rev in bottom left.

* It has no serial number stick-on tag like all rev 5.x boards will have on the MCB
* No white stripe of top silkscreen along the right edge and it has not P2 there either
* The top layer text on the left will be bordered by top layer full copper PC layer.

<img src="https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/MagniIdentificationForMcbRev4p9.jpg
" />  

### Revision 4.7 Main Control Board

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

There were several versions of switch boards from pre-production through first shipment of units using the rev 4.9 MCB. The revision number only started to appear on the switch boards at the time of the rev 5.0 MCB boards.

* The main thing to watch for is if the board has 3 resistors it is old for rev 4.9 or earlier MCB boards.

* A REV 5.x MCB  REQUIRES A 4 RESISTOR DESIGN AND BE LABELED REV  2.x or later.

* A rev 2.x switch board with 4 resistors can be used with earlier MCB boards.

<img src="https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/MagniIdentificationOfSwitchBoards.jpg
" />  
