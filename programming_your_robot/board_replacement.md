---
layout: default
title:  "Board Replacement"
permalink: board_replacement
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

## Removal And Replacement Of Magni Main PC Boards

This page explains the steps required to remove or replace boards from the Magni product that are associated with the main control or power switch board.

This is in text form at this time and we hope to add pictures in the future.

<H4 style="color:red">YOU MUST REMOVE POWER CABLES FROM THE BATTERIES FOR THESE PROCEDURES BECAUSE WE MUST BE SURE THE FULL BATTERY VOLTAGE IS NOT PRESENT ON THE BOARD WHEN REPLACED.</H4>

## Removal Of Main Control Board (MCB)

These steps are taken to remove the main board.  It should be noted that to replace a board these steps can be done in reverse order.

   - Disconnect ALL battery leads from your batteries but at this stage you can leave the thick red and black power cables connected to the old board.
   - Disconnect both large multi-pin black wheel cables with inline jacks from the wheels.  These can be very tight so you may need a very good grip and work the connectors gently back and forth as you try to extract.  Be careful to not bang your knuckles as they can release all at once at the end.
   - Unscrew the 2 screws that hold the small 'Switch Board' to the Magni front panel and place the screws in a safe place.   After screw removal the switch board can be unplugged from the main MCB board and taken out then set aside perhaps near the 2 screws.
   - If your Magni has the sonar board you should remove it's 50 pin cable from the main MCB board and remove the sonar board to make things easier for this process.    You can see how it is installed and do the reverse that is described on the last half of [THIS PAGE](https://learn.ubiquityrobotics.com/camera_sensors)
   - We are going to free up the RaspiCam flat white cable so the Raspberry Pi can be removed easier in next step.   Locate the white flat thin ribbon cable to the camera at the point it gets to the camera.  Take note at this time that the blue tape on the flat cable is away from the RaspiCam PCB which will have to happen as you reassemble later.   NOTE: The jack for the cable is very delicate so just pull back the tabs on each side just a mm or two and do not force it harder or it may break the tabs.  Pull out the cable from the camera end.  Again please see [THIS PAGE](https://learn.ubiquityrobotics.com/camera_sensors)
   - Unscrew one philips head screw that may be holding your raspberry Pi to a 20mm tall standoff.   Save this and don't loose it.
   - Now you may gently pry out the Raspberry Pi.  Care should be taken to never apply any pressure to the very thin Micro SD card in this process as it is easy to break.  Place the Raspberry Pi aside in your work area.  I find that gentle rocking away and towards the MCB at the side of the 20mm standoff while pulling pins out is easiest.
   - Now it is time to unscrew the 4 M3 screws that hold the MCB to the chassis using a 2mm long Allen wrench that came with the robot or your own. For some of the screws you may need to run a long allen wrench through access holes in the chassis 5cm away.
   - Getting the MCB board out requires patience and care.  Do not force anything!   It will be removed out the front in the general direction of 'up'.   As you look behind the MCB in the battery area free up the thick black cables and/or thick power cables then work the MCB out a bit more and eventually it will be guided out in an upward direction.
   - As a final step we use the 5mm allen wrench to remove the 2 main battery power cables from the board.  Note the order of the lockwasher and two nuts and perhaps put each screw back on the end of each battery cable with washer and nuts in same order to remember the order.
