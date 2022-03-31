---
layout: default
title:  "Board Replacement"
permalink: board_replacement
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

## Removal And Replacement Of Magni Main PC Boards

This page explains the steps required to remove or replace boards from the Magni product that are associated with the main control or power switch board.

The firmware on the MCB is critical to have up to date to avoid prior bugs from showing up when a different MCB is put in the robot.  The MCB board may be shipped with old firmware so all replacements of the MCB should be followed by checking and updating MCB firmware after the replacement board is running.  Refer to [Upgrading MCB Firmware](https://learn.ubiquityrobotics.com/firmware-upgrade) to check and upgrade if needed.

<H4 style="color:red">YOU MUST REMOVE POWER CABLES FROM THE BATTERIES FOR THESE PROCEDURES BECAUSE WE MUST BE SURE THE FULL BATTERY VOLTAGE IS NOT PRESENT ON THE BOARD WHEN REPLACED.</H4>

## Removal Of Main Control Board (MCB)

These steps are taken to remove the main board.  It should be noted that to replace a board these steps can be done in reverse order.

   - Disconnect ALL battery leads from your batteries. We always leave the thick red and black power cables connected to the MCB board because if you get a replacement it will have the power cables attached. The cables are routed in a very specific way with the mounting hardware also done in a precise way thus we do not remove these cables.
   - Disconnect both large multi-pin black wheel cables with inline jacks from the wheels.  These can be very tight so you may need a very good grip and work the connectors gently back and forth as you try to extract.  Be careful to not bang your knuckles as they can release all at once. See the ```The Motor cables to the Wheels``` section of [THIS PAGE](https://learn.ubiquityrobotics.com/unboxing) for pictures
   - Unscrew the 2 screws that hold the small 'Switch Board' to the Magni front panel and place the screws in a safe place.   After screw removal the switch board can be unplugged from the main MCB board and taken out then set aside perhaps near the 2 screws.

     ![Switch Board Screws](SwitchBoardMountingScrews.jpg)

   - If your Magni has the sonar board you should remove it's 50 pin cable from the main MCB board and remove the sonar board to make things easier for this process.    You can see how it is installed and do the reverse that is described on the last half of [THIS PAGE](https://learn.ubiquityrobotics.com/sonar_sensors)
   - We are going to free up the RaspiCam flat white cable so the Raspberry Pi can be removed easier in next step.  Refer to [THIS PAGE](https://learn.ubiquityrobotics.com/camera_sensors) for pictures.  Locate the white flat thin ribbon cable to the camera at the point it gets to the camera.  Take note at this time that the blue tape on the flat cable is away from the RaspiCam PCB which will have to happen as you reassemble later.   NOTE: The jack for the cable is very delicate so just pull back the tabs on each side just a mm or two and do not force it harder or it may break the tabs.  Pull out the cable from the camera end.  
   - Remove the large sheetmetal rectangular Front Bracket that has the raspicam camera bracket riveted to it.  Refer to the middle of [THIS PAGE](https://learn.ubiquityrobotics.com/unboxing) and see the ```Front Bracket``` picture. The removal is done using a long allen wrench with 4mm tip for the bolts that hold this 24mm wide side to side bar to the top shelf of the Magni chassis.   The 4mm allen wrench has to be long enough to go through from the top of the bar all the way to insert into the bolts.

   ![Front Bracket](FrontBracketWithRaspicam.jpg)

   - Unscrew one phillips head screw that may be holding your raspberry Pi to a 20mm tall standoff near the center of the MCB.   Save this and take note of the washers and 1mm thick plastic spacers on some boards and don't loose these tiny parts.
   - Now you may gently ease out the Raspberry Pi. This is a bit tricky so take your time. Care should be taken to never apply any pressure to the very thin Micro SD card in this process as it is easy to break.  You first back the Pi out of the 40 pin tall connector.  Next you have to remove in an upward direction the Raspberry Pi clear of the chassis. Place the Raspberry Pi aside in your work area.  I find that gentle rocking away and towards the MCB at the side of the 20mm standoff while pulling pins out is easiest.
   - Now it is time to unscrew the 4 M3 button head screws that hold the MCB to the chassis using a 2mm long Allen wrench that came with the robot or your own. For some of the screws you may need to run a long allen wrench through access holes in the chassis 5cm away in some cases to get the allen driver straight into the screw head.  Watch where these screws go and don't loose them.
   - Getting the MCB board out requires patience and care.  Do not force anything and do not rub any parts off from contact with the sheet metal on the way out!   It will be removed after no screws hold it by working it out upward now that the front bracket is removed.   As you look behind the MCB in the battery area free up the thick black cables and/or thick power cables then work the MCB out a bit more and eventually it will be guided out in an upward direction.  Just take your time and always watch for any parts that may get bent or scraped so try to do this carefully and take your time.
   - Once the MCB board is out we leave on the thick red and black cables and return those with the MCB if you have been requested to return the board for us to study the failure.

   This completes the mechanical replacement of the MCB


## Check MCB Firmware And Upgrade As required

   The MCB board may be shipped with old firmware so all replacements of the MCB should be followed by checking and updating MCB firmware after the replacement board is running.  Refer to [Upgrading MCB Firmware](https://learn.ubiquityrobotics.com/firmware-upgrade) to check and upgrade if needed.
