---
title: "Motor Control Board (MCB)"
permalink: noetic_magnisilver_mcb
group: "magni silver (gen 5)"
rosver: noetic
nav_order: 3
nav_exclude: false
--- 

# Motor Control Board 

This page explains the main control and power switch boards.

## Full PCB Layout

[Download PDF](Magni_MCB_pinout.pdf)

<embed src="Magni_MCB_pinout.pdf" width="100%" height="600px">

<hr>

## Finding The Firmware Version Info For Your Robot

Here is how to get firmware version and date once you have opened a SSH window into your robot.  This works by filtering off just the 2 items of interest which are ```Firmware Version```  and  ```Firmware Date```

    rostopic echo /diagnostics | grep -A 1  'Firmware [DV]'

#### Firmware Release Versions

The table below shows the default and latest available versions of firmware.  

The ```Rate``` in the table is the rate at which the MCB ```STATUS``` led will blink which is handy to visually check. It is best to count the time of 4 or more blinks then divide by that number for more accuracy. If the blink rate is found to be other than in the table it is possible you have a beta or non-approved version.

DateCode was started around version v35 and shows up in /diagnostics topic along with the version.  DateCode is in YYYYMMDD format for releases and YYMMDD for beta releases. It is the date of that particular version.  If you have a version that the date is before the date in the table it is likely a Beta or unofficial release. DateCode did not start till around v35.

| Ver | Rate | DateCode | Description |
| ------- | ---- | ---- | ----------- |
| v28 | 6.0 | NA | Depreciated Production shipment version that does wheel movement check on startup. Users should do a firmware upgrade from this very old version |
| v32   | 5.0 | NA | Depreciated Production firmware used in 2019.  |
| v35 | 4.0 | 20190815 | Last well known good release as of Sept 2020 with double resolution wheel encoders and many improvements.    Requires host side software update done after 11/10/2019 |
| v37 | 4.75 | 20200620 | Use in manufacturing only.  1st with a selftest. |
| v38 | 5.25 | 20201006 | Depreciated Beta release for some fixes. |
| v39 | 5.5 | 20201129 | Non-Magni release for in development 4wheel drive unit |
| v40 | 5.75 | 20201209 | Magni current release for most recent release candidate.  This is the default if just an enter is done when asking for version |
| v43 | 4.0 | 20210829 | Magni beta release with some fixes and minor new abilities |

To see more details about our firmware as well as our hardware revisions for the motor controller please view [this page](https://github.com/UbiquityRobotics/ubiquity_motor/blob/kinetic-devel/Firmware_and_Hardware_Revisions.md).

#### Checking ROS /diagnostics Topic For Firmware

The firmware version as well as the firmware release date are published by the motor_node to ROS topic  ```/diagnostics```

Use the following command for a couple seconds then use Control-C to stop the fast moving output.  Scroll back and look for ```"Firmware Version"``` and ```"Firmware Date"```

    rostopic echo /diagnostics

## Firmware Upgrade

Sometimes you may need to get a firmware upgrade for your Motor Controller board.
This operation is less frequently needed and in general we advise customers to do the firmware upgrade only if they run into an issue we know we have addressed in an update.

Before you can upgrade firmware, your robot needs to be connected to the
internet or you must have the firmware file and copy it to the robot.  See [Connecting the Robot to Your Network](noetic_quick_start_connecting).

We upgrade firmware using a tool that will by default install the latest released firmware. If an older fimware version is needed, it will require manual version entry seen in the Version column of the table below.

Improvements to out motor controller fimware (v35 and later) need the latest Linux host side software. Installing the latest Linux host side software can be done by following a [guide here](noetic_quick_start_microsd).

### Installation
To install any firmware first log into the Raspberry Pi:

    ssh ubuntu@YOURROBOT.local

all of the following commands are to be run on the Pi.  

### First Prevent The motor_node From Talking to the MCB

There are now two ways to prevent the ROS motor_node from constant queries and commands to the MCB board over the serial connection.  
For a standard Magni configuration it is best to completely stop the Ubiquity Software and then do the firmware upgrade.
As of our motor_node available as of April 2021 you can also do a new method that does not shutdown everything but only impacts the motor node by a disable or relinquish of the serial port to allow direct serial control which in this case would be for the firmware upgrade.   We suggest for a standard Magni use the full system stop.

#### Full Stop Of the Magni Software Is The Best Method

The following command fully stops standard Magni software but may not fully stop other configurations for some new Magni applications to be announced in 2021

    sudo systemctl stop magni-base

Once you have completed a firmware upgrade you are best off to reboot when using this method.   

    sudo shutdown -r now

#### A soft disable of motor_node control of the MCB Is Now Possible
As of about April 2021 users who have very current ubiquity_motor repository code are able to do a softer stop of the motor node and then later re-enable the motor node.   We suggest use of the prior method but show this to be complete.

    rostopic pub /system_control std_msgs/String "motor_control disable"

After the firmware upgrade you may still just reboot OR use this command

    rostopic pub /system_control std_msgs/String "motor_control enable"

### After The MCB is Free over serial do the upgrade_firmware

You can use the firmware upgrade utility that must have web access to fetch the most current released firmware OR you can load a version you have fetched yourself and placed in a file.

#### Upgrading Firmware From The Web If You Have A connection.
To use this method you would generally connect the robot to your own WiFi or use an ethernet cable into the Pi to your own network.  

Run the firmware upgrade utility:

    rosrun ubiquity_motor upgrade_firmware.py

After entering your email address, you should receive an email with an access token.
Enter this token and either accept the default version which is seen in the above table or specify a different version perhaps for latest features or perhaps an older trusted release (eg. specify version 35 by typing in 'v35'). The download will start and upgrade the motor controller board.

Should any problems show up you are always allowed to do this firmware upgrade again to select a version you know was best for your needs.

The process should take less than a minute; make sure that power is
not interrupted during this process. Otherwise your robot may become
inoperable and require factory servicing.

When done, reboot the robot with `sudo shutdown -r now` if you used

You are now on the latest version of the firmware.

#### Firmware Installation From A File

In some support situations you may be working with the development team here and be given a beta version of software in the form of an encrypted file.

Upgrade Firmware from a file that you place on your system:

    rosrun ubiquity_motor upgrade_firmware.py --file  v40_20201209_enc.cyacd

The name above is an example file for released v40 firmware. For beta firmware if you are told to use a given version then you may not be able to request a beta version so you would have to check our please visit  [OUR REPOSITORY](https://github.com/UbiquityRobotics/ubiquity_motor)  and navigate into the firmware folder.

As of mid 2021 we have decided on a naming convention for our firmware which will start the name with   vXX_YYMMDD  where XX is the firmware rev and YYMMDD is they daycode.  We use a daycode so that we can have 'beta' or early release tests.  When we do have a beta firmware we will try to end the name in  beta and may say other text.  So the vXX_YYMMDD is the only fixed format specified but other characters may be after that format.   

#### Firmware Installation Using Non Standard Serial support

In some situations the serial port used for controlling the robot may not be the default port on the Raspberry Pi host computer attached to the MCB.

To specify a different serial port such as one plugged into usb:

    rosrun ubiquity_motor upgrade_firmware.py --device /dev/ttyUSB0

<hr>

## PC Board Revision Identification
This page describes how to identify PC board Revisions used in the Magni robot.

- Raspberry Pi Host Computer
- Main Control Board (MCB)
- Switch board

### Raspberry Pi Host Computer

Starting in mid 2020 we started to ship the Raspberry Pi 4 host computer that is inserted into the MCB board so that only the bottom is visible and even that is hard to see due to the robot chassis.  The easiest way to tell which general Raspberry Pi you have (3 vs 4) is to look at the USB and Ethernet jacks as follows.
- Pi 4 middle 2 jacks use blue plastic for USB 3.0 ability.
- Pi 4 Ethernet jack will be the top metal jack (bottom on Pi3)
- Pi 3 will have a large square 1.2cm black chip on the bottom of the board

The best identification is to use this linux command from an ssh command line shell into the robot which will also show the rev of Pi3 or Pi4

    cat /sys/firmware/devicetree/base/model

- Typical Pi3 reply:  `Raspberry Pi 3 Model B Plus Rev 1.3`
- Typical Pi4 reply:  `Raspberry Pi 4 Model B Rev 1.2`

### Main Control Board Identification

All of the Main Control Boards, also called MCB, have a version number on the top of the board that is printed along the left edge of the PC board on the top copper layer.   

Because prior to rev 5.2 the version number is very hard to read we will supply pictures to help with board version identification prior to rev 5.2

#### Revision 5.2 And Later Main Control boards

Starting with version 5.2 the large text for the board revisions are printed in bright white silkscreen on the left edge of the board.  This page will not describe physical differences to identify the boards because the board revision is clearly marked.  Below shows rev 5.2 and rev 5.3 left edge markings.

![Current MCB Board Revision Markings](assets/support/Mcb_5p2and5p3_BoardRevMarking.jpg)

#### Revision Markings On Rev 5.1 and earlier

All MCB boards have the revision on the left edge of the PCB but bright white silkscreen was only started to be used as of rev 5.2.  Below is an enhanced picture of a rev 5.1 board to better show what is there.  These markings are very low contrast but are present on all MCB boards.

![Older MCB Board Revision Markings](assets/support/Mcb_5p1_BoardRevMarking.jpg)


#### Revision 5.1 Main Control Board

Starting with revision 5.1 the rev is also shown on the top silkscreen under the large text of Ubiquity Robotics.  This text is normally under the Raspberry Pi controller and so was hard to see.

The following items identify a rev 5.1 board.

* The white label with board serial number on the top will start with 39 (2019)
* Mid right edge of the board will have a 1.4 inch square outline for a display
* The P2 jack in mid right will be black plastic female 4-pin jack
* A thick white strip is on the right edge for notes as required.
* The large black MosFet transistor in lower right will fit the pads on the PC board.

<img src="assets/programming_your_robot/MagniIdentificationForMcbRev5p1.jpg
" />

#### Revision 5.0 Main Control Board

These items identify a rev 5.0 board besides the top copper board rev in bottom left.

* The white label with board serial number on the top will start with 38 (2018)
* Mid right edge of the board will have a 1.4 inch square outline for a display
* The P2 jack in mid right will be a 4-pin male header
* A thick white strip is on the right edge for notes as required.
* The upper left of the board will have a large capacitor on its side with white glue

<img src="assets/programming_your_robot/MagniIdentificationForMcbRev5p0.jpg
" />  

#### Revision 4.9 Main Control Board

These items identify a rev 4.9 board besides the top copper board rev in bottom left.

* It has no serial number stick-on tag like all rev 5.x boards will have on the MCB
* No white stripe of top silkscreen along the right edge and it has not P2 there either
* The top layer text on the left will be bordered by top layer full copper PC layer.

<img src="assets/programming_your_robot/MagniIdentificationForMcbRev4p9.jpg
" />  

#### Revision 4.7 Main Control Board

These items identify a rev 4.7 board besides the top copper board rev in bottom left.
THE REV 4.7 BOARD WAS A PRE-PRODUCTION BOARD FOR EVALUATION

* It has no serial number stick-on tag
* No white stripe of top silkscreen along the right edge and it has not P2 there either
* The board has not top layer of copper so large areas much darker than other areas will show through the top of the board and the text on the far left will not seem to be 'boxed' in copper.
* The 14-pin jack that holds the switch board had to be cut to not hit a large transistor.
* The large 50-pin jack in upper right will be a female jack for this pre-production board

<img src="assets/programming_your_robot/MagniIdentificationForMcbRev4p7.jpg
" />  


### The Power Switch Board Revisions

There were several versions of switch boards from pre-production through first shipment of units using the rev 4.9 MCB. The revision number only started to appear on rev 2.0 switch boards shipped at the time of the rev 5.0 MCB boards.


#### Switch Boards With Remote Switch Connectors

In order to support user needs to place the main power switch and/or the ESTOP switch in a location that is on their robot cover or perhaps is more accessible due to the customer physical additions we developed the revision 2.2 switch board seen below

<img src="assets/programming_your_robot/SwitchBoardRev2p2.jpg
" />  

The revision 2.2 board has P202 seen in the back right that is wired in series with the red keycap ESTOP switch on the board.   Our plan is we will ship the connector that mates with P202 that has one piece of wire sorted to itself.  In this way P202 is shorted from the factory and a user may remove this jack and put two wires going to his own ESTOP switch for his own robot needs.

However it is done ```P202 MUST be connected``` or no motor power will be enabled

On the back left you see P201 which is wired in parallel with the black main power switch.  The thought here is users who want a remote main power switch connect a connector to two wires and then leave the installed main power switch OFF or pushed in.  The customer switch will then be the power switch.

For either ESTOP switch or Main Power switch we ship one jumper that can be modified but sometimes for replacment boards you may want the part numbers for the cable.  Ideally a crimp tool would be used for the pins but it is possible to manually solder onto pins although that is time consuming and a little tricky to wrap the crimp metal around the wire so the pin will fit in the housing.  

    Plastic Housing:  Molex 0009501021.  Digi-Key Part Num WM18813-ND
    Female Pins:      Molex 0008701031.  Digi-Key part Num WM18820CT-ND

#### Switch Boards To Support Rev 5.x MCB Boards

* The main thing to watch for is if the board has 3 resistors it is old for rev 4.9 or earlier MCB boards.

* A REV 5.x MCB  REQUIRES A 4 RESISTOR DESIGN AND BE LABELED REV  2.x or later.

* A rev 2.x switch board with 4 resistors can be used with earlier MCB boards.

<img src="assets/programming_your_robot/MagniIdentificationOfSwitchBoards.jpg
" />  

#### Very early pre-production Switch boards

In early prototypes there were switch boards with large white switches that had green leds in them to show the state.  These should not be in production units unless some sort of replacement had to happen early in first production units.

<hr>

## Removal And Replacement Of Magni Main PC Boards

The firmware on the MCB is critical to have up to date to avoid prior bugs from showing up when a different MCB is put in the robot.  The MCB board may be shipped with old firmware so all replacements of the MCB should be followed by checking and updating MCB firmware after the replacement board is running.  Refer to [Upgrading MCB Firmware](noetic_magnisilver_mcb#firmware-upgrade) to check and upgrade if needed.

<H4 style="color:red">YOU MUST REMOVE POWER CABLES FROM THE BATTERIES FOR THESE PROCEDURES BECAUSE WE MUST BE SURE THE FULL BATTERY VOLTAGE IS NOT PRESENT ON THE BOARD WHEN REPLACED.</H4>

### Removal Of Main Control Board (MCB)

These steps are taken to remove the main board.  It should be noted that to replace a board these steps can be done in reverse order.

   - Disconnect ALL battery leads from your batteries. We always leave the thick red and black power cables connected to the MCB board because if you get a replacement it will have the power cables attached. The cables are routed in a very specific way with the mounting hardware also done in a precise way thus we do not remove these cables.
   - Disconnect both large multi-pin black wheel cables with inline jacks from the wheels.  These can be very tight so you may need a very good grip and work the connectors gently back and forth as you try to extract.  Be careful to not bang your knuckles as they can release all at once. See the ```The Motor cables to the Wheels``` section of [THIS PAGE](https://learn.ubiquityrobotics.com/unboxing) for pictures
   - Unscrew the 2 screws that hold the small 'Switch Board' to the Magni front panel and place the screws in a safe place.   After screw removal the switch board can be unplugged from the main MCB board and taken out then set aside perhaps near the 2 screws.

     ![Switch Board Screws](assets/support/SwitchBoardMountingScrews.jpg)

   - If your Magni has the sonar board you should remove it's 50 pin cable from the main MCB board and remove the sonar board to make things easier for this process.    You can see how it is installed and do the reverse that is described on the last half of [THIS PAGE](https://learn.ubiquityrobotics.com/sonar_sensors)
   - We are going to free up the RaspiCam flat white cable so the Raspberry Pi can be removed easier in next step.  Refer to [THIS PAGE](https://learn.ubiquityrobotics.com/camera_sensors) for pictures.  Locate the white flat thin ribbon cable to the camera at the point it gets to the camera.  Take note at this time that the blue tape on the flat cable is away from the RaspiCam PCB which will have to happen as you reassemble later.   NOTE: The jack for the cable is very delicate so just pull back the tabs on each side just a mm or two and do not force it harder or it may break the tabs.  Pull out the cable from the camera end.  
   - Remove the large sheetmetal rectangular Front Bracket that has the raspicam camera bracket riveted to it.  Refer to the middle of [THIS PAGE](https://learn.ubiquityrobotics.com/unboxing) and see the ```Front Bracket``` picture. The removal is done using a long allen wrench with 4mm tip for the bolts that hold this 24mm wide side to side bar to the top shelf of the Magni chassis.   The 4mm allen wrench has to be long enough to go through from the top of the bar all the way to insert into the bolts.

   ![Front Bracket](assets/support/FrontBracketWithRaspicam.jpg)

   - Unscrew one phillips head screw that may be holding your raspberry Pi to a 20mm tall standoff near the center of the MCB.   Save this and take note of the washers and 1mm thick plastic spacers on some boards and don't loose these tiny parts.
   - Now you may gently ease out the Raspberry Pi. This is a bit tricky so take your time. Care should be taken to never apply any pressure to the very thin Micro SD card in this process as it is easy to break.  You first back the Pi out of the 40 pin tall connector.  Next you have to remove in an upward direction the Raspberry Pi clear of the chassis. Place the Raspberry Pi aside in your work area.  I find that gentle rocking away and towards the MCB at the side of the 20mm standoff while pulling pins out is easiest.
   - Now it is time to unscrew the 4 M3 button head screws that hold the MCB to the chassis using a 2mm long Allen wrench that came with the robot or your own. For some of the screws you may need to run a long allen wrench through access holes in the chassis 5cm away in some cases to get the allen driver straight into the screw head.  Watch where these screws go and don't loose them.
   - Getting the MCB board out requires patience and care.  Do not force anything and do not rub any parts off from contact with the sheet metal on the way out!   It will be removed after no screws hold it by working it out upward now that the front bracket is removed.   As you look behind the MCB in the battery area free up the thick black cables and/or thick power cables then work the MCB out a bit more and eventually it will be guided out in an upward direction.  Just take your time and always watch for any parts that may get bent or scraped so try to do this carefully and take your time.
   - Once the MCB board is out we leave on the thick red and black cables and return those with the MCB if you have been requested to return the board for us to study the failure.

   This completes the mechanical replacement of the MCB

