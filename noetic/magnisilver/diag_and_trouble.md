---
title: "Diagnostics & Troubleshooting"
permalink: noetic_magni_silver_diagnostics_and_troubleshooting
group: "magni silver (gen 5)"
rosver: noetic
nav_order: 11
nav_exclude: false
---

# Diagnostics & Troubleshooting

This page is a collection of troubleshooting tips and pointers to things to be verified in the case that the robot does not seem to be operating properly for even simple movement commands.

See the [Tips and Tricks page](noetic_quick_start_tips_and_tricks) for software tips on debugging your robot.

For most of these operations you will need to have connected to the robot using ssh through either the robot's built in HotSpot or through a hard Ethernet connection to your workstation or laptop on the same network as the robot.

If you need help doing the connection refer to [Connecting a Workstation page](noetic_quick_start_workstation)

The most common and most basic issue that can come about in different ways is that the robot does not move.   The main focus of this page is to therefore troubleshoot common issues we have seen with the robot not moving when told to move.

This page also has some information on other forms of troubleshooting software we have written or that is common to use in ROS based robots (Robot Operating System)

Below are some other locations on in our documents that may help troubleshoot issues not discussed in enough detail on this page:

* [Magni Verification Tests](noetic_magni_silver_verification)  
* [Fiducial Follow or Localization](https://forum.ubiquityrobotics.com/t/troubleshooting-procedure-for-fiducial-localization-problems/134)   
* [Magni Does Not Move](https://forum.ubiquityrobotics.com/t/magni-does-not-move/98)  

## Motor And Wheel Encoders

We limit the motor speed by default to 1 M/sec.  It is possible to make this faster but perhaps contact us for guidance.

The standard motor/wheel unit we ship have these characteristics

    - 3 phase brushless motors with internal gearing.
    - The Wheels are spaced 0.33 meters apart.
    - The wheels have a circumference of 0.64 meters
    - The 3 phase magnetic encoders produce 43 pulses each phase per revolution
    - We use both edges of the 3 non-overlaping phases for 258 tics per rev
    - This translates to about 2.5mm as the rough linear travel per enc tick
    - This also translates to 1.4 degrees wheel rotation per encoder tick

### Verification Of Wheel Encoder Operation

Here is how to verify the wheel encoders work on the robot.  This may be of use for certain hardware failures that may be due to wheel encoder failure.

Start with the robot powered off.  Raise the robot front wheels from touching the floor perhaps using block(s) of wood or other objects.   Turn on the robot power and then turn off the motor power by pressing in the RED switch so the large RED led is off and thus motor power is off.

We are going to look at the robots readings for Wheel Odometry which are entirely determined by reading the wheel encoders and tracking where the robot should be if the wheels rotated any which way.

In an SSH window to the robot type  ```rosrun tf tf_echo odom base_link```
This produces a repeating display with robot Translation and with robot Rotations where rotations are given in 3 ways.  

We will focus on the last line that says  ```in RPY (degree)  [0.000, 0.000, 0.000]```   The third number is the rotation about the Z (up) axis that the robot calculates from wheel encoder tics as measured in degrees.  The Z axis rotation measures positive for a rotation to the left using standard robot axis definitions.

If you rotate the left wheel 1/2 of a rotation as if that wheel is moving forward then the Z rotation number will get to around 50 (degrees).  Rotate that same wheel back to where it started in reverse rotation and Z rotation goes back to about 0.

Test the right wheel in a similar way where rotating as if moving forward will generate a Z rotation of around -50 degrees.   These are approximate values.

If you have the robot back on the floor you can do a similar test starting from all zero numbers after a reboot and roll the robot forward 0.1 meters and then see the  Translation 1st number go to 0.1 Meters where X is forward.   

The 6 small gauge wires out of the motor cables are the wheel encoder and power to the wheel encoder wires in case you are curious.

## Motor Controller Board Pinouts

The pinouts for the many connectors on the main Motor Controller Board can be found [HERE](https://learn.ubiquityrobotics.com/Magni_MCB_pinout.pdf).

## Motor Controller Revisions

You can see and download the released MCB board firmware revisions [HERE](https://learn.ubiquityrobotics.com/firmware-upgrade)

A list of firmware and master controller board revisions can be found [HERE](https://github.com/UbiquityRobotics/ubiquity_motor/blob/kinetic-devel/Firmware_and_Hardware_Revisions.md).

## The MCB Serial Protocol and Commands
A baud rate of 38400 is used with one stop bit for communications with between the host cpu (normally raspberry Pi) and the MCB, Motor Control Board.

The Motor Control Board, sometimes called main control board, uses a protocol where a packet with checksum is sent and if a reply is required the reply will come back in the same binary protocol.

By default the raspberry pi default host cpu single serial port on the 40 pin connector.  The MCB constantly transmits status and this goes to the 40 pin connector pin 10 which is the host Receive.  The raspberry pi host sends commands on it's Transmit using 3.3V signals and this arrives at the MCB on pin 8 of the 40 pin connector.

Although the MCB sends status immediately on power up the host takes sometimes a minute or so to start all the nodes and then start sending commands to the MCB.  On revision 5.2 and later boards there are two blue leds that show both the directions of serial traffic.  Magni will not really be running till both of these are seen to be blinking very fast.

For MCB boards prior to revision 5.2 the MCB serial conversion circuits required a 3.3V power supply to appear on pin 1 of the 40 pin connector.

#### MCB Serial Protocol Details
For details see  [The Magni Serial Protocol Spec](https://github.com/UbiquityRobotics/ubiquity_motor/blob/indigo-devel/Serial_Protocol.md)

#### Standalone Test Program To Control The Magni_MCB
A standalone test program used in our tests and development. You can get the source to this program on github  [HERE](https://github.com/UbiquityRobotics/ubiquity_motor/blob/kinetic-devel/scripts/test_motor_board.py)

### The Battery

By FAR the number 1 issue we see time and again is a weak or discharged battery. We have protections to shutdown things but as the battery gets weak many other issues show up.  We are continuously making strides to better inform customers of dangerously low batteries. In versions of the product shipping in 2022 we have the battery state indicated on the recently added OLED display to greatly help make this issue very visible to users of the robot.  The battery voltage and if it is too low will show up in 2022 current systems.

If you are 100% sure your battery is delivering over 23volts to the MCB board large power connectors as the robot runs then you can skip this section and move to the next section.

Make sure your battery is installed correctly, with all the contacts
fully attached and the batteries are fully charged.
A pair of fully charged Lead Acid batteries should give around
26-27V - if you don’t have a voltmeter and the robot has enough charge to run then the robot can self report
battery voltage which is covered later on in this message (point 5).

A good way to make sure the batteries are fully charged is by
plugging in the provided charger. If it switches off automatically
then the the batteries are fully charged.

Make sure the 14 pin connector that connects the switch board to the main board has not pulled out and if it has push it in again.

Make sure that both push buttons on the front of the robot are out
all the way (the red push button de-energizes the motor circuit
as an emergency stop). Both blue and red LED on the small ```switch board``` PCB that has the switches should be illuminated.

### Main Power And Status LEDs Must All Be Brightly Shining

There are many important LEDs on the large MCB PC board. First verfiy the row of 5 horizontal LEDs on the master control board just above the switch board connector. All 5 leds must be brightly on with the STATUS led to the right doing very brief drop-outs in light every 4-6 seconds as a heartbeat for the on-board processor. Also verify the 3.3V LED which is higher up on the MCB just under the large white 'MAIN' power jack at the top of the MCB is brightly on always.  If you don’t see these LEDS lit brightly you very likely have a bad power supply on the MCB board. If your board pre-dates rev 5.2 there is no 3.3V led.

If you are convinced there is a problem you can make a post on our [Online Support Forum](https://forum.ubiquityrobotics.com) or send an email to support@ubiquityrobotics.com to start a discussion

If you see no leds at all check the main fuse on the robot in the middle lower part of the board - you should be able to visually see if the fuse is blown.  The main fuse is on the front near the center and low on the MCB.  The left black switch of course must be in the 'out' position for ON and the big Blue led on the switch board must be ON before you will see any MCB board LEDs on.

### Verify Serial Monitor LEDS are operating properly

When the MCB is properly running the SOUT led that is 1cm below the AUX  12V pin should start blinking very fast within 5 seconds of power on to the MCB.  If you do not see this the MCB is very likely to be in need of repair IF you do see the STATUS led doing it's drop-out blinks every 4-6 seconds.  

If the SOUT is blinking fast then verify that when the MCB and the host Raspberry Pi computer are operating and host software is fully running The SIN led high up on the MCB just near the AUX 12 pin on left white connector at the top of the LCD will be blinking fast within 5 minutes of turning on the robot.  If this is not happening it may be host software OR a failure in MCB serial hardware inbound driver chip.

When both SIN and SOUT leds are blinking it means the hardware serial driver chips and connections are all functional for communications between the Raspberry Pi and the MCB.

#### Verify If you are seeing the Rare Baud Rate Bug

If you DO SEE the SIN and SOUT leds blinking fast yet the robot is not operational for movements here is one known issue which we feel has been solved since late 2022.

For this issue the /motor_node is running but is getting garbled messages from the MCB.  We can look for huge numbers of REJECT messages.    Use this command from an ssh console to the robot.

    grep REJECT ~/.ros/log/latest/rosout.log | wc

If you see the first number as hundreds or even thousands and doing the command again gives an even larger number then this softare defect is happening.  This defect is sometimes disappears after a ```sudo shutdown -r now``` command to soft reboot the robot.

The fix we discovered it to a root user CAREFLLLY edit /boot/config.txt and find the line that is commented out with a # sign in column 1 with ```init_uart_baud``` .   You must remove the pound sign and change the value so the line reads as follows:

    init_uart_baud=38400

Save the file and then a reboot is required after this change which will fix the baud rate bug and the REJECT messages will no longer be in the log when fixed.

#### Power Supply And Status LEDs Quick Reference Table

Here is a summary of the leds on the MCB main board.  If any of these are not correct the robot is likely to have significant problems (except for the 2 Aux power leds).   All of these LEDs exist for rev 5.2 or later boards and earlier boards have less leds.  

There are also 2 large leds on the switch board where a blue led when on indicates main power is activated by the black switch and a red led indicates the motor power is activated by the red switch. The red led will always be off if the main power is off.

The locations are specified as if you are looking at the board with the two large white 4-pin power connectors at top and large 50 pin connector also at top right.

Most of these not able to turn on as described is generally a big problem.

| Purpose |	Location | Description |
| ------- | ---- | ----------- |
| MainPower | LowerLeft | ON when black main power key is activated in the out position. When this is on all 5 of the leds in a row will be active as shown below |
| MotorPower| LowerRight | ON when red & black keys are both activated in the out position |
| 12vAux | MidLowerLeft | ON for Aux 12V active. This is far left of 5 leds. On pre rev 5.0 MCB  this was lowest led. (Robot will work with no 12V Aux) |
| 12vMain | MidLowerLeft | ON for Main 12V active. This is 2nd from left of 5 leds. On pre rev 5.0 MCB this was 2nd led from bottom |
| 5vMain | MidLowerLeft | ON for Main 5V active. This is 3nd from left of 5 leds. On pre rev 5.0 MCB this was 3nd led from bottom |
| 5vAux | MidLowerLeft | ON for Aux 5V active. This is 4th from left of 5 leds. On pre rev 5.0 MCB this was 4nd led from bottom |
| 3.3V | TopMidLeft | ON for 3.3V active. This is on rev 5.2 and later MCB boards |
| STATUS | MidLowerLeft | ON with short blinks off/on every 3 to 6 sec depending on firmware rev. This is the far right of 5 leds. On pre rev 5.0 MCB this was the top led in the row of 5 |
| WIFI | MidRight | WiFi status. 2 blink/sec in startup, 1 blink/sec when operating as an Access Point, 1 blink every 2 sec when connected to another WiFi.  The complement of this led is on our optional switch board|
| SOUT |  TopMiddle | Blinks very fast when the MCB processor is running. On rev 5.2 and later |
| SIN |  TopMiddle | Blinks very fast when the host CPU is actively up and communicating with the MCB.  This can take a couple minutes to start blinking for bootup. On rev 5.2 and later |

### Verify the Raspberry Pi CPU Is operational

We have seen cases where the serial lines of the Raspberry Pi have been damaged.   First of all the Raspberry Pi when booting up will blink its tiny green LED located very near the small USB C connector opposite the LAN jack on the Pi.  If you see not blinking after power-on the green Pi Activity led or the Micro SD card is broken.  

#### Things To Inspect If RasPi Green Led Does Not Blinks

Remove the micro-SD card and verify it is not cracked.   Perhaps try a second micro SD card burned fresh from a known good image we supply.  If you have an HDMI monitor and can plug that into the Pi before power-up of the Raspberry Pi you can maybe see if the Pi is booting up or is stuck in some way.   If the HDMI monitor does NOT show the Pi boot up properly then your Pi or the SD card is in some way bad.    

#### Verification The RasPi WiFi is operational

Every so often we see Raspberry Pi units where there is no WiFi.   For our default images after a good bootup and if you have NOT configured this unit to use your own WiFi yet then you would see our WiFi show up in a scan for WiFi using your phone perhaps.   The name would be ubiquityrobotXXXX where XXXX is a unique ‘hex’ code.   If you have a fully powered up system using a known good and proven production ready codeset with no WiFi it is likely a broken WiFi on that Raspberry Pi

#### Detailed Inspection Of The RasPi Bootup sequence

This is a last resort but can in fact shed light into failures due to SD card corruption or unanticipated failure modes of the host software on the Raspberry Pi host CPU.  You would have to with a cable plug in a HDMI monitor into the Raspberry Pi or on some more modern robots inspecting the output of the 7 inch TFT display as the system boots up. For the Raspberry Pi 4 this is the tiny micro HDMI so you need a cable with one end with the tiny HDMI connection.  Either HDMI micro will work but we tend to use the HDMI0 connector which is next to the USB C connector.   If you see some fault that looks bad and things stop there is an issue with either the  Raspberry Pi or perhaps our Image has hit some unknown case or some Raspberry Pi hardware on the Pi is broken.

### Very Long Startup Times

If it takes 3 to 5 minutes for the /motor_node to show up this has been seen in the past to usually be because the CR2032 coin cell battery mounted on the back of the MCB board has never been installed or has gone dead.  Refer to our [Unboxing and Assembling page](https://learn.ubiquityrobotics.com/unboxing)
We had to stop shipping these small batteries due to assorted recent regulations and are sorry for this inconvenience.

### Verify The MCB Is Recognized Over Serial port
If you are unable to get the robot to move and are sure battery is ok and the ESTOP switch is not set to disable motor power then here is how to verify the linux host computer is talking to the MCB.

Edit file ```/home/ubuntu/.ros/log/latest/rosout.log```.   Look for a ROS error with the word ```Firmware``` in it that says ```not reporting its version```.   That message is because the motor node has been unable to communicate with the MCB over the serial interface.

#### Manually Connecting To The MCB
We have a tool that can be used to figure out if the linux host computer is able to communicate with the MCB.  We need to stop the main motor control node then use the tool to do something simple like read the firmware version.

    cd ~
    sudo systemctl stop magni-base
    python /opt/ros/$ROS_DISTRO/lib/ubiquity_motor/test_motor_board.py

This should show a help menu and at that point we know we can open the serial port so then we can read the firmware version using the version command

    v
    Fetch sofware and hardware version information
    fw revision 40
    fw daycode  20201209
    hw revision 5.1

If you see the version (whatever it may be) serial is working at least at the level of linux serial port talking to the MCB.

Restart the robot code using  ```sudo systemctl start magni-base```

#### Verify the motor_node is running

We want a key observation before we continue.  Assuming you have waited a couple minutes after starting the robot we want to SSH in to the robot so you have a console on the robot itself and run this command:

    rosnode list

The list of ROS nodes must show /motor_node or the robot will not function.
Keep track of if you see the /motor_node for the troubleshooting sections that follow.

### Troubleshooting Failure Of The Motor Node

If the list of ROS nodes did not show /motor_node the robot will not function.  Troubleshooting the motor node not starting is likely to require help from the Ubiquity Robotics team.

If earlier sections on this page have not resolved the startup issue we suggest you add a post to the [Ubiquity Robotics Forum](https://forum.ubiquityrobotics.com/)

It is likely you will be asked for the ROS log file or to try manually starting the robot.  Both will be described below.  There have been default images that have generated excessive logs that we call 'log spam'.  If you see a great many log lines repeating, ignore it for now as the valuable lines will be in the logs for debug.

#### Locating The ROS Log To Send In For Analysis

There is some chance that a clue may be in the most recent ROS log.   Startup the robot and let it run for 5 minutes then be sure you cannot control the robot with the twist command shown at the start of this page.   Make a copy of the ROS log once you confirm the problem.  If you let the system run for hours the log can get very large so take it early on. This assumes default user 'ubuntu'   

Use this line below to gather the ROS log into rosLog.txt file for analysis

    cp  /home/ubuntu/.ros/log/latest/rosout.log  /home/ubuntu/rosLog.txt


#### Manually Starting The system

Sometimes it is valuable to manually start the system and see on a console screen what was going on which perhaps can be valuable as feedback if the lines are saved to a file and returned to the factory.  In the lines below we have a console screen and we stop the system and then wait 10 seconds then manually start the system.  There will be a large amount of logs so after you determine the robot cannot be told to move with the twist command use  Control-C to stop this window and scroll back to inspect or copy off lines to a file.

    sudo systemctl stop magni-base          (give it 15 seconds is plenty to stop)
    roslaunch magni_bringup base.launch

If the motor node does die you should see assorted errors that will help debug the issue.



    rostopic list

This will list all the available topics on the robot one of
which will be /battery_state or similar then type

    rostopic echo /battery_state

it will give you a bunch of diagnostic information about the
battery state. Confirm that the battery voltage is at least
above 24V. This is also a good way to confirm that you are
getting correct communication between the raspberry pi main
computer and the master control board.



## Using Robot Commander For Simple Debug

Some debug or just general driving can be done using our ```Robot Commander``` app connected to the robot.   This has driving controls and has other commands.

Robot Commander requires a functioning Magni robot so it will not help much in debug of a robot that will not move for other reasons internal for the robot hardware or software.   

One example of a Robot Commander verbal command is you can ask the app "battery".  You should get a reasonable percentage answer.  Also, the battery command should appear in the Command Log window.

The app also allows telling the robot it is at a 'waypoint' and it keeps track of the wheel positions then later you can tell it to go to that waypoint.

#### Connecting

In the following we assume that Robot Commander is connected to the robot.  By voice, after connecting, command
"battery".  You should get a response indicating the battery charge state. Now you know that the connection works.

#### Operation

Open a window on your workstation and ssh into the robot. Type

    rostopic list

This will list all the available topics on the robot, one of
which will be /cmd_vel.  If it doesn't, the trouble must be in the robot. Then type

    rostopic echo /cmd_vel

Any new messages on the /cmd_vel topic will now be shown on the screen.

Now tap the forward arrow on the RC screen.  You should immediately see
something like the following on the workstation screen:

    linear:
      x: 0.2
      y: 0.0
      z: 0.0

If you do, the command is getting from RC to the robot, and if the robot does
not move, there is a fault in the robot. In this case, ssh in to the robot in a
second window. In the robot try teleop by typing

    rosrun teleop_twist_keyboard telelop_twist_keyboard.py

once this is running you should be able to drive the robot
forward by typing the “I” key.  Again you will see the command echoed in the
first window.  If the robot does not move it is clear that there is a fault in
the robot alone.

If you do not see the command echoed, note the command log on the RC screen.
If the command is not logged there, RC has not understood the command
and issued it.  If the command is logged and you know that the connection
works, then the RC has tried to transmit the command. The problem may be with RC, with
the connection between the phone and the robot or with the phone or with the network.

Sometimes if an Android phone is connected to a data plan then the
phone will try to direct packets from robot-commander to the
internet rather than the robot. It may help to switch off
your external data plan before trying to make robot commander work.

### Rviz Troubleshooting

Some users have reported that, when running on a virtual machine workstation, it is necessary to turn off hardware acceleration.
