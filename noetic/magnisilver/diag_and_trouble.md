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


## Keeping your battery charged

By FAR the number 1 issue we see time and again is a weak or discharged battery. We have protections to shutdown things but as the battery gets weak many other issues show up.  We are continuously making strides to better inform customers of dangerously low batteries. In versions of the product shipping in 2022 we have the battery state indicated on the recently added OLED display to greatly help make this issue very visible to users of the robot.  The battery voltage and if it is too low will show up in 2022 current systems.

If you are 100% sure your battery is delivering over 23 volts to the MCB board large power connectors as the robot runs then you can skip this section and move to the next section.

Make sure your battery is installed correctly, with all the contacts
fully attached and the batteries are fully charged.
A pair of fully charged Lead Acid batteries should give around
26-27 V - if you don’t have a voltmeter and the robot has enough charge to run then the robot can self report
battery voltage which is covered later on in this message (point 5).

A good way to make sure the batteries are fully charged is by plugging in the provided charger. If it switches off automatically then the the batteries are fully charged.

Make sure the 14 pin connector that connects the switch board to the main board has not pulled out. If it has push it in again.

Make sure that both push buttons on the front of the robot are out all the way (the red push button de-energizes the motor circuit as an emergency stop). Both blue and red LED on the small ```switch board``` PCB that has the switches should be illuminated.

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


## Motor Control Board (MCB) Issues

First [identify your MCB revision](noetic_magnisilver_mcb#board-revision-identification) and pass on that info to support once you contact them, that'll make issues much easier to pinpoint.

### Standalone Test Program
A standalone test program used in our MCB tests and development. You can get the source to this program [on github](https://github.com/UbiquityRobotics/ubiquity_motor/blob/kinetic-devel/scripts/test_motor_board.py).

### Verify that the MCB is recognized over serial
If you are unable to get the robot to move and are sure battery is ok and the ESTOP switch is not set to disable the motor power, then here is how to verify the linux host computer is talking to the MCB.

Edit file ```/home/ubuntu/.ros/log/latest/rosout.log```.   Look for a ROS error with the word ```Firmware``` in it that says ```not reporting its version```.   That message is because the motor node has been unable to communicate with the MCB over the serial interface.

#### Manually Connecting To The MCB
We have a tool that can be used to figure out if the linux host computer is able to communicate with the MCB.  We need to stop the main motor control node then use the tool to do something simple like read the firmware version.

    cd ~
    sudo systemctl stop magni-base
    python /opt/ros/$ROS_DISTRO/lib/ubiquity_motor/test_motor_board.py

This should show a help menu and at that point we know we can open the serial port so then we can read the firmware version using the version command

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

### Troubleshooting Failure of the Motor Node

If the list of ROS nodes did not show /motor_node the robot will not function.  Troubleshooting the motor node not starting is likely to require help from the Ubiquity Robotics team.

If earlier sections on this page have not resolved the startup issue we suggest you add a post to the [Ubiquity Robotics Forum](https://forum.ubiquityrobotics.com/)

It is likely you will be asked for the ROS log file or to try manually starting the robot.  Both will be described below.  There have been default images that have generated excessive logs that we call 'log spam'.  If you see a great many log lines repeating, ignore it for now as the valuable lines will be in the logs for debug.

#### Locating The ROS Log To Send In For Analysis

There is some chance that a clue may be in the most recent ROS log.   Startup the robot and let it run for 5 minutes then be sure you cannot control the robot with the twist command shown at the start of this page. Make a copy of the ROS log once you confirm the problem.  If you let the system run for hours the log can get very large so take it early on. This assumes default user 'ubuntu'   

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
above 24 V. This is also a good way to confirm that you are
getting correct communication between the raspberry pi main
computer and the master control board.

### Serial Protocol and Commands

A baud rate of 38400 is used with one stop bit for communications with between the host cpu (normally raspberry Pi) and the MCB, Motor Control Board. For details see [the Magni Serial Protocol Spec](https://github.com/UbiquityRobotics/ubiquity_motor/blob/indigo-devel/Serial_Protocol.md).

The Motor Control Board, sometimes called main control board, uses a protocol where a packet with checksum is sent and if a reply is required the reply will come back in the same binary protocol.

The MCB constantly transmits status and this goes to the 40 pin connector pin 10 which is the host (Raspberry Pi) Receive. The host sends commands on it's Transmit using 3.3V signals and this arrives at the MCB on pin 8 of the 40 pin connector.

Although the MCB sends status immediately on power up the host takes sometimes a minute or so to start all the nodes and then start sending commands to the MCB.  On revision 5.2 and later boards there are two blue leds that show both the directions of serial traffic.  Magni will not really be running till both of these are seen to be blinking very fast.

For MCB boards prior to revision 5.2 the MCB serial conversion circuits required a 3.3V power supply to appear on pin 1 of the 40 pin connector.
