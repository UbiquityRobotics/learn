---
layout: default
title:  "Troubleshooting"
permalink: troubleshooting
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

# Troubleshooting

This page is a collection of troubleshooting tips and pointers to things to be verified in the case that the robot does not seem to be operating properly for even simple movement commands.

For most of these operations you will need to have connected to the robot using ssh through either the robot's built in HotSpot or through a hard Ethernet connection to your workstation or laptop on the same network as the robot.

If you need help doing the connection refer to [Connecting a Workstation page](https://learn.ubiquityrobotics.com/verification)

The most common and most basic issue that can come about in different ways is that the robot does not move.   The main focus of this page is to therefore troubleshoot common issues we have seen with the robot not moving when told to move.

This page also has some information on other forms of troubleshooting software we have written or that is common to use in ROS based robots (Robot Operating System)

Below are some other locations on in our documents that may help troubleshoot issues not discussed in enough detail on this page:

[The Battery](#the-battery)
[Magni Verification Tests](https://learn.ubiquityrobotics.com/verification)  
[Fiducial Follow or Localization](https://forum.ubiquityrobotics.com/t/troubleshooting-procedure-for-fiducial-localization-problems/134)   
[Magni Does Not Move](https://forum.ubiquityrobotics.com/t/magni-does-not-move/98)  
[Robot Commander](#robot-commander)  
[Rviz](#rviz-troubleshooting)


## Troubleshooting Lack Of Robot Movement

The most basic way to do a test to verify the robot can move is to be on an SSH console window on the robot and see if you can use keyboard to control the robot at all where ```Forward``` is just tapping the ```i``` key.  

    rosrun teleop_twist_keyboard telelop_twist_keyboard.py

Before we dig into some detailed troubleshooting below on lack of movement be aware you can also review some key high level reasons for lack of movement that we had in a post on our forum from some time back that you should review just in case it is one of these issue then this page will elaborate.    

Feel free to read that post if this page does not resolve your problem quickly.   
Older troubleshooting post:  [Magni Does Not Move issue on our forum](https://forum.ubiquityrobotics.com/t/magni-does-not-move/98)  


### The Battery

By FAR the number 1 issue we see time and again is a weak battery. We have protections to shutdown things but as the battery gets weak many other issues show up.  We are continuously making strides to better inform customers of dangerously low batteries.

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

### The Main Power And Status Row of LEDs

There are many LEDs to show status but perhaps the most important to check first are the row of 5 LEDs on the master control board. The MCB, master control board, is the big PCB on the robot that you can see on the front of the robot right above the switch boardPCB.   A table will show details but in short the 5 LEDs should be illuminated. On all official production boards the leds are horizontal but very early boards had the row as vertical. The 'STATUS' led (to the right but on top for old boards) should be mostly on but have very brief blink-out then on every 4-6 seconds. If you don’t see proper behavior see the table presented next.   

If you are convinced there is a problem you can make a post on our [Online Support Forum](https://forum.ubiquityrobotics.com) or send an email to support@ubiquityrobotics.com to start a discussion

If you see no leds at all check the main fuse on the robot in the middle lower part of the board - you should be able to visually see if the fuse is blown.  The main fuse is on the front near the center and low on the MCB.

#### Power Supply And Status Quick Reference Table

Here is a summary of key leds on the MCB main board.  If any of these are not correct the robot is likely to have significant problems (except for the 2 Aux power leds).   All of these LEDs exist for rev 5.2 or later boards but earlier boards have less leds.  

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
| STATUS | MidLowerLeft | ON for Aux 5V active. This is the far right of 5 leds. On pre rev 5.0 MCB this was the top led in the row of 5 |
| 3.3V | TopMidLeft | ON for 3.3V active. This is on rev 5.2 and later MCB boards |
| SOUT |  TopMiddle | Blinks very fast when the MCB processor is running. On rev 5.2 and later |
| SIN |  TopMiddle | Blinks very fast when the host CPU is actively up and communicating with the MCB.  This can take a couple minutes to start blinking for bootup. On rev 5.2 and later |


### Very Long Starup Times

If it takes 3 to 5 minutes for the /motor_node to show up this has been seen in the past to usually be because the CR2032 coin cell battery mounted on the back of the MCB board has never been installed or has gone dead.  Refer to our [Unboxing and Assembling page](https://learn.ubiquityrobotics.com/unboxing)
We had to stop shipping these small batteries due to assorted recent regulations and are sorry for this inconvenience.

### Communication Check LEDs Within The Robot

If the battery looks to be charged we can then look at communications between the Raspberry Pi host Linux computer and the Master Control Board, MCB.

#### The MCB Status Led Indicates Working On-Board Processor

If you do not see the 'STATUS' led mostly on then doing very fast drop-out blinks every 4 to 6 seconds something is wrong with the MCB processor and requires advanced troubleshooting or a replacement from the factory.

#### The Serial Communications Leds Both Blink fast

Check if you see both SIN and SOUT leds are blinking rapidly 5 minutes after bootup. If SOUT is blinking fast it means the onboard processor is outputing state to the main host processor or Raspberry Pi SBC.  

Watch for the SIN led to start to blink very fast from 30 seconds and sometimes as long as a few minutes after powerup.   

When both SIN and SOUT leds are blinking it means the hardware serial driver chips and connections are all functional for communications between the Raspberry Pi and the MCB.

#### A Note On A rare Serial Bug Seen In 2019

there is a chance you may be seeing an intermittent serial communications port problem now fixed but was very common. For this issue the /motor_node is running but is getting garbled messages from the MCB.  The symptom was that all messages read from the MCB were rejected so the log was filled with the word REJECT.   Use this command from an ssh console to the robot.

We want a key observation before we continue.   SSH in to the robot so you have a console on the robot itself and run this command:

    rosnode list

The list of ROS nodes must show /motor_node or the robot will not function.
Keep track of if you see the /motor_node for the troubleshooting sections that follow.

#### Verify If you are seeing the Rare Baud Rate Bug

If you DO SEE the SIN and SOUT leds blinking fast AND you see /motor_node when you run ```rosnode list``` and the robot is not operational for movements here is one known issue which we feel has been solved since early 2020.

For this issue the /motor_node is running but is getting garbled messages from the MCB.  We can look for huge numbers of REJECT messages.    Use this command from an ssh console to the robot.

    grep REJECT ~/.ros/log/latest/rosout.log | wc

If you see the first number as hundreds or even thousands and doing the command again gives an even larger number then this softare defect is happening.  This defect is generally fixed by doing a ```sudo shutdown -r now``` command to soft reboot the robot.

To fix this it is best to perform a full ```Linux Host Software Update``` as described on our [Updating Software and Firmware page](https://learn.ubiquityrobotics.com/verification)  

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
