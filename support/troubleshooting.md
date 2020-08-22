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

Before we dig into some detailed troubleshooting on lack of movement it may be good to review some key high level reasons for lack of movement that we had in a post on our forum from some time back that you should review just in case it is one of these issue then this page will elaborate.   
Take a look at the  [Magni Does Not Move issue on our forum](https://forum.ubiquityrobotics.com/t/magni-does-not-move/98)  

If that post does not resolve things here are more detailed proceedures.


### The Battery

Make sure your battery is installed correctly, with all the contacts
fully attached and the batteries are fully charged.
A pair of fully charged Lead Acid batteries should give around
26-27V - if you don’t have a voltmeter and the robot has enough charge to run then the robot can self report
battery voltage which is covered later on in this message (point 5).

A good way to make sure the batteries are fully charged is by
plugging in the provided charger. If it switches off automatically
then the the batteries are fully charged.

Make sure that both push buttons on the front of the robot are out
all the way (the red push button de-energizes the motor circuit
as an emergency stop). Both blue and red LED on the small ```switch board``` PCB that has the switches should be illuminated.

Check the row of 5 LEDs on the master control board. The MCB, master control board, is the big PCB on the robot that you can see on the front of the robot right above the switch boardPCB.   All 5 LEDs should be illuminated. The 'STATUS' led, the rightmost one, should be mostly on but have very brief blink-out then on every 4-6 seconds. If you don’t see this behavior then  email support@ubiquityrobotics.com. (On very old units the status led is the top led in a vertical row of leds)

If you see no leds at all check the main fuse on the robot - you should be able to visually see if the fuse is blown.  The main fuse is on the front near the center and low on the MCB.

### Very Long Starup Times

If it takes 3 to 5 minutes for the /motor_node to show up this has been seen in the past to usually be because the CR2032 coin cell battery mounted on the back of the MCB board has never been installed or has gone dead.  Refer to our [Unboxing and Assembling page](https://learn.ubiquityrobotics.com/unboxing)
We had to stop shipping these small batteries due to assorted recent regulations and are sorry for this inconvenience.

### Communication Issues Within The Robot

If the battery looks to be charged we can then look at communications between the Raspberry Pi host Linux computer and the Master Control Board, MCB.

If you do not see the 'STATUS' led mostly on then doing very fast drop-out blinks every 4 to 6 seconds something is wrong with the MCB processor and requires advanced troubleshooting or a replacement from the factory.

We want a key observation before we continue.   SSH in to the robot so you have a console on the robot itself and run this command:

    rosnode list

The list of ROS nodes must show /motor_node or the robot will not function.
Keep track of if you see the /motor_node for the troubleshooting sections that follow.

#### Verify If you are seeing the Baud Rate Bug

If you DO SEE /motor_node and the robot is not operational for movements check this section.

If you have a version 5.2 or later MCB board as seen in white silkscreen in lower left of the board we want to look for activity on our debug leds.  If using an earlier board keep reading.

Check if you see both SIN and SOUT leds blinking rapidly there is a chance you may be seeing an intermittent serial communications port problem now fixed but was very common. For this issue the /motor_node is running but is getting garbled messages from the MCB.  The symptom was that all messages read from the MCB were rejected so the log was filled with the word REJECT.   Use this command from an ssh console to the robot.

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
