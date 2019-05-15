---
layout: default
title:  "Verification"
permalink: verification
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

### idea
I'm thinking we setup a page that is sort of hidden and not linked yet
on learn.
Then we make this like we want it for this 1st pass and do a link to it
after it seems ok.
Forgive the formatting in raw text.   I have 3 sections so it is not just
some huge 'dump' of stuff


### Basic Movement Tests:
   - 1st Test is Firmware Only test:  With no RasPi connected power up
Motor controller with ESTOP not active.
     RESULT: No jump in motors and motors are in locked state strongly
resisting movement
   Next tests are full system
   - Power up unit with ESTOP switch allowing power to motors AND/OR
ESTOP powering down motors then power up motors within 5 seconds
     RESULT: Wheels PID locked wheels to a stopped state with full
resistance. (we did this in 5 sec to do so before motor node started up)
   - Edit ros log with vi `roslaunch-logs`/rosout.log  and verify last
'Firmware version' line in log is expected firmware version
   - Wait for motor node to be fully started which takes 20 seconds or
so sometimes.
     RESULT: Wheels PID locked still and no jump in movements.
   - Using twist set to speed of 0.5 meters per second do following:
(rosrun teleop_twist_keyboard teleop_twist_keyboard.py)
     - Do a rostopic echo /odom to a window. Verify Position X is 0.
Use the 'i' key to rotate wheels very nearly 1 full as you can get
       RESULT: Position x: will be very near 0.64 meters
     - Using a stopwatch and Magni on blocks so it does not drive press
and hold 'i' which tries to move at 1 meter/sec.
       RESULT: 10 full turns should take 13 seconds if the Magni wheels
are running at 1 meter per second

### ESTOP Testing:
(Assumes rev 5.0 or later board. If not exception will be
noted)
   - For any rev 5.0 board and current code the ROS topic
/motor_power_active indicates if power is on or off.
     The topic is not instantanious in response and can take a couple
seconds.  Note that ESTOP active means motor power off.
   - Press in to engage ESTOP with 0 cmd_vel OR non-zero cmd_vel. (keep
ESTOP pressed)
     RESULT: Wheels have slight resistance when ESTOP is active
   - Release ESTOP having not moved the motors and motor node is on
     RESULT: Wheels return to locked, stopped state with none or only a
tiny amount of movement noticed
   - Press ESTOP again and this time move the wheel a half revolution
(there is resistance but it moves).  Release ESTOP
     RESULT: For version 5.0 board no wheel 'lurch' happens and motors
return to locked state with tiny or no movement.
     RESULT: For version 4.9 board the wheel will snap back quickly the
half turn then lock. This cannot be avoided.
   - Run the joystick or use 'twist' to make motors actively move
(perhaps on blocks not on ground).  Press ESTOP to active state
     RESULT: Motors will no longer have power and will slow to a stop
with mild 'self breaking' resistance to movement.
   - Continue to run the joystick for a couple seconds then release the
joystick and then release ESTOP a second later to power motors.
     RESULT: Rev 5.0 board will power up the wheels and there will be
tiny or no movement as wheels return to locked stopped state
     RESULT: This test is not recommended for rev 4.9 boards as ESTOP
switch could not be read so large movements can happen.
   - Like before run the joystick with motors running then hold joystick
active and press ESTOP (wheels stop).  Release ESTOP in 3 sec
     RESULT: Even though joystick was acive all the time, on release of
ESTOP after a half second or so wheels nicely ramp to speed again.

### Deadman Timer Testing:

The robot is designed to return to zero speed if
it looses touch with constant host velocity commands
   - Startup and run the robot on blocks perhaps at a given speed. Kill
the motor node OR disconnect serial (if your system allows)
     The motor node can be stopped and starting using:  sudo systemctl
stop magni-base.service
     RESULT: The robot will return to stopped state with wheels actively
locked.
   - Start or re-connect serial to the motor node using  sudo systemctl
start  magni-base.service (takes 15 or more seconds to start)
     RESULT: Robot should be operational after the motor node starts.
For re-connect of serial it will start back up in a second or less.
