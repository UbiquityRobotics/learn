---
layout: default
title:  "Verification"
permalink: verification
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

## Basic Tests To Verify Main MCB Board Operation

This page tells how to verify basic operation of the Magni robot.  These tests can be used to regression test hardware and firmware changes or new production boards installed in the Magni robot.
If you have Magni Silver then the robot will have a few blue LEDs on top that will be helpful but not required to do these tests.

### Power Supply And Status LED Indications
There is a line of 4 power supply indicator LEDs and a 'STAT' or status led that are in a row to the lower left of the board.  The 4 power supply leds should all be on and the status LED default state is to be on almost all the time but have very brief 'dropout'blink that form a blink every 4 to 6 seconds.  See the [firmware_upgrade page](https://learn.ubiquityrobotics.com/firmware-upgrade) for  the expected blink rates of released firmware.

Revision 5.0 main control boards and after have the leds in a horizontal line with the 'STAT' led to the right.  On version 4.9 and other early production units the LEDs are vertical with 'STAT' at the top of the line of leds.

If the 'STAT' led is off or does not blink there is something wrong with the onboard microprocessor subsystem on the main board.

Starting with version 5.2 of the main control board, MCB, there is an onboard 3.3V power supply and a blue led up near the top and a bit below the large white 'MAIN' 4-pin power jack.  This led has the label 3.3V and must be on.

### The /diagnostics ROS topic
When the robot is running quite a few pieces of diagnostic information can be viewed by looking at the ROS topic the motor node publishes.  If you run the following command for a few seconds they use Control-C to stop it you can browse back and see many things helpful for diagnostic work.

    rostopic echo /diagnostics       

After you have run this for a couple seconds and stopped it with control-C here are a few items that are valuable to know about for diagnostics

   - `Battery Voltage` key shows the battery level in DC volts
   - `Motor Power` key is True when the ESTOP is enabling wheel power
   - `Firmware Version` shows the main board firmware version
   - `Firmware Date` shows the date for the firmware
   - `Firmware Options` shows hardware options if enabled

### The Electronic Circuit Breaker, ECB', LEDS
Starting with main board version 5.0 there are two LEDs that indicate the ECB circuit involved is active and passing power.  The switch board switches must be able to control the two ECB circuits as discussed below.

On the lower left of the main board a blue LED will be ON if the `Main ECB` has been enabled due to the main power switch being set to the out position. The main switch is black and is to the left on the little switch board.

On the lower right of the main board a blue LED will be ON if the `ESTOP` switch is set to the out position AND the main power switch is also in the out position.

### Serial Communications LEDS
Starting with version 5.2 of the main control board, MCB, there are two leds that show if serial communications are active between the host processor (Raspberry Pi) and the MCB.   Both of these leds are located in the middle of the MCB and very high up near the white power jacks at the top of the PC board.

The lower blue LED is the SOUT led and will blink rapidly when the MCB processor is active even if there is no host processor.  This LED shows the signal seen by the host processor so the level shifter must work for this to be seen.

The upper blue LED is the SIN led and will blink when the host processor is actively sending commands and queries to the MCB.  This is the most important led to be blinking.   It indicates of the ROS node called /motor_node is actively controlling the MCB.


### Wifi HotSpot Verification

If no LAN cable is attached and if the robot has not been configured to look for a WiFi OR if no WiFi can be seen then the Magni software will create a HotSpot that you can connect to with your laptop.

If you have Magni Silver with the Sonar board then as the robot is powered up the LED2 (right LED on Sonar board as seen from the front) will light with dim blue light.
After 6 or so seconds that LED will turn off. After about 16 or so seconds if WiFi is able to come up LED2 will start to blink brightly about once per second indicating that the WiFi HotSpot is up.  We are working on enhancements to be available by mid 2020 which will indicate AP mode active or that the wifi specified by pifi utility is not available and perhaps more states on this led.

Starting with main control board version 5.2 there will also be a wifi led on the right visible from the front of the magni.  This led is opposite from the sonar board led so it is off when the sonar board led is on and so on.

At this time if you have on your smartphone some sort of WiFi network scanner you will see a ubiquityrobotics WiFi with last 4 digits being a unique hex value.

You will also see this HotSpot show up on your laptop and will be able to connect.  Read [HERE](https://learn.ubiquityrobotics.com/connecting) for more.

### I2C Bus Devices
The I2C bus on the host CPU needs to be able to communicate to a few devices on the MCB.  There is an I2C excpander at addr 0x20 and RealTime clock chip at address 0x6F. If there is a OLED display loaded on P2 it is at 0x3c. We should stop the motor node then run i2cdetect which is part of i2c-tools package.

    sudo systemctl stop magni-base.service
    sudo i2cdetect -y 1

The above command will output 8 lines each with 16 possible hex addresses. We want to note that it detected 20 as well as 6f.  If an OLED display is loaded you may also see 3c.   After this test you may restart magni-base service

    sudo systemctl start magni-base.service


### Basic Movement Tests:
   - The first Test is a Firmware Only test: With no WiFi connected, have the red 'ESTOP' switch in the 'out' position and the black main power switch pushed in so the Magni is totally off. Then push the black main power switch which will turn on main power. At this point the main power switch will be in the 'out' position.

     RESULT: No jump in motors and motors are in the locked state strongly resisting movement.

<!--
   - The next tests are full system. Power up unit with ESTOP switch allowing power to motors AND/OR
ESTOP powering down motors then power up motors within 5 seconds.

 RESULT: Wheels PID locked wheels to a stopped state with full
resistance. (we did this in 5 sec to do so before motor node started up) -->
   - Edit ROS log with  
   ``vi `roslaunch-logs`/rosout.log``  

   and verify that the last
'Firmware version' line in the log is the expected firmware version.
   - Wait for motor node to be fully started which takes 20 seconds or
so sometimes.

     RESULT: Wheels PID locked still and no jump in movements.

### Distance and Low Speed Movement Tests:

Enter keyboard movement using:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py  

  Press the  'z' key about 12 times until the 'speed' value shows about 0.15 meters per second;
    the 'turn' value will show about 0.3

  At this point the robot will not move because when teleop is first entered it is in same state as if the 'k' was hit.

We need a second window open that we will call the 'tf' window; in that window type:

Also as setup have a second window open and in that type:

    rosrun tf tf_echo odom base_link  

This command will continually update the robot position. There will be one line that shows the translation and 3 values that are for X,Y,Z in meters.  The line will look like this if the robot was powered up in it's current position:

    Translation: [0.000, 0.000, 0.100]   at first where X and Y are 0.000.

Now we will do a few tests so make sure the robot has room to move forward about 1 meter and could have room to rotate fully. Because these tests are not precisely timed the distances and rotations will be only near the expected vaues.

   - In the teleop window press  the **i** letter key at a quick rate for 4 seconds.   This should move the Magni about 0.6 meters forward which i about one spin of each wheel since one rotation is very near 0.64 meters.

   - Look at the 'Translation' line in the second tf window and the first of the 3 numbers is X and should be near 0.6 meters perhaps within 0.1 meter.

   - In the teleop window press the **,** (comma) key at a quick rate for 4 seconds. This should move the Magni backwards about 0.6 meters to where it started.

   - Look at the ‘Translation’ line in the second tf window and the first of the 3 numbers is X and should have returned to near 0.0.

   - Next press the **j** letter key at a quick rate for about 5 seconds so the Magni rotates a little more than 90 degrees to face left. The tf window will have the 3rd line that says 'degrees' where at this rotation the 3rd number should be near or just above 90 degrees to the left. If it goes too far you can use quick taps to the l key to inch it back to about 90.

   - Press the **l** letter key at a quick rate for 5 seconds and the Magni will rotate clockwise back to the starting point and will have the 3rd line that says 'degrees' now show the 3rd number to be near 0.

### RaspiCam Camera Test:

   There is a very simple way to test the RaspiCam camera on the robot.   This test will generate a jpeg still picture in about 6 seconds just to check the camera functionality.

   ``raspistill -o testpicture.jpg``

   To verify the camera is operating properly the testpicture.jpg file needs to be moved to your laptop or other computer that has a jpeg picture viewer.  If it is too difficult to move the picture using ftp or some other linux operation, the next best thing is to look at the file size. This can be done in the line below and the reply shows the 1543213 as the size in bytes for the jpeg image file.

   ``ls -l testpicture.jpg``

   ``-rw-rw-r-- 1 ubuntu ubuntu 1543213 Aug  4 08:18 testpicture.jpg``

### Sonar Board Test:

If you have installed and enabled the sonar board using the install guide viewed [HERE](https://https://learn.ubiquityrobotics.com/camera_sensors) then you can verify sonar operation in realtime once the robot has been started.


[The sonar node](https://github.com/UbiquityRobotics/ubiquity_sonar) publishes a `sensor_msgs/Range message` for each sonar reading.  Using `rostopic echo /sonars` you can view all the sensor readings in one topic where the frame_id of `sonar_3` would be for the front facing sonar 3.  Using the table that will follow you can place boxes in front of sensors to gain confidence that each sensor is showing the distance to that object.  A thin bar may not be seen properly and you may get mixed messages for what is behind it or may see the bar so use large objects for this test.

There is one separate topic for each sensor as seen in the table that follows.  


| | |
|---|---|
|Topic|                Direction|
|/pi_sonar/sonar_0|   Far right|
|/pi_sonar/sonar_1|   45 degrees to the left|
|/pi_sonar/sonar_2|   45 degrees to the right
|/pi_sonar/sonar_3|   Front|
|/pi_sonar/sonar_4|   Far left|

Rviz can visualize these messages as cones.  There are launch files to do this in:  
https://github.com/UbiquityRobotics/magni_robot (the source package, not the binary packages)

The [move_basic node](http://wiki.ros.org/move_basic) uses the messages published by the sonar node to determine proximity to obstacles.

### ESTOP Testing:
(Assumes rev 5.0 or later board. If not exception will be
noted)
   - For any rev 5.0 board and current code the ROS topic
/motor_power_active indicates if power is on or off.
     The topic is not instantaneous in response and can take a couple
seconds.  Note that ESTOP active means motor power off.
   - Press in to engage ESTOP with 0 cmd_vel OR non-zero cmd_vel. (keep
ESTOP pressed)

     RESULT: Wheels have slight resistance when ESTOP is active
   - Release ESTOP having not moved the motors and motor node is on

     RESULT: Wheels return to locked, stopped state with none or only a
tiny amount of movement noticed
   - Press ESTOP again and this time move the wheel a half revolution
(there is resistance but it moves).  Release ESTOP

     RESULT: For rev 5.0 board no wheel 'lurch' happens and motors
return to locked state with tiny or no movement.
     RESULT: For rev 4.9 board the wheel will snap back quickly the
half turn then lock. This cannot be avoided.

   - Run the joystick or use 'twist' to make motors actively move
(perhaps on blocks not on ground).  Press ESTOP to active state

     RESULT: Motors will no longer have power and will slow to a stop
with mild 'self braking' resistance to movement.
   - Continue to run the joystick for a couple seconds then release the
joystick and then release ESTOP a second later to power motors.

     RESULT: Rev 5.0 board will power up the wheels and there will be
tiny or no movement as wheels return to locked stopped state

     RESULT: This test is not recommended for rev 4.9 boards as ESTOP
switch could not be read so large movements can happen.
   - As before run the joystick with motors running then hold joystick
active and press ESTOP (wheels stop).  Release ESTOP in 3 sec

     RESULT: Even though joystick was active all the time, on release of
ESTOP after a half second or so wheels nicely ramp to speed again.

### Max Speed Limit Test:

Here we look to verify the max speed limit value will cause the robot to not exceed the default 1 meter per second setting.  We will again use teleop_twist_keyboard so just keep it active OR start it like this if not running yet

``rosrun teleop_twist_keyboard teleop_twist_keyboard.py``  

Place the robot on 'blocks' for the front wheel so the drive wheels do not touch the floor. Normally we put a block of wood or a small stack of books under the front of the robot and it raises it up so the wheels do not touch the floor. Put a piece of tape on the outside of a wheel so while testing we can count revolutions to get the actual speed.

YOU MUST HAVE THE ROBOT DRIVE WHEELS ELEVATED TO NOT TOUCH THE GROUND FOR THIS TEST!

In the teleop window press the **k** key once to be in "stop" mode then press the **z** key several times until the "speed" value shows a value just under 1.0 meters per second.  If you go too far the **,** (comma) key backs the speed value down.

In the teleop window press the **i** key repeatedly at a fast rate (3 or 4 times a second) and the wheels spin.

Verify the speed is going at 1 meter per second by watching the wheels turn 10 times in about 14 seconds.  The wheels have a circumference of just near 0.64 meters.  This is not a scientific test, it is looking for things being far off of the expected speed.

### Deadman Timer Testing:

The robot is designed to return to zero speed if
it loses touch with constant host velocity commands.
   - Startup and run the robot on blocks at a constant speed. Kill
the motor node OR disconnect serial (if your system allows).
     The motor node can be stopped and starting using:

     `sudo systemctl stop magni-base.service`

RESULT: The robot will return to stopped state with wheels actively locked.
   - Start or re-connect serial to the motor node using

   `sudo systemctl start  magni-base.service`

RESULT: Robot should be operational after the motor node starts (takes 15 or more seconds to start).

For re-connect of serial it will start back up in a second or less.
