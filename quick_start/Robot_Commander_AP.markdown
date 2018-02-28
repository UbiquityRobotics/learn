---
layout: default
title:  "How to control the robot using Robot Commander"
permalink: robot_commander
---
# How to drive with a Smartphone or workstation using Robot Commander in AP mode

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/) - - - &uarr;[up](ix_quick_start)

Robot Commander has two versions:  One for the Android phone, and one for a workstation. iPhone's are not currently supported.

<div class="image-wrapper">

<img src="https://ubiquityrobotics.github.io/learn/assets/Robot_Commander.png" />


<p class="image-caption">Robot Commander</p>

</div>

## To use an Android phone:

1. Turn the robot on.
2. If you haven't, install the Robot Commander app on your Android phone. Download it on your phone at <https://play.google.com/store/apps/details?id=com.jrlandau.robotcmdr>.
3. Allow at least 1 minute after turning the robot on to allow the robot to boot up. The robot will come up as its own WiFi network. So on the phone, if you go to Settings/WiFi, you should see a WiFi network that looks something like`ubiquityrobotWXYZ`, where `WXYZ` corresponds to 4 hexadecimal digits. Connect to this network.  The password is `robotseverywhere`.
4. Start the Robot Commander app.  
5. Enter the address `ubiquityrobot.local` in the space to the left of the Connect button.
Continue with #6 below.

## To use a workstation
1. Turn the robot on.
2. Unless you've done it already, install the [Google Chrome browser](https://www.google.com/chrome/browser/desktop/index.html) on your workstation.  Bring up the browser.
3. Allow at least 3 minutes after turning the robot on. On your workstation, connect to the UbiquityRobot network. The password is `robotseverywhere`.
4. Enter the address <https://10.42.0.1/speechcommands.html> in the address bar of the Chrome browser, and press the enter key.
5. If the browser gives you any warnings regarding a lack of security, disregard them.
6. Press or click the `Connect` button. You will hear "Connected" and the button will now read "Disconnect".

## Using Robot Commander
* Press any arrow to move the robot. To keep the robot moving, keep pressing or clicking.
* Tap the Microphone to use speech.
* Say, "forward", "back", or other commands. There is a list of commands in the menu in the top right corner of the screen. The forward and back and the rotation commands allow you to say how far, for example "forward 3 feet" or "back 5 meters", or "rotate right 90 degrees".
* Where there is a lot of competing speech, you can turn on the "wake word" feature, in Settings. When it is on, all commands must be prefaced by the wake word, "robot".

## Robot Commander Commands
* forward, advance, keep going, go ahead, go straight, reverse, back, backward, go back, retreat<br>
* forward/reverse etc., may be followed by meters/centimeters/feet
* turn right
* turn left
* rotate right (may add "n degrees")
* rotate left (may add "n degrees")
* turn around
* stop, halt
* faster, speed up
* slower, slow down
* (set) waypoint ____ (waypoint name)
* go to ____ (waypoint name)
* remove waypoint ____ (waypoint name)
* list waypoints
* again, repeat
* help

When you are finished, you can shut down the robot, Robot Commander, or both.

#### &larr;[back](connecting)- - - - - - - - - - [next](keyboard_teleop)&rarr;
