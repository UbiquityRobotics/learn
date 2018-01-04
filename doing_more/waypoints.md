---
layout: default
title:  "How to control the robot using Robot Commander"
permalink: waypoints
---
# Use Robot Commander over the network to go to waypoints

#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/) - - - &uarr;[up](ix_doing_more) - - -

Robot Commander has two versions:  One for the Android phone, and one for a workstation.

<div class="image-wrapper">

<img src="https://ubiquityrobotics.github.io/learn/assets/Robot_Commander.png" />


<p class="image-caption">Robot Commander</p>

</div>

## To use an Android phone:

1. Turn the robot on.
2. If you haven't, install the Robot Commander app on your Android phone. Download it on your phone at "https://play.google.com/store/apps/details?id=com.jrlandau.robotcmdr".
3. On the phone, in Settings/WiFi, connect to the network (that is, the SSID) that you connected to in the section `Connecting the Robot to Your Network`, above.
4. Start the Robot Commander app.  
Continue with #6 below.

## To use a workstation
1. Turn the robot on.
2. Bring up the Chrome browser on your workstation.
3. On the workstation, connect to the network (that is, the SSID) that you connected to in the section `Connecting the Robot to Your Network`, above.
4. Enter the address "https://ubiquityrobot.local/speechcommands.html" in the address bar of the Chrome browser, and press the enter key.
5. If the browser gives you any warnings regarding a lack of security, disregard them. The browser will load Robot Commander.
6. Enter the address `ubiquityrobot.local` in the space to the left of the Connect button. Press or click the button. You will hear "Connected" and the button will now read "Disconnect".

## Set a waypoint and go there
* You can move the robot in the same way as you did in the Robot Commander section above.  To set waypoints you must have set up fiducial markers as in the section on fiducials above.
* Tap the Microphone to use speech.
* Say `waypoint alpha`. You can use any name you like for a waypoint, but the phonetic alphabet is convenient as it is, by design, easily understood.
* Robot Commander will ask whether it has understood the waypoint name. Assuming that it has,
* Check the waypoint with the voice command `list waypoints`.
* Now you can move the robot to some other location in the area.
* The voice command `go to alpha` will now cause the robot to go to the waypoint you established, and to face in the same direction it was when you set the waypoint.
* You can remove a waypoint with the voice command `remove waypoint alpha`.

<<[back](fiducials)- - - - - - - - - - [next](sensors)>>
