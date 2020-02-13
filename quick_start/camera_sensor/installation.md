---
layout: default
title:  "Camera and Sensor Installation"
permalink: camera_sensors
author: Alan Federman
---
#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

# Camera and Sensor Installation

There are two versions of Magni. The Silver version
comes with a sonar short range obstacle avoidance system, and the Gold
version (not yet available) adds an IR long range obstacle avoidance
package. Both versions come with a Raspberry Pi Camera. The camera is
needed for fiducial follow, partybot and waypoint navigation, and other apps you may write.

## Sonar Board Cable And Camera Assembled
The picture below shows the Raspberry Pi camera in the **forward** orientation
and the cable to the Sonar board included with Magni Silver. This is a very
popular configuration for Magni.

![Magni Sonar To Mcb With Camera](MagniSonarToMcbWithCamera.jpg)

## Camera Installation

Camera Installation
The Raspberry Pi camera can be mounted and configured to work in different
orientations. From the start of the Magni robot there have been 2 popular
orientations. The **forward** camera mounting points forward but also tilts
up about 20 degrees. This enables Magni to view things in front
such as fiducials in the Fiducial Follow application. When the robot is to be
navigating in a space with fiducial patterns on the ceiling the **upward** mounting is used.

![Magni Camera Forward And Upward Mounting](MagniCamera_ForwardAndUpwardMounting.jpg)

POWER MUST BE COMPLETELY OFF FOR THE CAMERA INSTALLATION

Notice that in both orientations that the cable was routed through the slots in
the metal bracket. The camera is screwed to fixed standoffs using M2 screws
3 or 4mm in length. The white side of the flat cable is towards the top of the
board and below there is a blue piece of tape on the cable. The cable must be
inserted as shown in order to properly connect the camera. Make sure the cable goes all the way in as it can seem to be in but not fully making contact.

The Magni software must be configured to set usage of any camera mounting other than the **forward** configuration. If
you for example use the **upward** facing camera you must edit the
`/etc/ubiquity/robot.yaml` file as root user and change the line for the raspicam
orientation.

`sudo nano /etc/ubiquity/robot.yaml`

The line would be as shown below and a reboot of the robot
is required. There must be a space between the colon and the left bracket.

`raspicam: {'position':'upward'}`

Take care when removing the pi to gently rock it back and forth after
unscrewing it&#39;s screw that goes into a standoff on the main board. BE CAREFUL
TO AVOID application of ANY PRESSURE to the very thin Micro SD card inserted
in the right side of the Pi because it sticks out. It is very easy to break the SD
card in this process if your fingers push on the SD card.

Next attach the cable to the Pi; the ‘blue’ part of the cable faces toward the USB
ports.  Make sure the cable goes all the way in as it can seem to be inserted but not fully making contact.

![Raspberry Pi with Camera Cable](a2.jpg)

Next reinstall the Pi, making sure the pins are aligned correctly as in the picture
below. (Misaligned pins will cause permanent failure!)

![Magni Sonar Board](MagniRaspberryPiMounting.jpg)

## Testing the camera.
If you find that fiducial follow or waypoint
navigation launch files have errors early on for the camera, you may want quick way to test the camera. You can first open an ssh session to the robot and then try the following command:

`raspistill -o test.jpg`

If you don’t get an error message, you have a good camera. An (mmal) error
message indicates the camera is not being detected by the Raspberry Pi, this is
usually due to a poor cable connection or less likely a bad camera.

## Sonar Board Attachment To The Robot

Below is a picture of the Sonar board included with Magni Silver configuration. This section will show how the Sonar Board is mounted on tall standoffs and a 50-pin ribbon cable is then attached. The sonar board is included in the large box for a Magni Silver but is not attached to the robot prior to shipment.  Be careful to avoid the need to often re-bend the sonars if they bump something because the pins can only be bent and re-bent a limited number of times.

![Magni Sonar Board](MagniSonarBoard.jpg)

The Magni software must be configured to enable usage of the sonar board. You must edit the
`robot.yaml` file as root user then modify the file.

`sudo nano /etc/ubiquity/robot.yaml`

Make an edit so the only uncommented line with `sonars:` in it is as shown below.  There must be a space after the colon or it will not work.

`sonars: 'pi_sonar_v1'`

The Sonar board is mounted to the chassis using 4 standoffs that screw into 4
fixed 3mm standoffs on the chassis. This description will show the 2 standoffs
on the right but two other standoffs on the left also at the 45 degree angle are
used as well. The standoffs and the M3 screws are shown below for reference.

![Magni Standoff Reference](MagniStandoffReference.jpg)

The standoffs screw into the fixed nuts on the chassis. Here we show the 2 on
the right.

![Magni Sonar Board Standoffs](MagniSonarBoardStandoffs.jpg)

Below is shown the right side of the sonar board fully mounted using the M3
standoffs and M3 screws from the top.

![Magni Sonar Board Mounted On Standoffs](MagniSonarBoardMountedOnStandoffs.jpg)

The ribbon cable is then inserted into the main Magni board as shown in the
picture below where it will be plugged into the Sonar board as the final step.

![Magni Sonar Board Cable From Main Pc Ready](MagniSonarBoardCableFromMainPcReady.jpg)

Be sure the 50 pin ribbon cable is fully inserted into both the main Magni board as well as the Sonar board.  It can be held up due to being tight or having bent pins.   Notice how far they should go into the jacks, about 5mm or so, as shown from the side in the picture below and also shown as first picture on this page.

![Magni Sonar Board Cable From Main Pc Ready](Sonar50pinCableInserted.jpg)

LED1 is for the wifi but if you do not see that start to blink within around 15 seconds after a fresh power up case there may be something not connected on the 50 pin cable.


Lastly attach the cover plate with 6 M6 screws using an M4 Allen wrench. You are
done, hooray!
