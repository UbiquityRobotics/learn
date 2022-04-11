---
title: "Sensors"
permalink: noetic_quick_start_sensors
group: "magni silver (gen 5)"
rosver: noetic
nav_order: 4
nav_exclude: false
--- 

# Sensors

Magni can accomidate most ROS compatible sensors, however some are already supported out-of-the-box which you can find documented here.

## Pi Camera Module V2

<H4 style="color:red">When changing out sensors makes sure to disconnect the RED battery cable from the batteries and wait 2 minutes so high voltage and current are not linked with the main board!</H4>

### Camera Installation

The Raspberry Pi camera can be mounted and configured to work in different
orientations. From the start of the Magni robot there have been 2 popular
orientations. The **forward** camera mounting points forward but also tilts
up about 20 degrees. This enables Magni to view things in front
such as fiducials in the Fiducial Follow application. When the robot is to be
navigating in a space with fiducial patterns on the ceiling the **upward** mounting is used.

Other postions include **downwards** which corresponds to the tower+shell setup and has the camera pointing down to see floor markers, and **ahead** which points straight forward.

![Magni Camera Forward And Upward Mounting](assets/camera_sensor/MagniCamera_ForwardAndUpwardMounting.jpg)

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

![Raspberry Pi with Camera Cable](assets/camera_sensor/a2.jpg)

Next reinstall the Pi, making sure the pins are aligned correctly as in the picture
below. (Misaligned pins will cause permanent failure!)

![Magni Sonar Board](assets/camera_sensor/MagniRaspberryPiMounting.jpg)

### Testing the camera.
If you find that fiducial follow or waypoint
navigation launch files have errors early on for the camera, you may want quick way to test the camera. You can first open an ssh session to the robot and then try the following command:

`raspistill -o test.jpg`

If you don’t get an error message, you have a good camera. An (mmal) error
message indicates the camera is not being detected by the Raspberry Pi, this is
usually due to a poor cable connection or less likely a bad camera.

If you have a laptop running ROS where the robot is the ROS master then you may have or want to install  ```image_view``` on the laptop to see the camera output fairly easily.  Briefly if image_view is on your laptop and configured use this:

    rosrun image_view image_view image:=/raspicam_node/image

If you know how to use rviz on the laptop that too can be configured to show you the camera image but that is a bit more complex.



#### Camera Calibration
Sometimes for best performance or certainly if a different type of camera than the robot has from the factory you may want or need to do a camera calibration.   The calibration is used to ```flatten the field```.   All camera lenses introduce optical distortion some people refer to as ```fish eye``` distortion.  The shorter the focal length the greater the field of view but along with that comes greater fish eye distortion.    

To perform this calibration please see our 'Calibration' section in the readme in our  [raspicam github repository](https://github.com/UbiquityRobotics/raspicam_node) where we explain that our camera must be running the raspicam node and publishing raw camera data.   Then on a laptop setup as a ROS node where the robot is the ROS master a camera_callibrator program is run.  

The link above will point to a ROS calibration process that is used for the calibration itself once our raspicam node is publishing images.  One thing that can surprise people is after it has been indicated the program has enough data to do the calibration you will click the 'Calibrate' button.   What happens then can take a great many minutes to complete and it appears that the camera_callibrator program is hung up but in fact it has to do a great many calculations so give it time.

The ROS page with calibration is the [Monocular Calibration Page](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)

### Using A Different Camera
The robot ships with a Raspicam v2.1 which has a fixed lens suitable for many of the uses for the robot such as fiducial applications and viewing where the robot is heading.  

Using a different camera is very infolved so this section has been written to help guide those who need a better or different camera than the Raspicam v2.1

This section will explain what sort of steps must take place to use a different camera that is compatible with the Raspberry Pi standard flat cable interface, the CSI-2 interface.   A short list of the things that will be required is presented then some details that will focus on the 2020 new camera called the RaspicamHQ using a 6mm lens.  Any different camera will require hardware and software as well as config file changes.

Decide on a camera name and stick with this name as it will appear in files and as parts of file names to help you keep the many references less confusing.  We will in this example use ```camerahq6mm_2048x1520```

We consider a given camera with a given lens as one camera so if you use something like a RaspicamHQ with a 6mm lens, that is one camera config.

    1) An adapter to physically mount the camera and connect flat cable
    2) A camera calibration file created for this camera and lens
    3) Possible changes to the URDF file for location of the camera
    4) Changes to any launch files to start apps using this camera

#### Physically Mounting The Camera To The robot
In the simple case a 3D printed part that attaches to the existing Magni robot camera standoffs can be used.   We have done this with at least 2 cameras incuding the RaspicamHQ camera.  Below is shown the 3D printed adapter by itself and then the RaspicamHQ mounted to the adapter.

![RaspicamHQ Mounted](assets/camera_sensor/RaspicamHQ_MountedOnMagni.jpg)

#### Camera Calibration File Must Be created
You will need to create a camera calibration file per instructions above in 'Camera Calibration' section.   We suggest you give it a name that indicates the camera type, the lens being used and the camera resolution.  For our example here lets say we call this file ```camerahq6mm_2048x1520.yaml```.  

For this example we will set  'camera_name' in this file to be ```camerahq6mm_2048x1520``` and you will need to remember this for later in other files to be modified.

The camera calibration files are normally placed in ```/home/ubuntu/.ros/camera_info```  so you can put it there along with other cal files.

In some cases you may need this config file also in another default location so put it in both places and modify it both places later as required to avoid having to know the exact reasons each location may be used.  The other folder is ```/opt/ros/kinetic/share/raspicam_node/camera_info```

#### Possible Changes To They URDF file
The URDF file is a file that defines where physically each part of the robot is relative to the main 'origin' of the robot which is called ```base_link```.    If you are not requiring precise navigation and can live with errors due to camera placement that are only a cm more or so we suggest you not modify the URDF file as this can be an advanced task.  

The default path to the Magni URDF file, which you should backup for sure if you are going to try to change it, is this path  ```/opt/ros/kinetic/share/magni_description/urdf/magni.urdf.xacro```

Seek assistance and plan on taking some time to change this but if you do then note that all the things that are likely to change are in one or more sub-sections of that file with the word 'raspicam_mount' and the values to be changed will be the center of the camera as 'origin xyz'  and then 'rpy' which is a 3D space rotational notation too complex for this brief discussion.

#### Changes To Launch Files That Will Need The camera
This is the most involved of all the changes because it spans many launch files.   We suggest you use names in these files that are like the calibration file so you can keep track with minimal confusion.  For that reason I will use names that contain  ```camerahq6mm_2048x1520``` in the very brief coverage of files and pieces of those files that are likely to be used for the camera to use fiducials for navigation.

I will be mentioning file paths assuming you have not cloned Ubiquity Robotics folders that are for fiducial navigation be because if you do that the files to be created or modified will be in your catkin_ws space.

- Create a new launch file to specify the camera, resolution and frame rate where for example we can call it ```camerahq6mm_2048x1520_10fps.launch``` and this file would by default be located in ```/opt/ros/kinetic/share/raspicam_node/launch``` folder.  You may want to copy over an existing file such as camerav2_1280x960_10fps.launch and give the copy this new name then modify it.   

Be aware that near the top the parameter ```camera_name``` must be the name that is used inside the config file which in this example was going to be

- A very similar file in the same folder is sometimes needed so create in ```/opt/ros/kinetic/share/raspicam_node/launch``` this similar file where you can copy over the existing camerav2_1280x960.launch to form ```camerahq6mm_2048x1520.launch``` then edit this file to customize it for your new camera.

- For fiducial recognition we use a package called aruco.  You will need to perhaps save a copy of aruco.launch to aruco_old.launch and edit the file ```/opt/ros/kinetic/share/magni_nav/launch/aruco.launch``` so that instead of references to original camera has new camera.  You must modify the line near the top that has default camerav2_1280x960_10fps.launch in it and use your new launch file which for this example we called ```camerahq6mm_2048x1520_10fps.launch```


<hr>

## Sonars

<H4 style="color:red">When changing out sensors makes sure to disconnect the RED battery cable from the batteries and wait 2 minutes so high voltage and current are not linked with the main board!</H4>

The picture below shows the Raspberry Pi camera in the **forward** orientation and the cable to the Sonar board included with Magni Silver. This is a very popular configuration for Magni.

![Magni Sonar To Mcb With Camera](assets/camera_sensor/MagniSonarToMcbWithCamera.jpg)

### Attach and Enable The Sonar Board

Below is a picture of the Sonar board included with Magni Silver configuration. This section will show how the Sonar Board is mounted on tall standoffs and a 50-pin ribbon cable is then attached. The sonar board is included in the large box for a Magni Silver but is not attached to the robot prior to shipment.  Be careful to avoid the need to often re-bend the sonars if they bump something because the pins can only be bent and re-bent a limited number of times.

![Magni Sonar Board](assets/camera_sensor/MagniSonarBoard.jpg)


### Enable Sonar Board To Run in robot.yaml file

The Magni software must be configured to enable usage of the sonar board. You must edit the
`robot.yaml` file as root user then modify the file.   

`sudo nano /etc/ubiquity/robot.yaml`

#### For Images Running Kinetic ROS Code (Prior to 2022)
Make an edit so the only uncommented line with `sonars:` in it is as shown below.
Be sure to edit the line with  `sonars: none` so the line starts with a pound sign and is thus a comment.    

Remove the ```#``` from the line that will enable the sonar software and note there must be a space after the colon or it will not work.

`sonars:  'pi_sonar_v1'`

#### For Images Running Noetic ROS Code (As of 2022)

It is now more readable so find the sonars_installed line in robot.yaml and use true

    sonars_installed: True # to enable set to True, to disable set to False

### Verification Of Sonars Operating

Once the sonars are enabled to run in the robot.yaml file you can reboot the robot and verify the sonars are running with a simple command.

In a SSH console session to the robot after the robot has fully started (can take a minute) type ```rostopic hz /sonars```
If the Sonars are working then you will see a periodic messaage repeat with a rate that the sonar messages are being received.

You can also use  ```rostopic echo /sonars``` which if the sonars are running will scroll a great deal of messages so hit Control-C to stop the messages and when you scroll back the window you should see over and over all 5 sonars showing their range.

The most common reasons for this not to work are as follows:
* Sometimes we see the customer has not fully inserted the 50 pin ribbon cable going from the MCB to the sonar board.  Be sure to hutdown and power off the robot and disconnect the fat red battery before trying to check for this condition.
* The line in the robot.yaml file is not quite right with the most common issue being the lack of a space after the `sonars:`  as there must be a space before the rest of the line.
* We have had a standoff put below the middle of the board due to a manufacturing error.  In a section just below this is discussed so check for this case.
* For Raspberry Pi 4 boards we have found and fixed a problem in the GPIO drivers that prevents sonars from working.  See the next section below for details.

#### Raspberry Pi 4 Issues With Sonar board

We have found serious issues with using a Raspberry Pi 4 in combination of the Sonar board.  These come from fundamental architectural changes in how the GPIO lines are implemented on the Pi4.  

If your unit has a Pi4 then this fix is required for Images we supply up through 2020. This is a very high priority issue.  Certain workarounds had been found in mid 2020 but they seem to no longer work.   If your raspberry Pi is a Pi 3 you do not need to do this nor should you do this process!

Here is what must be done to make the Sonar board work with a Raspberry Pi 4 CPU.  
Obtain and burn a fresh image using our 2020-11-07 image located on our [Download Page](assets/camera_sensor/https://downloads.ubiquityrobotics.com/pi.html)

To do this fix you will need to have the Magni connected to your own Wifi so that the robot can gain access to the internet and apply these fixes.  You would use an SSH session to the robot and to assist in that setup see this page:
[Connecting the Robot to Your Network](assets/camera_sensor/https://learn.ubiquityrobotics.com/connect_network)

We will fix this in a new image as soon as possible as a high priority but until then this is the process.

From an SSH session to the robot that has internet access use these commands

    sudo systemctl stop magni-base
    cd catkin_ws/src
    git clone https://github.com/UbiquityRobotics/pi_sonar
    cd pi_sonar
    git checkout daemon_pigpio
    cd ~/catkin_ws
    catkin_make               (This will take a couple minutes)
    sudo systemctl enable pigpiod
    sudo shutdown -r now      (This will reboot the robot)

You may now setup the robot to enable the sonar board as part of our normal sonar board install that continues below



### The Sonar Board Is Mounted To The Robot With standoffs

The Sonar board is mounted to the chassis using just 4 standoffs.  Only the two standoffs to the sides that will have large pads with holes on the sonar board for 3mm standoffs are to be used. This description will show the 2 standoffs
on the right but two other standoffs on the left also at the 45 degree angle are
used as well.

<H4 style="color:red">If you get your robot with a standoff screwed into the front top bar of the robot in the middle location REMOVE THAT STANDOFF because it will cause a SHORT CIRCUIT to parts in the middle of the Sonar board. The middle standoff is used ONLY for a board that sits below the Sonar board</H4>


The standoffs and the M3 screws are shown below for reference.

![Magni Standoff Reference](assets/camera_sensor/MagniStandoffReference.jpg)

The 4 standoffs screw into the fixed nuts on the chassis. The picture below shows the 2 standoff locations on right and the 2 on the left.  Center standoff is NOT used.

There is also no need to load the large fuse on the back of the board for just the Sonar board and we recommend removing F701 if it is loaded for when you only have the Sonar board (Up to 2020 that was always the case).  The fuse allows 24V to get routed to other boards that sometimes can sit below the sonar board but it is best safety to just remove that fuse.

![Magni Sonar Board Standoffs](assets/camera_sensor/MagniAllSonarBoardStandoffs.jpg)

Below is shown the right side of the sonar board fully mounted using the M3
standoffs and M3 screws from the top.

![Magni Sonar Board Mounted On Standoffs](assets/camera_sensor/MagniSonarBoardMountedOnStandoffs.jpg)

The ribbon cable is then inserted into the main Magni board as shown in the
picture below where it will be plugged into the Sonar board as the final step.

![Magni Sonar Board Cable From Main Pc Ready](assets/camera_sensor/MagniSonarBoardCableFromMainPcReady.jpg)

Be sure the 50 pin ribbon cable is fully inserted into both the main Magni board as well as the Sonar board.  It can be held up due to being tight or having bent pins.   Notice how far they should go into the jacks, about 5mm or so, as shown from the side in the picture below and also shown as first picture on this page.

![Magni Sonar Board Cable From Main Pc Ready](assets/camera_sensor/Sonar50pinCableInserted.jpg)

LED1 is for the wifi but if you do not see that start to blink within around 15 seconds after a fresh power up case there may be something not connected on the 50 pin cable.

Lastly attach the cover plate with 6 M6 screws using an M4 Allen wrench. You are
done, hooray!

<hr>

## LiDAR


<hr>

## IMU

Coming soon.