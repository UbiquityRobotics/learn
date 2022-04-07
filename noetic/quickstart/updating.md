---
title: "Updating Software and Firmware"
permalink: noetic_overview_updating
group: "quick start"
rosver: noetic
nav_order: 11
nav_exclude: false
---
 
# Updating Software and Firmware

From time to time Ubiquity Robotics will be updating either the full Raspberry Pi image, the working software on the Raspberry Pi. This page also contains a link to the page for firmware on the Motor Controller Board. This page will show how to do one of the actions below.

* Obtain and burn a fresh Raspberry Pi image onto a micro SD card
* Perform a Linux host software upgrade to use all released Software
* Clone one or more GitHub repositories to get very latest beta software
* Do a firmware upgrade which only updates MCB board firmware

The last 3 of the above list will require the robot to have access to the internet which means you will likely have to connect the robot to your network using WiFi or an Ethernet cable.

### Raspberry Pi Image Update

To check or update the full Raspberry Pi image you will have to obtain the image and then burn it to a micro S card using some laptop or workstation you have onhand.  Visit the [Ubiquity Robotics Download](https://downloads.ubiquityrobotics.com/pi.html) page.    ```Updating a full image will lead to loss of any customization you have done.```  Never destroy your old card till you know you have everything on the new image.

#### Raspberry Pi 4 Supported on MCB Version 5.2 or later

In order to use the more powerful Raspberry Pi 4 on the robot you will needs to have an MCB (Main Control Board) of version 5.2 or later.  The version for rev 5.2 and beyond is in bright white large silkscreen on the left edge of the MCB board.  Earlier versions of the MCB came out before the Pi4 introduction so we have seen 3.3V power related issues that often lead to things like the network not working and in some cases failure to boot up the Pi4.

The Raspberry Pi 4 generally requires some sort of fan or at least installing a minimum of a not very tall heat sink on top of the silver metal topped  CPU located near the center top of the Pi 4 board.  We do not have specific detials yet.

#### Known Stock Image Issues Of Note

Here is a list of some key image dates and issues or limitations of the raspberry Pi images that are known to effect customers. We will just show the datecode part of the image name.

|  DateInName | Issue Or Limitation |
|-------------------------|----------------------|
|  2020-11-07 |  Does not run Sonar board when CPU is a Pi4 |
|  2020-02-10 |  Ethernet and Sonars do not work when CPU is a Pi4 |
|  2019-06-19 |  Supports Pi3 but came before Pi4 support |
|  2020-02-10 |  First image meant to add Pi4 support. Cannot run our Sonar board without manual fixes seen on our Sonar board install page |
|  2020-11-07 |  Adds more fixes for Pi4 usage. Being evaluated. |


### Linux Host Software Update

One step down from the full image burn is the upgrading of the software on the Linux host computer.  This will pull in packages and software that we have made available from the upgrade because they have been tested and are releasable.

Use  apt-get as follows to update any new Linux software for Magni:

On your workstation, start a terminal window (Linux shortcut: ctrl-alt-t). In that window, log in by typing:

    ssh ubuntu@ubiquityrobot.local     (use the robot’s name or IP address)
    sudo systemctl stop magni-base
    sudo apt-get update  
    sudo apt-get upgrade

This may take some time, since it may have been a while since the original image was made.  When it completes reboot the robot using  ```sudo shutdown -r now```

### Clone A Github Repository for latest Software

We have new software being qualified and then placed into our github repositories.
Often we must tell people to clone and make a particular repository so that they can get the very latest code.   

Some changes in order to use latest software may require 1 or more other repositories to also be used.  Contact us on our forum at https://forum.ubiquityrobotics.com/ if you are unsure.

Direct use of our repositories is a sort of Beta usage of the software and as such you may hit issues we have not found yet.  In that case we encourage you to post an entry on our forum which was just listed in prior paragraph. 

Consider direct use of our repositories as Beta software which has shown promise.   All of the Ubiquity Robotics repositories will show up in your folder of  ```/home/ubuntu/catkin_ws/src``` folder where this assumes you are using the image as the default user of  ubuntu.

#### Updating An Already Existing repository

You may have a local copy of the repository already or you my need to set it up the first time such as after a new image is installed.  For example let us say you have already setup our ubiquity_motor repository.  Here is how to get newer code.

    cd ~/catkin_ws/src/ubiquity_motor
    sudo systemctl stop magni-base         (this is done for faster make)
    git pull
    cd ~/catkin_ws
    catkin_make

After this completes with no errors in the git pull part, do a reboot using  ```sudo shutdown -r now```

If you have errors with the git pull due to modified existing files in this repository you will need to do a bit of cleanup prior to the git pull.  Save what you have in whatever way you wish and once you are certain you can loose all your changes in this repository then before the git pull use the  ```git stash```  (We are not going to explain a great many other possible git actions you could take, that is not our purpose right now)

#### Installing A fresh copy of a repository

This can be done if you do not have the repository yet or if you fully remove the repository after saving it somewhere OUTSIDE of the catkin_ws workspace.

    cd ~/catkin_ws/src
    sudo systemctl stop magni-base
    git clone https://github.com/UbiquityRobotics/ubiquity_motor
    cd ~/catkin_ws
    catkin_make

After this completes with no errors do a reboot using  ```sudo shutdown -r now```


### MCB Firmware Upgrade

Sometimes you may need to get a firmware upgrade for your Motor Controller board.
This operation is less frequently needed and in general we advise customers to do the firmware upgrade only if they run into an issue we know we have addressed in an update.

To get the firmware upgrade follow
the process on [the firmware-upgrade page](https://learn.ubiquityrobotics.com/firmware-upgrade).

# Upgrading Motor Controller Firmware

Before you can upgrade firmware, your robot needs to be connected to the
internet or you must have the firmware file and copy it to the robot.  See [Connecting the Robot to Your Network](/connect_network).

We upgrade firmware using a tool that will by default install the latest released firmware. If an older fimware version is needed, it will require manual version entry seen in the Version column of the table below.

Improvements to out motor controller fimware (v35 and later) need the latest Linux host side software. Installing the latest Linux host side software can be done by following a process we call 'Linux Host Software Update' seen on [this page](https://learn.ubiquityrobotics.com/updating).

## Finding The Firmware Version Info For Your Robot

Here is how to get firmware version and date once you have opened a SSH window into your robot.  This works by filtering off just the 2 items of interest which are ```Firmware Version```  and  ```Firmware Date```

    rostopic echo /diagnostics | grep -A 1  'Firmware [DV]'

## Firmware Release Versions

The table below shows the default and latest available versions of firmware.  

The ```Rate``` in the table is the rate at which the MCB ```STATUS``` led will blink which is handy to visually check. It is best to count the time of 4 or more blinks then divide by that number for more accuracy. If the blink rate is found to be other than in the table it is possible you have a beta or non-approved version.

DateCode was started around version v35 and shows up in /diagnostics topic along with the version.  DateCode is in YYYYMMDD format for releases and YYMMDD for beta releases. It is the date of that particular version.  If you have a version that the date is before the date in the table it is likely a Beta or unofficial release. DateCode did not start till around v35.

| Ver |	Rate | DateCode | Description |
| ------- | ---- | ---- | ----------- |
| v28 | 6.0 | NA | Depreciated Production shipment version that does wheel movement check on startup. Users should do a firmware upgrade from this very old version |
| v32	| 5.0 | NA | Depreciated Production firmware used in 2019.  |
| v35 |	4.0 | 20190815 | Last well known good release as of Sept 2020 with double resolution wheel encoders and many improvements.    Requires host side software update done after 11/10/2019 |
| v37 | 4.75 | 20200620 | Use in manufacturing only.  1st with a selftest. |
| v38 | 5.25 | 20201006 | Depreciated Beta release for some fixes. |
| v39 | 5.5 | 20201129 | Non-Magni release for in development 4wheel drive unit |
| v40 | 5.75 | 20201209 | Magni current release for most recent release candidate.  This is the default if just an enter is done when asking for version |
| v43 | 4.0 | 20210829 | Magni beta release with some fixes and minor new abilities |

To see more details about our firmware as well as our hardware revisions for the motor controller please view [this page](https://github.com/UbiquityRobotics/ubiquity_motor/blob/kinetic-devel/Firmware_and_Hardware_Revisions.md).

## Checking ROS /diagnostics Topic For Firmware

The firmware version as well as the firmware release date are published by the motor_node to ROS topic  ```/diagnostics```

Use the following command for a couple seconds then use Control-C to stop the fast moving output.  Scroll back and look for ```"Firmware Version"``` and ```"Firmware Date"```

    rostopic echo /diagnostics


## Firmware Installation
To install any firmware first log into the Raspberry Pi:

    ssh ubuntu@YOURROBOT.local

all of the following commands are to be run on the Pi.  

### First Prevent The motor_node From Talking to the MCB

There are now two ways to prevent the ROS motor_node from constant queries and commands to the MCB board over the serial connection.  
For a standard Magni configuration it is best to completely stop the Ubiquity Software and then do the firmware upgrade.
As of our motor_node available as of April 2021 you can also do a new method that does not shutdown everything but only impacts the motor node by a disable or relinquish of the serial port to allow direct serial control which in this case would be for the firmware upgrade.   We suggest for a standard Magni use the full system stop.

#### Full Stop Of the Magni Software Is The Best Method

The following command fully stops standard Magni software but may not fully stop other configurations for some new Magni applications to be announced in 2021

    sudo systemctl stop magni-base

Once you have completed a firmware upgrade you are best off to reboot when using this method.   

    sudo shutdown -r now

#### A soft disable of motor_node control of the MCB Is Now Possible
As of about April 2021 users who have very current ubiquity_motor repository code are able to do a softer stop of the motor node and then later re-enable the motor node.   We suggest use of the prior method but show this to be complete.

    rostopic pub /system_control std_msgs/String "motor_control disable"

After the firmware upgrade you may still just reboot OR use this command

    rostopic pub /system_control std_msgs/String "motor_control enable"

### After The MCB is Free over serial do the upgrade_firmware

You can use the firmware upgrade utility that must have web access to fetch the most current released firmware OR you can load a version you have fetched yourself and placed in a file.

#### Upgrading Firmware From The Web If You Have A connection.
To use this method you would generally connect the robot to your own WiFi or use an ethernet cable into the Pi to your own network.  

Run the firmware upgrade utility:

    rosrun ubiquity_motor upgrade_firmware.py

After entering your email address, you should receive an email with an access token.
Enter this token and either accept the default version which is seen in the above table or specify a different version perhaps for latest features or perhaps an older trusted release (eg. specify version 35 by typing in 'v35'). The download will start and upgrade the motor controller board.

Should any problems show up you are always allowed to do this firmware upgrade again to select a version you know was best for your needs.

The process should take less than a minute; make sure that power is
not interrupted during this process. Otherwise your robot may become
inoperable and require factory servicing.

When done, reboot the robot with `sudo shutdown -r now` if you used

You are now on the latest version of the firmware.

#### Firmware Installation From A File

In some support situations you may be working with the development team here and be given a beta version of software in the form of an encrypted file.

Upgrade Firmware from a file that you place on your system:

    rosrun ubiquity_motor upgrade_firmware.py --file  v40_20201209_enc.cyacd

The name above is an example file for released v40 firmware. For beta firmware if you are told to use a given version then you may not be able to request a beta version so you would have to check our please visit  [OUR REPOSITORY](https://github.com/UbiquityRobotics/ubiquity_motor)  and navigate into the firmware folder.

As of mid 2021 we have decided on a naming convention for our firmware which will start the name with   vXX_YYMMDD  where XX is the firmware rev and YYMMDD is they daycode.  We use a daycode so that we can have 'beta' or early release tests.  When we do have a beta firmware we will try to end the name in  beta and may say other text.  So the vXX_YYMMDD is the only fixed format specified but other characters may be after that format.   

#### Firmware Installation Using Non Standard Serial support

In some situations the serial port used for controlling the robot may not be the default port on the Raspberry Pi host computer attached to the MCB.

To specify a different serial port such as one plugged into usb:

    rosrun ubiquity_motor upgrade_firmware.py --device /dev/ttyUSB0
