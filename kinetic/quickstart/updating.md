---
title: "Updating Software"
permalink: kinetic_overview_updating
group: "quick start"
rosver: kinetic
nav_order: 9
nav_exclude: false
---
 
# Updating Software 

From time to time Ubiquity Robotics will be updating either the full Raspberry Pi image, the working software on the Raspberry Pi. This page also contains a link to the page for firmware on the Motor Controller Board. This page will show how to do one of the actions below.

* Obtain and burn a fresh Raspberry Pi image onto a micro SD card
* Perform a Linux host software upgrade to use all released Software
* Clone one or more GitHub repositories to get very latest beta software
* Do a firmware upgrade which only updates MCB board firmware

The last 3 of the above list will require the robot to have access to the internet which means you will likely have to connect the robot to your network using WiFi or an Ethernet cable.

### Raspberry Pi Image Update

To check or update the full Raspberry Pi image you will have to obtain the image and then burn it to a micro S card using some laptop or workstation you have onhand.  Visit the [Ubiquity Robotics Download](kinetic_pi_image_downloads) page.    ```Updating a full image will lead to loss of any customization you have done.```  Never destroy your old card till you know you have everything on the new image.

#### Raspberry Pi 4 Supported on MCB Version 5.2 or later

In order to use the more powerful Raspberry Pi 4 on the robot you will needs to have an MCB (Main Control Board) of version 5.2 or later.  The version for rev 5.2 and beyond is in bright white large silkscreen on the left edge of the MCB board.  Earlier versions of the MCB came out before the Pi4 introduction so we have seen 3.3V power related issues that often lead to things like the network not working and in some cases failure to boot up the Pi4.

The Raspberry Pi 4 generally requires some sort of fan or at least installing a minimum of a not very tall heat sink on top of the silver metal topped  CPU located near the center top of the Pi 4 board.  We do not have specific detials yet.

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