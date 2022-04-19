---
title: "Mapping & Navigation"
permalink: noetic_quick_start_navigation
group: "quick start"
rosver: noetic
nav_order: 6
nav_exclude: false
---  

# Mapping & Navigation

If you're not using Conveyorbot or EZ-Map with one of our Lidars, we also have this tutorial on how to set up mapping yourself.

This document discusses running navigation software on a Ubiquity Robotics robot base using a RPLidar that would have to have been installed by yourself. It also assumes that you have a workstation with ROS installed connected to a network in common with the robot and configured so the robot is the ROS master. For workstation configuration please see [Setup a ROS workstation](https://learn.ubiquityrobotics.com/workstation_setup)

Use of a Lidar is a popular method of implementing robot navigation.  We are going to supply here some basic starter launch files and some directions for how to get started in robot navigation using a Lidar.

The Lidar will allow the robot to see walls all around and both map and then later navigate within a known mapped area.  We will use a RPLidar simply because they are very available and a ROS driver exists for that Lidar.  

The full system here could be studied and investigated by learning more about such things as ROS /tf topic and the lidar itself as well as how ROS understands the orientation of sensors in 3D space. Plenty to learn but this is a known starting point as an example.


## The RPLidar Placement and Connection

We will use a relatively low cost and very popular Slamtec RPLidar A1 that is connected to the Magni raspberry Pi using USB and will by default in most cases show up as serial device `/dev/ttyUSB0`.  If you have a Pi4 we recommend using one of the jacks with blue plastic as they have higher current capability.  Use a good quality USB cable to avoid loss of power in many of the cheap cables leading to odd problems.

For the launch files in this demos the RPLidar is screwed to the top plate using 10mm spacers so the ribbon cable can bend around and allow the RPLidar little USB board to be connected via USB cable to a USB port of the Raspberry Pi. Be sure to not let the little USB board short out against the metal on the Magni and of course use standoffs for that as well if desired.

The center of the lidar when looking from the top is located half way between each wheel but of course on top of the Magni top plate. This places the translated X and Y of the lidar at 0,0 relative to what is called ```base_link``` in the robot physical model, URDF model,  the rotation of the Lidar is specified so the little pulley is to the rear side of the robot.

![RpLidar Mounting](assets/doing_more/Magni_RpLidarMounting.png)

To modify this example for use of any other Lidar it would be best done after you understand this demo but even reading this demo set of instructions will offer you many things to investigate and research to go as far as you like with your own robot hardware.  Should you wish to use a different Lidar the launch files would need modifications and you would need a ROS driver for that Lidar.  

The position of the Lidar should match in X and Y as defined in the launch files by ```lidar_translation```   If you cannot place the lidar where it is shown here then you should adjust the X and Y values relative to what you see in this picture in the launch files.  I strongly suggest you not rotate the lidar unless you really must so try to have the large rotating lidar wheel towards the front as in picture.

There are 6 floating point values in the lidar_translation definition. First 3 are X,Y,Z translation in meters from our 'base_link' or a spot in space located directly between the centers of our two large main drive wheels. Z is not too critical so standoff height is not important because the lidar only generates data in the 2D X-Y plane. The next 3 values are rotational values in radians around the X,Y,Z axis.  If you must rotate the lidar about Z then note that the 6th value is the rotation about the Z axis (straight up). If you must rotate the lidar about X or Y ... you are on your own, it can be confusing.

### ROS Configuration And Prerequisites

Unless we later install these on our images at this time, late 2020, these installs may be required.  These do no harm if they are already installed.  Your robot must be able to see the web to do these updates.

    sudo apt update
    sudo apt install ros-$ROS_DISTRO-navigation
    sudo apt install ros-$ROS_DISTRO-slam-gmapping

After the above installs to be prepaired to run navigation code you will also need the driver for the SlamTec RPLidar. There is a bug in how they setup the /scan topic ROS publisher so the ROS parameter of scan_topic_name does not work at the time of this writing, Oct 2020. So I have edited the hard coded string they use as seen below before the make.

    cd ~/catkin_ws/src
    git clone https://github.com/sharp-rmf/rplidar_ros
    cd ~/catkin_ws
    vi ~/catkin_ws/src/rplidar_ros/src/node.cpp
    Edit to replace ```scan_topic_name``` with ```scan``` where ros::Publisher_scan_pub is setup
    catkin_make

### How To Get Our Demo Launch Files

You will need to update your `~/catkin_ws/src/demos`  repository in order to get the launch files.  The command below will pull the new magni_lidar folder into catkin_ws/src/demos folder by doing a pull for your existing demo folder below.  The actual location of the magni_lidar demo on github is :  https://github.com/UbiquityRobotics/demos/tree/master/magni_lidar

    cd ~/catkin_ws/src/demos
    git pull
    cd ~/catkin_ws
    catkin_make -j 1

## Making a map of your area

It is suggested on first time use of this demo to choose a fairly simple room without a lot of chairs and tables with thin legs at the height of the lidar.  It is also suggested until you understand this entire demo that you have a few things in your room so it is not just a complete square with no features.   The robot later will try to understand where it is so if you put it in a totally square room there are multiple choices and some situations may confuse the robot navigation stack.

Place the magni where you would like to be the origin of the map which will be X,Y as 0,0.  The easiest way to operate later when you use the map is to know where the front wheels are as you start your map and what direction the robot is facing so I suggest 2 pieces of masking tape to the sides of the wheels to use later.  Now you should totally restart the robot right at this origin which is done using a ```sudo shutdown -h now``` and then a full power off and power back on. The reason for this is that this is a certain way to set the MCB odom counters at 0,0 so when the magni-base service starts odometry will be reset.

### Verfiy the robot is fully ready to respond

Whenever there is a power-up situation or a reboot of the robot there will be delays  due to network before the robot is ready.  We suggest the simple command and wait for it to be active.  There can be many `no new messages`  shown but after the rate shows the robot is ready.  (Use Contro-C to stop the rostopic command)

    rostopic hz /odom
    no new messages              (These can go on for a minute or more)
    average rate: 20.181         (Once these start the robot is ready)
             min: 0.044s max: 0.052s std dev: 0.00159s window: 19

### Launch the map making application

The next command will be to launch the map maker example to start a lidar, the RPLIDAR A1, and make the system ready for gmapping is in this repository.

    roslaunch magni_lidar magni_lidar_mapmaker.launch

#### Verify that the Lidar is publishing to a ROS topic

Once the lidar is started, the /scan ROS topic will publish the lidar scan data. You can verify it is generating data (although it is a great deal of data) using this test command

    rostopic hz /scan
    average rate: 6.740
            min: 0.144s max: 0.152s std dev: 0.00370s window: 6

Use Control-C to stop this periodic display of the rate for publications of the lidar /scan topic because we just wanted to see if the Lidar is running

#### Debug help if /scan shows no data

If you did not get anything you can debug by running the lidar node all by itself till you resolve the issue which may be wrong serial port or power to the lidar or other things.   The following command will just start the lidar used in this demo.

    roslaunch magni_lidar rplidar.launch

You can use the rostopic hz command as above or see the console if there is some fault in serial port or other problem the launch will abort.

Once you feel the lidar is working exit this with a Control-C so you can go on with this demo where the lidar will be launched within the map maker launch file by running as before this command.  

    roslaunch magni_lidar magni_lidar_mapmaker.launch

Here too you could verify using the rostopic hz command before continuing.

#### Clear any prior map data

We now want to clear any old ROS global map so a fresh one is generated next. We will form that folder if it does not exist and then remove or rename the map.txt file in that folder if the folder did exist already.  There are other ways to do this but this is straightforward and easy to also remind you to save a prior map if you are running the test in a different room or room configuration.

    cd ~
    mkdir -p ~/.ros/slam
    mv ~/.ros/slam/map.txt ~/.ros/slam/map.old

### Running gmapping Once the launch file starts

On the laptop copy over lidar_mapmaker.rviz from this repository to your home directory so it can configure rviz easily. Then on the laptop you can run this from home folder.

    rosrun rviz rviz -d lidar_mapmaker.rviz

### Drive around to create a suitable map

Might be best to have a Joystick but if not you have to ssh to the robot and use 'twist' to drive around. You would then drive around the area with the optional Logitech F710 joystick or 'twist' command below but perhaps start

rosrun teleop_twist_keyboard teleop_twist_keyboard.py
As the robot drives. you will be building more date for the map and you would see it on RVIZ

Your best chance at a good map is to drive slowly.   So use the ```z``` key if using teleop_twist_keyboard so you set the speed down to around 0.2 or even less.   Never pick up and manually move the robot while making a map.  The robot must know where it's wheels are and see the Lidar as it moves around.   Sometimes I stop the robot every meter or so and let it get a clean set of lidar scans then move to another location.   Your goal is to drive around any objects so the Lidar sees all sides of all objects in the room.

Later you can do this whole map exercise in more complex rooms but keep it simple if you have never done this sort of thing before.

### BE SURE TO SAVE THE MAP!

It is very important to ```SAVE THE MAP``` because if you just stop things you loose the entire set of data you just created with some effort I might add.

    rosrun map_server map_saver -f mynewmap-ils

We suggest you copy both the .pgm and the .yaml files into magni_lidar/maps so they can be found and used easily.  You can name them with things like 'myworkshop' or 'labarea' with no spaces in the names.  Later you can put the desired map in the proper location or modify the launch files with your own map names.  Maps will be used from your  ```~/.ros/slam``` folder.

## Navigating within a pre-created map

Once a map is available you can then navigate within that map or set of rooms. This is that you have been waiting for frankly! The idea here is the launch file publishes onto ROS the previously made map and then you either drive around using only robot odometry OR you use some very advanced software called AMCL or Adaptive Monte Carlo Locationization figures out where the robot is at any given time. Both methods will be shown in this section.

A key piece of software used in this simple example is the move_basic package that will accept commands to go places and talk to the robot navigation stack to drive the robot to the destination. The move_basic package is unique to Ubiquity Robotics and is a simplified version for just point to point movements based on the more advanced move_base concepts. The ROS move_base package does path planning as well as object avoidance if the system has been setup for detection of things like a person or object getting in the way of the robot.

You then can use RViz on your laptop (described in mapping example) and can define a pose that you want the robot to move to. A 'Pose' means a specific location in terms of X and Y on the floor as well as the rotation of the robot.
Lets GO!

### Start the navigation stack using an existing map

Here we need to start the launch file and specify a map that will be used for navigation in whatever room or area you are in that has previously been mapped using gmapping and saved as a map. Edit magni_lidar_maprunner.launch to set the desired map. We supply a tinyroom.map as an example but this is just a small square area and unless you duplicate it exactly this will not work for you. It was about 1.9M x 1.5M if you have a bunch of cardboard you could start doing navigation without the making of the map part of this demo

    roslaunch magni_lidar magni_lidar_maprunner.launch  

This launch file will strictly respect the odom information the robot keeps track of to determine robot position and rotation (called robot pose). The problem with this method is all robot odom only determination of pose drifts over time the more movements that take place. So this method is ok for a short demo but not very usable in general real world situations.

### Start move_basic for setting navigation goals

So far we have setup things so the robot knows where it is within a map. We need to start some software that we can tell where we want to move to so that that piece of software can control the robot to approach a desired destination X,Y and rotation (both of these things together are called a desired pose. A pose is the X and Y location on the map as well as the rotation of the robot within the map.   We will use rviz but we must start this piece of software called move_base now. Ubiquity Robotics move_basic is a simplified version of move_base where move_base can do complex plans to get around objects or corners. The move_basic package can only do line of sight straight paths and if something gets in the way it stops and does not plan around the object.

    roslaunch magni_nav move_basic.launch

We will now be ready to accept 'goals' and then move to those goals.

The more general solution to navigation uses object avoidance and the move_base package combined with a map that holds obstacles seen by sensors such as the sonar. The objects that move in fron are in what is called the costmap. Perhaps that will be added to this example in the future.

### Run RViz on your laptop to watch and set goals

On the laptop copy over lidar_mapmaker.rviz from this repository to your home directory so it can configure rviz easily. Then on the laptop you can run this from home folder.

rosrun rviz rviz -d lidar_mapmaker.rviz

Below is an example of a very simple map shown in RViz.  The black outline surrounds gray 'known open area' and is the navigation stack's thinking on where the map ends or objects were located.    The red outline is the realtime lidar scan data.  The cones are shown here but we are not taking advantage of them so far in this demo.   The cones are data from the Magni Silver sonar board.  The colorful lines are different axis of different key parts of the robot and where they are located.   This picture does not have the simulated model of the Magni showing up because I like to see ALL the axis of the robot and you can even see the ones for the wheels rotate when the robot drives!

![RpLidar Mounting](assets/doing_more/Magni_ExampleOfLidarAndSonarsInMap.png)

### Tell the robot where it is located on the MAP

The AMCL package is now told just where the robot is within the map and which direction it is pointing. This step allows the AMCL package to have a very good initial pose for the robot so it can estimate how to get to other locations right away.

Set the 2D Pose Estimate using the RViz interface as follows:

    - Left click the ```2D Pose Estimate``` button in RViz
    - Find the place the robot is in the map and left click on that spot but HOLD MOUSE BUTTON
    - A large green arrow will appear and you have to drag mouse so the arrow points the same angle as the robot is facing.
    - Release the mouse button and then the robot will have a very good estimate of it's pose

### Define a target pose as a navigation goal

Here is the really fun part, assuming all the other things are working. This is where you tell the robot to move to places in the map. Because we are using the simple move_basic planner be sure the path is clear to the destination.

The 'pose' of a floor based robot is both it's X and Y location as well as the rotation on the floor. That is what you will define.

I will attempt to explain in words how to define a goal for the robot. Basically we want to form a 2D Goal for the robot that is at some location and indicates the direction we want the robot to be facing when the goal is reached

    - Identify a place you want the robot to drive to
    - Click on the left mouse button on that spot and HOLD MOUSE KEY DOWN because an arrow will show up
    - Move around the mouse so the arrow points in the direction the robot will face when done
    - Release the left mouse button

If the gods are with you the robot will approach that spot and rotate to the direction you specified.

## Running AMCL to correct robot location

The most common package that figures out where the robot it in the room (relative to the map made before) is called AMCL. It uses an adaptive Monte Carlo method to find the location of the robot at any given time.

Run the launch file below to use AMCL. To use AMCL in any real world situation which may have a complex map you first have to use RViz to tell the robot where it is in the map.

    roslaunch magni_lidar magni_lidar_maprunner_amcl.launch  

More things have to be all working nicely for reasonable results using AMCL so we suggest you get the odom only running first and only after that works move on to this example using AMCL.

The AMCL package figures out the map to odom transform and publishes that which in effect corrects the errors that build up on all robot self posting of odom frame. The earlier launch file had a static transform that published that the map to odom transform was 'zero' from map to odom frames. This launch file removes that static transform and lets the AMCL package publish the map to odom transform or tf.

## Where to go from here using move_base instead of move_basic

This demo uses move_basic which cannot plan to move around objects or avoid objects that show up as the robot is navigating to a new pose (location).

We are not using the more powerful move_base in this example (so far) to keep things as simple as possible. If you choose to use move_base the planing is much more advanced but of course more things can then go wrong.

We hope to enhance this discussion of navigation by showing how to use move_base  as time permits us to add to what is here now.  What is presented here now is at least a good first start for people to get their feet wet then later dig in to the more complex but far more capable move_base doing true path planning around corners and around dynamically moving objects that were not in the map when it was made.

ROS navigation stack can use detection of things that get in the way of the robot or were not present at the time of map making to then alter the plan to follows a path the robot was taking to the destination.  Sensors such as sonar sensors can be used to detect things and place them in something called a costmap. The costmap can change as things move into and out of the path of the robot like your dog or cat or even somebody walking by or the movement of some object in front of the robot.

The move_base code can dynamically re-plan the path to be taken to avoid objects in the costmap.
