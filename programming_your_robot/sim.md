---
layout: default
title:  "Running Magni in Simulation"
permalink: simulation
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

## Running Magni in Simulation

The Magni simulator utilizes the full desktop install of ROS Kinetic, plus components of the magni-robot distribution from Ubiquity robotics.
The simulator runs a virtual magni within the Gazibo simulator and allows development without a physical robot.

### Prerequisites

We suggest your workstation be a machine with a Nvidia GPU display running Ubuntu 16.04.
You can verify the required components are present by looking for required files

       $ roscd magni_gazebo

       $ ls

       CHANGELOG.rst CMakeLists.txt config launch media package.xml urdf

If the above commands fail or the list of files is not found you will need to add the components required for Magni simulation in Gazebo.

### Adding the required Components

#### Install catkin

Catkin may already have been installed, but if so this will do no harm.

    ~$ sudo apt-get install ros-kinetic-catkin

#### Make Catkin Folders And Run catkin_make
    ~$ mkdir -p ~/catkin_ws/src
    ~$ cd catkin_ws
    ~/catkin_ws$  catkin_make

#### Source the Setup.bash file
    ~/catkin_ws$  source devel/setup.bash

#### Install gazebo
    ~/catkin_ws$  sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

#### Download the Gazebo ROS Packages

Make catkin_ws/src the current directory.

~/catkin_ws/src$ git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b kinetic-devel

#### Install libgazebo and Check the Dependencies
#### Looks like this has already been done!

    ~/catkin_ws$ sudo apt-get install -y libgazebo7-dev
    ~/catkin_ws$ rosdep check --from-paths . --ignore-src --rosdistro kinetic

#### Run Catkin Make
This may take rather a long time.  

    ~/catkin_ws$ catkin_make

#### Add the Required Lines to setup.bash

Ensure that the last few lines in your ~/.bashrc look like this:

    source /opt/ros/kinetic/setup.bash  
    source ~/catkin_ws/devel/setup.bash  
    export ROS_HOSTNAME=`cat /etc/hostname`.local

### Now get the Magni Gazebo Software
Make catkin_ws/src the current directory. Then from catkin_ws, check the dependencies.

    ~/catkin_ws/src$ git clone http://github.com/ubiquityrobotics/magni_robot
    ~/catkin_ws$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y


#### Run Catkin Make Again

    ~/catkin_ws$ catkin_make

This completes the installation of the prerequisites.

### Run Gazebo   
In a new terminal, launch rviz and gazebo_ros_pkgs

    $ roslaunch  magni_gazebo empty_world.launch

When empty_world.launch starts it will start 3 windows--Rviz, Gazebo, and a control pad.

[Rviz](https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/rviz.png)

[Gazebo](https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/gazebo.png)

[Control Pad](https://ubiquityrobotics.github.io/learn/programming_your_robot/assets/controlpad.png)

--------------------

<!--
# Material for documenting voice_cmd

To run this example you will need a launch file called   voice_cmd.launch that is available   HERE   (LINK TO GITHUB voice_cmd.launch be it on github or on learn pages as a file but do NOT clutter the presentation with huge text files is my point)

Running the Simulation

In a terminal window on the workstation run this command which will bring up 3 windows.

      $ roslaunch magni_gazebo voice_cmd.launch

In another terminal window you may run the following command to show the many ROS topics        (Again: do not clutter the doc with all the listing, it works or it fails)

      $  rostopic list

When voice_cmd.launch starts it will start 3 windows.   Below we discuss and show example windows

<put in the pictures and window screen shots Alan supplied all here now>
-->
