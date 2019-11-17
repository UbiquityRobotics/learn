---
layout: default
title:  "Running Magni in Simulation"
permalink: simulation
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

## what follows is a comment and not intended for publication
<! --
Although there is enough content to explain the magic command to run things I feel it says a couple brief things then dumps a huge launch file then talks more about the 'meat' of the issue.  Since we do not supply the workstation some link to how to setup your workstation for this should be supplied.   I 'think' there is a setup workstation link on learn somewhere but I do not know if it includes all the required gazibo install.

I'm going to suggest this 'presentation' if we want to show this to real customers:   (sorry but it was hard to follow the doc IMHO)
-- >
## Running Magni in Simulation

The Magni simulator utilizes the full desktop install of ROS Kinetic, plus components of the magni-robot distribution from Ubiquity robotics.
The simulator runs a virtual magni within the Gazibo simulator and allows development without a physical robot.

### Prerequisites

We suggest your workstation be a machine with a Nvidia GPU display running Ubuntu 16.04.
You can verify the required components are present by looking for required files

       $ roscd magni_gazebo

       $ ls

       CHANGELOG.rst CMakeLists.txt config launch media package.xml urdf

If the above commands fail or the list of files is not found you will need to add the components required for Magni simulation in Gazibo

To run this example you will need a launch file called   voice_cmd.launch that is available   HERE   (LINK TO GITHUB voice_cmd.launch be it on github or on learn pages as a file but do NOT clutter the presentation with huge text files is my point)

Running the Simulation

In a terminal window on the workstation run this command which will bring up 3 windows.

      $ roslaunch magni_gazebo voice_cmd.launch

In another terminal window you may run the following command to show the many ROS topics        (Again: do not clutter the doc with all the listing, it works or it fails)

      $  rostopic list

When voice_cmd.launch starts it will start 3 windows.   Below we discuss and show example windows

<put in the pictures and window screen shots Alan supplied all here now>
