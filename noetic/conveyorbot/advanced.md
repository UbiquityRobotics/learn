---
title: "Advanced Documentation"
permalink: noetic_conveyorbot_advanced
group: conveyorbot
rosver: noetic
nav_order: 5
nav_exclude: false
---

# Advanced Docs

Intended for developers.

## Visualize with RViz

Rviz (ROS visualization) is a 3D visualizer for displaying sensor data and state information from ROS. It can show the robot in space as well as graph practically any robot parameter.

There is a nice short introductory rviz video at [this location](http://wiki.ros.org/rviz).

To run, type

  ```roslaunch breadcrumb_demos demo_world.launch```

![rviz](assets/breadcrumb/rviz_image.png)

Using rviz, you can move the robot:
Click on "2D Nav Goal" in the menu bar.
Click again in the black screen area and indicate what the robot is to do.

Here is a [short video](https://ubiquityrobotics.github.io/breadcrumb_learn/ConveyorBot/assets/rviz_with_nav.mp4) showing the robot navigating with our setup.

In the right hand window you see the steps that are carried out on the workstation including loading rviz etc.

Once RViz is loaded you can see that the user issues a go to goal target in RViz using a few mouse clicks. She first clicks **2D Nav Goal** in the menu bar, then clicks on the desired pose. The arrow that is on the field of view is the desired location and orientation.

<hr>

## Running in Gazebo

Before launching in Gazebo, you need to generate the markers that you will place in the simulation world.
Our fiducials generators provide you a simple UI to do this.

### Fiducials Model Generator
Arbitrary ranges of Fiducial marker Gazebo models can be generated with `/scripts/fiducial_generator/fiducial_gen.py`

This script populates `/models` directory.
Before launching the Gazebo simulation, add to `~/.bashrc` or enter to terminal:

    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/ConveyorBot/breadcrumb_gazebo/models

Launch files enable you to run the Gazebo, RViz and add our ConveyorBot robot in it. Additionally a simple controller opens up with which you can drive the ConveyorBot around.

### Launch files

There are several demo worlds you can use to get familiar with the ConveyorBot functionality. They can be found in `worlds` directory.
To run them use:

    roslaunch breadcrumb_gazebo fiducial_world.launch world_name:=<WORLD-NAME>.world

If `world_name` is left empty launch files runs by default a world with robot and a markers route.

add to `~/.bashrc` or enter to terminal before launching (modify the path so it will point to `/models` directory in breadcrumb_gazebo):

    export GAZEBO_MODEL_PATH=/catkin_ws/src/ConveyorBot/breadcrumb_gazebo/models

Make sure the directory is populated with fiducial models before hand. If not, go to `scripts/fiducial_generator` directory and run:

    python fiducial_gen.py

This scripts will generate everything necessary to spawn a fiducial into Gazebo simulation.


### Fiducial follow

In order for the robot to start moving according to the markers, run:

    roslaunch ground_fiducials sim_ground_fiducials.launch touchscreen:=false mode:=general_mode

Where you can determine mode and whether you want to use touchscreen.

If touchscreen is used, consider reading the section below.

#### Touchscreen UI

Run [`breadcrumb_touchscreen`](https://github.com/UbiquityRobotics/ConveyorBot/tree/indigo-devel/breadcrumb_touchscreen)
on your local workstation. If you want to make sure it will not launch Chromium in kiosk mode then open Chromium before running `breadcrumb_touchscreen`.

```
roslaunch breadcrumb_touchscreen touchscreen_rosbridge.launch
```

<br>


## ConveyorBot Navigation Stack

`breadcrumb_nav` is the main ROS package that consists of:
- **Planning of the goals** (Phantom planner)
- **Execution of navigation goals** (Move Smooth)
- **Collision avoidance** based on LiDAR and/or Sonars

### Node Details 

#### Phantom planner

Phantom planner node handles planning of navigation goals, sending them forward to `move_smooth`. 
We are using `phantom_planner` node to plan two different types of goal: **Fiducial goal** and **Phantom goal**. 
- **Fiducial goal** is in the origin of the Fiducial marker coordinate frame, whose state can be transformed using tf into any other tf frame (e.g. base_link)
- **Phantom goal** is just like any goal, only that its purpose is to serve as a next goal(second goal in the Action Server queue), which enables
the robot to make a smooth transition from following the first goal to the phantom goal. e.g. In case there wouldn't be any next goal robot would stop at first.

Correspondingly there are two ROS Services:
- `send_fiducial` handles sending/preempting Fiducial goal
- `send_phantom` handles sending/preempting Phantom goal

In practical terms, the two goals are equal, our idea was to distinguish the two since the both serve for a different purpose.
In case of ConveyorBot, the purpose of phantom goal is for the robot to blindly drive in the direction in
which marker points (planning in Fiducial frame), until it detects a new marker. When it detects a new marker it preempts the
phantom goal and removes it from the Queue on the Action Server and sends a pair of Fiducial goal and a new Phantom goal 
according to this new detected marker.

#### Move Smooth

Move smooth is a navigation node that receives a maximum of two goals at a time in an arbitrary frame and executes navigation maneuvers to arrive to the goals.
If more than two goals are send, the QueuedActionServer(Custom ROS Action Server running in parallel to move_smooth) preempts the first goal send, starts executing the next(second) goal in the queue and adds the new goal to the queue. 

#### Collision Avoidance

If data from a laser scanner or sonars is available, collision avoidance can be performed. If an obstacle is detected, it will slow or stop in an attempt to avoid a collision.

## Dynamically Reconfigure Navigation parameters

While ConveyorBot is running, you can modify its velocity or change the stopping distance of the robot in order to prevent collision etc.
Before proceeding you should make sure ConveyorBot is connected to the network (https://learn.ubiquityrobotics.com/connect_network) and your workstation synced with ConveyorBot (https://learn.ubiquityrobotics.com/workstation_setup). 
In order to do so, open a terminal on your workstation and run:

    rosrun rqt_reconfigure rqt_reconfigure
    
This will open up a UI where on the left side you choose `move_smooth`. 
Then you can choose to modify multiple parameters:

* **`max_angular_velocity`** (double, default: 1.0, min: 0, max: 4.0)

	Maximum turning velocity on spot.

* **`min_angular_velocity`** (double, default: 0.2, min: 0, max: 4.0)

  Minimum turning velocity on spot.
  
* **`max_angular_acceleration`** (double, default: 0.3, min: 0, max: 4.0)

	Maximum turning acceleration during on spot rotation.

* **`max_linear_velocity`** (double, default: 0.5, min: 0, max: 1.1)

	Maximum linear velocity.

* **`max_linear_acceleration`** (double, default: 0.5, min: 0, max: 1.1)

	Maximum linear acceleration.

* **`smooth_decceleration`** (double, default: 0.17, min: 0, max: 1.1)

	Decceleration to stop.

* **`max_linear_in_turn_velocity`** (double, default: 0.4, min: 0, max: 1.1)

	Maximum linear velocity during the turn.

* **`max_lateral_acceleration`** (double, default: 0.8, min: 0, max: 4.0)

	Maximum lateral acceleration during lateral correction.

* **`linear_tolerance`** (double, default: 0.1, min: 0, max: 1.0)

	Within linear tolerance, linear error is negligible.

* **`angular_tolerance`** (double, default: 0.1, min: 0, max: 1.0)

	Within angular tolerance, orientation error is negligible.
  
* **`max_incline_without_slipping`** (double, default: 0.3, min: 0, max: 5.0)

	Maximum incline of the robot turned sideways before starting to slip (Constant determined based of physical properties of the robot).
 
* **`max_lateral_deviation`** (double, default: 0.2, min: 0, max: 5.0)

	Maximum lateral deviation from the planned path.
  
* **`angle_boundary`** (double, default: 15.0, min: 0, max: 50.0)

	Angle at which we transition from turning to small lateral corrections.
  
* **`min_side_dist`** (double, default: 0.3, min: 0, max: 5.0)

	Minimum distance to maintain at each side.
  
* **`forward_obstacle_threshold`** (double, default: 0.5, min: 0, max: 3.0)

	Minimum distance to maintain in front.
