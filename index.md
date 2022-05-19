---
rosver: noetic
nav_exclude: true
---

# Ubiquity Robotics Documentation

<H3 style="color:red">Warning</H3>

The Magni robot is strong, fast, and heavy. Initially, use lumber, bricks, or whatever you have to lift the wheels free of the floor, or run it somewhere where it can't hurt anyone or anything if it surprises you. NOT ON A TABLE TOP.

<H4 style="color:red">Always remove the red battery cable for any work on boards to remove live voltage from the main board</H4>

<hr>

Sources that you might find helpful:

#### Discourse Support Forum - [forum.ubiquityrobotics.com](https://forum.ubiquityrobotics.com)

#### Store - [ubiquityrobotics.com](https://ubiquityrobotics.com)

#### Archival Learn Pages - [learn_legacy_archive](https://ubiquityrobotics.github.io/learn_legacy_archive)

<hr>

## Preface: Ubiquity Robotics' Robot Development Platform

**The goal of this guide is to get web, mobile, and maker developers using ROS (the Robot Operating System) via Magni, the Ubiquity Robotics development platform.** We hope you will see the amazing possibilities and opportunities, dive in and never look back.

### What is Magni and What Does it Do?

> ![Magni/Loki](assets/Magni_best.png)

Magni is a hardware platform in the form of a mobile base. When powered by ROS software, it can handle vision, localization, communication and mobility. Magni is a heavyweight platform capable of moving payloads up to 100kg. It can autonomously move anything on top of it to wherever that item needs to go, avoiding obstacles along the way.

A mobile base is the heart of a modular/interoperability model of robotics. Without a shared base, parts such as robotic arms, sensors, and other tools could not find or get to their location. Even at their desired location, each would require an independent “brain” to know what to do, which would in turn require interpretation between each of the components.

If you want to build anything from laundryBots to a fleet of waiters for hire, Ubiquity's hardware, driven by the open source ROS software, is an ideal starting point for any developer transitioning into robotics.

### What is ROS?

The Robot Operating System (ROS) is an open source framework for writing robot software. In ROS, the intense research and expertise of engineers, computer scientists and more have been distilled into ready-to-use development environments and function calls. ROS has simplified the task of creating complex and robust robot behavior across a wide variety of tasks.

### Start Designing!

When a mobile base can move wherever it is needed without running into obstacles, lots of opportunities arise.

Whether you are delivering coffee, transporting people, or moving industrial equipment, progress starts with knowing where you are, what your goal is, and how to achieve it while avoiding obstacles. Magni makes learning these core concepts affordable, accessible and fun.

### What can I do while my robot is being shipped?

The Ubiquity robots are controlled by a Raspberry Pi. If you have one of these (especially if you have a camera for it), there are numerous things you can profitably do by running the Ubiquity software on your Pi. It is as though you were running a robot with no motors.

* [Install the software image](noetic_quick_start_microsd)

* [Connect to and set up the image](noetic_quick_start_connecting)

* [Set up your ROS workstation](noetic_quick_start_workstation)
