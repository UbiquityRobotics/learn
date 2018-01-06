# Preface: Ubiquity Robotics' Robot Development Platform

**The goal of this guide is to get web, mobile, and maker developers using ROS (the Robot Operating System) via the Ubiquity Robotics development platforms.** We hope you will see the amazing possibilities and opportunities, dive in and never look back.

## What are the Ubiquity Development Platforms and What Do they Do?

> ![Magni/Loki](assets/MagniLoki2.jpg)

Magni and Loki are hardware platforms and mobile bases. When powered by ROS software, these platforms can handle vision, localization, communication and mobility. Magni is the heavy weight platform capable of moving payloads as high as 100kg. It can autonomously move anything on top of it to wherever that item needs to go, avoiding obstacles along the way.

Loki is great for the development environment. Its software compatibility with Magni saves money, time, and effort during the development phase. <!---*{Wayne: Loki currently does not have ledge detectors.  Loki really has no business on the desktop.  It is designed to live on the floor.}*--->

A mobile base is the heart of a modular/interoperability model of robotics. Without a shared base, parts such as robotic arms, sensors, and other tools could not find or get to their location. Even at their desired location, each would require an independent “brain” to know what to do, which would in turn require interpretation between each of the components.

If you want to build anything from laundryBots to a fleet of waiters for hire, Ubiquity's hardware, driven by the open source ROS software, is an ideal starting point for any developer transitioning into robotics.

## What is ROS?

The Robot Operating System (ROS) is a flexible framework for writing robot software. In ROS, the intense research and expertise of engineers, computer scientists and more have been distilled into ready-to-use development environments and function calls. ROS has simplified the task of creating complex and robust robot behavior across a wide variety of tasks.

## Start Dreaming!

When a mobile base can move wherever it is needed without running into obstacles, lots of opportunities arise.

Whether you are delivering coffee, transporting people, or moving industrial equipment, progress starts with knowing where you are, what your goal is, and how to achieve it while avoiding obstacles. Magni and Loki make learning these core concepts affordable, accessible and fun.

## What do you need to get going with these tutorials

### Communications

As the robot is delivered it of course has no connection to your local network. Because of this, the robot has its own network (called an access point or "AP mode") that enables you to [connect to it directly](connecting), without connecting to your local network. For example, you can drive the robot with our Android Robot Commander app. You can use AP mode to connect directly to the robot from a workstation, to run ROS commands such as keyboard teleoperation. However, in AP mode, the robot cannot access the Internet. AP mode is used for all the quick start tutorials.

[Connecting to your local network](connect_network) is required for more serious operation of the robot and is pretty important if you are programming the robot. A working network connection is assumed for all the tutorials after the quick start section. After you connected to a working internet connection, with ROS set up on your workstation, you can issue ROS commands to execute on the workstation which then control the robot over your local network. This may improve performance if the workstation is more powerful than the robot.
 
For the setup to work the local WiFi network must support zeroconf (the thing that allows you to find printers on your local WiFi network) and PSK (the thing that allows you to input a WiFi password). Almost all networks these days do, but if you suspect yours doesn't (for example if you can't find a WiFi connected device or you can't enter a WiFi password). Then 
you might either wind up using the robot only in AP mode (as explained in quickstart) or upgrade your router.

<!---*{Wayne: The wikipedia article on `zeroconf` is not very good.  I think it would be better to summarize the important
feature of `zeroconf` which is that it allows a processor that is running the `zeroconf` protocols to access the
robot and/or workstation using a name of the form `HOSTNAME.local`, where `HOSTNAME` is name specified by the
robot software developer.  This is instead of using a numeric internet address that continually changes. }*

<!---The statement: local network must support zeroconf and operate in in PSK (Pre-Shared Key) infrastructure mode. isn't very helpful. Perhaps we should talk about this in terms of the implications as very few people are going to know the jargon or care about it. This doesn't get better by better explainations of what this is.

<!---You might say something like: The local WiFi network must support zeroconf (the thing that allows you to find printers on your local WiFi network) and PSK (the thing that allows you to input a WiFi password). Almost all networks these days do, but if you suspect yours doesn't (for example if you can't find a WiFi connected device or you can't enter a WiFi password). Then 
you might need to either use the robot in AP mode (explained in tutorials 1-5 or upgrade your router.

<!---It could be even better if there was a way to test to make sure your network had these capabilities
"--->

### The Workstation

**The workstation must have WiFi capability.**  Most laptops do, many desktops don't.

You need a workstation to control the robot. In this manual it is assumed that your workstation, whether a laptop, desktop, or virtual machine, is running Ubuntu 16.04 LTS. <!--- ("16.04" stands for "2016, April" and "LTS" denotes "Long Term Support" which means 5 years.) ---> Ubiquity Robotics supports only this release because this is a mainstream release of the most popular Linux variant. ROS itself is written on top of Ubuntu Linux.

<!--- *{ Wayne: I think we need to describe what WiFi capability "means".  Maybe something like, the workstation needs
the ability to connected to a WiFi access point, such as a small office/home office.  I'll let you wordsmith this. }*

Not everyone has Ubuntu Linux installed on their machine, so we've created a virtual machine (VM) as a downloadable VirtualBox image. This is a system that allows most any computer to pretend that it is a Ubuntu Linux machine. Our VM is preconfigured with Ubuntu, ROS (Robot Operating System) and Ubiquity Robotics' workstation software.

Note that the robot itself runs Ubuntu Linux.

Instructions for workstation setup are given in the "doing more with your robot" section.

