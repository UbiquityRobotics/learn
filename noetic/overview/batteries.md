---
title: "Batteries"
permalink: noetic_overview_batteries
group: overview
rosver: noetic
nav_order: 2
nav_exclude: false
---

# Batteries

The robot ships by air worldwide. The batteries are not included in order to keep shipping costs down, as they are are difficult to ship worldwide and safety restrictions vary by destination. The recommended lead acid batteries are easy to source locally.

An added advantage of not including batteries is that the robot accepts different battery sizes so the user can select batteries depending on whether they prefer a long-endurance heavier robot or a short- endurance lighter robot. In short you need to find your own batteries to put in the robot and these are commonly available online (https://www.batterysharks.com/) or in local stores that supply products for scooters, wheelchairs, uninterupped power supply systems or even automotive.

<H4 style="color:red">AT ALL TIMES IF RUNNING OR CHARGING THE BATTERY VOLTAGE CONNECTED TO THE Main BOARD MUST REMAIN 30.0V OR LESS.</H4>

## Specific Qualified Lead Acid Batteries

The robot requires **2X 12V** lead acid style batteries and typically we recommend one of the choices in this section.

| Battery Size      | Capacity (Ah)  | Runtime (hours)  | Notes |
| ---------------- | ---------------- |---------------- |----- |
| 1250/1255  | 4 - 6      |      3 - 4		 |	Used when portability of the robot is at a premium - for example if you are travelling by air with the robot. |
| 1270  		  | 7 - 10    |   6 - 8  	 	 |	This size battery makes the robot still light enough to lift. |
| 12350  		  | 30 - 35     |   24+  	 	 |	Recommended only for those who must have extraordinary endurance. This sized battery makes the robot sufficiently heavy that it will be difficult for most users to lift. |

The provided charger is specified for lead acid batteries. We provide foam inserts with the robot to fit the above battery sizes. Do not discard these foam inserts with the packaging.

While any set of batteries that can together supply roughly 24V will work, the ideal battery type is a deep cycle lead acid battery. Typically for the smaller batteries (1250, 1255, 1270) a gel type lead acid is common and for the larger types (12350) an AGM type is more common. Li battery types will work, but it should be a drop in replacement type that is fully compatible with a lead acid charging cycle and has its own battery balancing system. As the system is designed for lead acid batteries if you use anything else the battery state topic could give misleading numbers as to the true battery state, but this will not affect the ability of the robot to drive properly.

## Typical Current Draw For The Robot

Below is a table showing ```typical currents``` seen on the positive lead of the battery using a DC clamp on meter for steady states.


|  Operating State | DC Current in Amps |
|-------------------------|----------------------|
|  Stationary robot using the Pi4 with 4GByte and on flat ground with motor power off | 0.4 - 0.45 |
|  Driving on flat surface with no load at about 0.5 meters/sec  |  0.8 - 0.9 |
|  Rotating in place with no load (about same as slow driving) | 0.8 |
|  Stationary on flat ground with power to the motors |  0.5 - 0.6 |
|  Stationary on flat surface but pushing down and back on robot so wheels have to fight to stay in one place but we are not slipping just yet | 1.2 |
|  Place the robot so it cannot move and apply a great deal of torque to each wheel so the motor controller has to fight to hold the wheels firm.  | 2 - 3 |

The instantaneous currents can be well over 10 amps in certain cases but since these are transient cases for stress tests they are not considered useful for battery life calculations.

Other cases such as the robot driving up a slope with large loads of course also increases current over the above values.

## The Size Of The Battery Compartment

We ship Magni with a foam cut-out that nicely holds two 1270 format Lead Acid batteries.  Avoid discarding this insert when unpacking.

The floor of the battery compartment is always at least 205mm x 258mm.  Due to manufacturing tolerances it may be larger but that cannot be guaranteed.

From the floor to the top of the top rails on the side we have 135mm of height.  Batteries can go up taller to the top flat metal plate and that would be a height of 184mm.  These measurement are intentionally meant to avoid trying to get so close on a mm of clearance as our manufacturing cannot guarantee mm specs.

### Other information That May Help Battery Selection

We are looking into solutions that perhaps in the future may be able to support  LiFePO4 battery chemistry but we have not determined if they meet the always under 30.0V limit even when on charger and fully charged.   There are many lithium solutions that would be over 30.0V and those absolutely cannot be used.   

The stock battery charger we supply is ONLY FOR LEAD ACID batteries and will NOT work and in fact may be dangerous for other battery technologies.


## The Real-Time Clock Battery

There is also a CR2032 coin cell battery on the back of the [MCB](noetic_magnisilver_mcb).  This provides power to the real-time clock, which is **essential** for time keeping while the robot is without power or turned off. If this battery is not installed, obtain one and install it. Insert the battery with the lettering side up.

![Coin cell clip](assets/unboxing/Magni_CR2032_Battery.jpg)

 
