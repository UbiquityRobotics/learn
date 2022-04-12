---
title: "Tower and Shell"
permalink: noetic_quickstart_shell_tower
group: "magni silver (gen 5)"
rosver: noetic
nav_order: 3
nav_exclude: false
---

# Shell and Tower Assembly Instructions

**Before you begin:**

**①** It is assumed that you have a general understanding of Magni and its parts (Pi camera, Raspberry Pi, MCB, Switchboard, Batteries, etc.). If that is not so, [make sure to check that out first](noetic_overview_need_to_know).

**②** It is assumed that Magni base robot is already [unboxed and assembled](noetic_quickstart_unboxing). 

**③** The sonar board also has to be mounted mounted [as described here](noetic_quick_start_sensors#sonars).

**④** **🛑 ALWAYS DISCONNECT THE CABLE FROM THE BATTERIES BEFORE DOING ANYTHING WITH ELECTRONICS 🛑** The most important thing to note, is that you always have to have the batteries physically disconnected, while you are doing anything with the cables or electronics to prevent any short circuits to the main PCB board. Turning off the robot with the provided main switches is not enough. **Failing to do this can result in injuries and/or fried electronics!**

**⑤** If the Raspberry PI has a micro SD card mounted – avoid powering it on and off too many times just by cutting off the power. That can lead to microSD card corruption. Always try to either use touchscreen or an SSH connection to shut it down and then cut the power.

## Full BOM of the Conveyorbot (Without base Magni)

![Mechanical Parts](assets/shell_tower/mechanical_parts.jpg)

### Mechanical Parts BOM
1. 1x Shell
2. 1x Lidar Mount
3. 1x Touchscreen Back Cover
4. 1x Tower Angle Front
5. 1x Touchscreen Front Cover
6. 1x Top Shelf Holder
7. 1x Tower Angle Bottom
8. 2x Shelves
9. 1x Front Shell Cover
10. 1x Front Charger Cover
11. 1x Battery Shell Cover
12. 2x Touchscreen Holders
13. 2x Shell Distancers (if these are not found in the package, you should have gotten extra washers in the SHELL bolts bag)

### Electrical BOM
  - 1x Touchscreen
  - 1x Emergency switch custom cable (red wires)
  - 1x Power switch custom cable (black wires)
  - 1x Black Shell Power switch
  - 1x USB micro cable 1m
  - 2x Flat cable 1m
  - 1x Adhesive Cable clip

### Bolt BOM
  - 3x Grommet 20mm (black rubber to cover lidar holes with)
  - 4x 	M6 x 2-2.5mm thick washers -  temporary replacement for Shell Distancers (2-2.5mm thickness whichever is available)
  - 4x	M3x5mm Cap Socket
  - 12x	M3x15mm Cap Socket
  - 4x	M3x8mm Cap Socket
  - 8x	M4x8mm Cap Socket
  - 2x	M4x8-10mm Thumb Screw (8 or 10 mm whichever is available)
  - 12x	M5x15mm Cap Socket
  - 10x	M5x15mm Button Socket
  - 6x	M6x15mm Cap Socket

Screw types used and their names in the instructions:

![Screw Types](assets/shell_tower/bolts.png)

What tools will be needed to assemble the Breadcrumb together:
 - small screwdriver (for M2 camera bolts)
 - big screwdriver (for mounting emergency switch)
 - M3 Allen key
 - M5 Allen key
 - M6 Allen key 
 - Painter's tape

---

<iframe width="640" height="360" src="https://www.youtube.com/embed/RDe5XLQTHwM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<iframe width="640" height="360" src="https://www.youtube.com/embed/O-YKXxbponM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

## Shell Assembly

&nbsp;

**①** Remove the top plate and the Sonar board from the robot by unscrewing it and disconnecting the flat cable. Also unscrew the camera from its holder (**4xM2 screws**) – **the camera is going to be remounted onto the Tower later**. Here if the picture of how the robot looks without the sonars and camera. 
   
Note the batteries orientation - they need to have terminals at the back of the robot as shown (This is to ensure easy reconnecting of main battery cable after the shell is mounted).

![Base Robot Without The Camera](assets/shell_tower/magni_wo_camera.png)

&nbsp;

**②** Take lidar cable and route the MOLEX and ethernet as shown

![Lidar cabling](assets/shell_tower/lidar_cabling.jpg)

&nbsp;

**③** Connect the MOLEX power connector to the MCB (like shown with green arrow). Do not plug it into the connector with the red cross. Next plug the LAN connector into the RPI LAN port (purple). Notice also that the other end of the LIDAR cable is routed from battery compartment through the spacer hole (yellow arrow).

![Lidar cabling 2](assets/shell_tower/lidar_cabling2.jpg)

    
    
Here we are going to mount the lidar on LEFT side from the robots perspective, but there are other lidar positions available - [see picture](assets/shell_tower/lidar_mounts2.png). If another position is preferred, please route the lidar cables accordingly.

![Coordinate system](assets/shell_tower/coordinates.jpg)

Lidar connector needs to be guided from the battery compartment **through** the spacer hole (purple arrow) under the camera mount. It needs to come out **over** the sonar board (green arrow). Pull the cable out **only a short amount** like shown (stop at red line).

![Lidar cabling 3](assets/shell_tower/lidar_cabling3.jpg)

&nbsp;

**④** Now prepare the shell by mounting the two shell switches. Red ESTOP button is assembled from two separate pieces. To install, first disassemble it by turning red and black "mushroom" part counter-clockwise and pulling it apart. Put the "mushroom" part through the hole in the shell like shown.

![Disassembled ESTOP](https://user-images.githubusercontent.com/53408077/127981007-8b6c83c5-acd2-4087-aebe-32a92b743f72.jpg)


Assemble the estop button by pushing it lightly inwards and turn it clockwise (This part should go together easily without force). Note that the shell does not have screw holes for mounting the ESTOP button, the screws just push on the shell to keep the button in place.

![Emg Assemble](assets/shell_tower/emg_assemble.jpg)
    
Plug the black cables into the power switch and red ones into the EMG switch. Use the self-adhesive cable clip to fix the cables in place (red arrow). If the cable clip is not already attached it should be found in one of the zip bags.

![Shell Buttons Clip](assets/shell_tower/shell_buttons_clip.jpg)

&nbsp;

**⑤** Mount the LIDAR holder using **2xM5x15mm Cap Socket** screws from SHELL zip bag (red arrows). Also unscrew the front plate and save the 4 screws that hold it in place (green arrow).

![Lidar and Front plate](assets/shell_tower/lidar_front_plate.jpg)

&nbsp;

**⑥** Turn the shell and use some paper tape to glue the 4 washers from zip bag SHELL over the corner holes - make sure the screws from the top will be able to go through both the shell hole and the washer. **This is a temporary solution to have the shell mounted higher so the sonars are not obstructed by the shell. In the future these washers are going to be replaced by a dedicated part.**

![Washers](assets/shell_tower/washers.jpg)

&nbsp;

**⑦** Put the shell on the robot. Be careful that you don't bend the sonars when putting the shell onto the robot (red lines). We suggest first putting the front part over the sonars like shown. 

![Mounting the shell](assets/shell_tower/shell_mounting_2.jpg)

Also use this chance to route the red and black cables through the sonar hole (green arrow). The robot with shell fully mounted should look like shown. Shell should rest fully on the magni with NO cables in between them (lidar cable and EMG switch cables can be problematic).

![Mounting shell 2](assets/shell_tower/shell_mounting_3.jpg)
<!-- ![Shell On Robot](shell_on_robot.png) -->

&nbsp;

**⑧** Connect the shell switches into the switch board – emergency button behind the red switch and square power shell switch behind the black one. 
    
There might be a loop cable connected into one of those connectors already. In that case you can disconnect it, but be sure to save it, it might come handy for future uses.

![Switch Cables](assets/shell_tower/switch_cables.jpg)

Make sure that the black button here is pressed in while the red one is released.

![Switchboard](assets/shell_tower/switchboard.jpg)

&nbsp;

**⑨** Connect the LIDAR by pulling the lidar connector through the hole just a bit and then screwing it onto the dedicated thread.

![Lidar Connection](assets/shell_tower/lidar_connection.png)

&nbsp;

**⑩** Screw the LIDAR with **4xM3x8mm Cap Socket** screws from the SHELL zip bag. You can place two screws before moving the lidar into position for easier mounting.

![Lidar screw1](assets/shell_tower/lidar_back_screws.jpg)

![Lidar Screw](assets/shell_tower/lidar_screw.png)

***

## Tower assembly

**①** Some touchscreen parts come pre-assembled and some cables already attached and tucked inside the assembly (purple arrow). First unscrew and save **4xM3x15mm screws** (red arrows)

![Touchscreen pre assembled](assets/shell_tower/touchscreen.jpg)

&nbsp;

**②** Attach the camera with **4xM2 screws** that it came attached to the robot with. If the bolt is hard to screw, simply tighten and un-tighten it a couple of times until it gets easier. After that attach the flat cable to the camera.

![Flat Cables](assets/shell_tower/flat_cables.png)

Notice how the blue sides of the flat cable connectors are turned. Please also make extra sure, that the flat cable connectors of BOTH camera and touchscreen are well connected and as fully pressed into the connectors as possible (red arrow) with the connector lock pressed in (purple arrows).

![Camera](assets/shell_tower/cam_attach.jpg)

Also use a sharpie or some other tool to indicate the camera flat cable to be able to distinguish the cables later.
 
&nbsp;

**③** A lot of times the flat cable connections being not adequate or faulty is the reason screen or camera don't work. You can test this connections (by following steps in the [Connecting The Tower To Shell](noetic_quickstart_shell_tower#connecting-the-tower-to-the-shell) Section below) by connecting to RPi and powering it up to see if the screen and camera are working - to avoid having to reassemble everything up again later.

&nbsp;

**④** Now use the **4xM3x15mm Cap Socket screws** from step 1. to mount the screen into the back plate. You can use a zip tie to mount the USB cable into the housing using the square holes.

![Screen Back Screw](assets/shell_tower/screen_back.png)

&nbsp;

**⑤** Use **6xM5x15mm Cap Socket screws** from TOWER zip bag to put the two angle parts of the tower together. 

The holes in two parts might not always fully align if the parts are just out of the box. If that is the case first screw the front two screws tightly and then use a bit of force to also screw the others. The parts should fit together with a bit of effort. 

![Angle Parts](assets/shell_tower/angle_parts.jpg)

&nbsp;

**⑥** Use **4xM3x15mm Cap Socket screws** from SCREEN zip bag to mount the screen assembly onto the angle assembly. At this point you can also unglue the protective sticker from the touchscreen if you wish.

![Screen To Angle](assets/shell_tower/screen_to_angle.jpg)

&nbsp;

**⑦**  As the last part of the tower assembly, mount the front panel using **4xM3x15mm Cap Socket screws** from SCREEN zip bag.

![Screen Front Screw](assets/shell_tower/screen_front.jpg)
    
&nbsp;

**⑧**  The cables can also be inserted into cable guides. 

![Cable Guides](assets/shell_tower/cable_guides.png)

***

## Connecting The Tower To The Shell

**①** If you haven't already, [burn the latest software image of the suite you intend to use onto a MicroSD card](noetic_quick_start_microsd). 

&nbsp;

**②** Put the assembled tower onto the robot. Be careful with the fragile flat cables.

![Tower On Shell](assets/shell_tower/tower_on_shell.png)

&nbsp;

**③** If you haven't already, please **MAKE SURE THE BATTERIES ARE DISCONNECTED FROM THE ROBOT** before continuing.

&nbsp;

**④** Disconnect the Raspberry Pi from the MCB and route the flat cables BEHIND the sonars as shown.

![Flat cable routing](assets/shell_tower/flat_cable_rounting.jpg)

&nbsp;

**⑤** Connect the two flat cables to Raspberry Pi (RPi). The screen flat cable connector is at the back and for camera in the middle of RPi. Notice that the blue part of the flat cable connector is pointed backward (touching black latch) for the screen cable and to the front for the camera cable. Make sure that these cables are pushed into the RPI as far as possible – a lot of people have problems when these connectors are not properly connected.

![RPI Connect 1](assets/shell_tower/rpi_connect1.png)

![RPI Connect 2](assets/shell_tower/rpi_connect2.png)

&nbsp;

**⑥** Connect the RPI back into the MCB board. Make very sure that the RPI pins are not off by 1 or more pins in any direction with respect to the MCB connector! 

![RPI to MCB](assets/shell_tower/rpi_to_mcb.jpg)

&nbsp;

**⑦** Connect and fold the USB cable

![USB cable](assets/shell_tower/usb_cable.jpg)

&nbsp;

**⑧** Before screwing the rest of the robot, try powering it up, to see if the screen and the camera work. You can do that by 
    
1.) first make sure emergency switch is pressed inside and square switch is switched off, 

2.) connecting the battery cable connector through the battery opening (green square), 

3.) switching on the square switch on the shell. 

The screen should be lighting up and after the robot boots you should see camera stream by clicking the Screen icon on the Conveyorbot app. The camera stream might not show up immediately - please wait at least until the battery icon shows some battery charge. If anything does not work, the fault, most probably is the bad connections of the flat cables – recheck those. Turn off the robot and disconnect the RED battery connector before continuing.

![Connection Test](assets/shell_tower/connection_test.png)

![Camera Stream](assets/shell_tower/camstream.jpg)

If everything went well touch the camera stream to exit and shutdown the RPI, wait 20 seconds for RPI to properly shutdown and then again disconnect the red battery cable before proceeding.

&nbsp;

**⑨**  Use **4xM4x8mm Cap Socket screws** which you unscrewed before to mount the front plate back.

![Front Plate](assets/shell_tower/front_plate.png)

&nbsp;

**⑩**  Mount the charging port cover if its not mounted already.

![Charging Cover](assets/shell_tower/charging_cover.png)

&nbsp;

**⑪**  Use **6xM6x15mm Cap Socket screws** from TOWER zip bag to mount the tower and the shell to the robot. 

![Tower to Shell Mount](assets/shell_tower/mount_tower_to_shell.png)

 You can use **4xM5x15mm Cap Socket screws** to mount the angle shelf holder and **10xM5x15mm button socket screws** to mount the two shelves.

![Shelves Mount](assets/shell_tower/shelves_mount.png)


&nbsp;

**⑫**  Mount black grommets into the empty LIDAR connector holes. Side point: more LIDAR positions are available. All those can be used, but then an easy parameter change needs to be done in the system of the robot.

![Lidar Mounts](assets/shell_tower/lidar_mounts.png)

![Lidar Mounts 2](assets/shell_tower/lidar_mounts2.png)


**⑬** Wherever you mounted your lidar, make sure the system knows about its location on the robot by SSH-ing into the robot and then editing the following folder
 
        sudo nano /etc/ubiquity/robot.yaml
    
    And the lidar position should be changed to the proper one. In this tutorial the lidar was mounted on the left side of the shell. Position of the camera should be downward

        lidar: {'position' : 'shell_left'} # possible: none, top_plate, shell_left, shell_right, shell_center, tower_center


**⑭** You can now reconnect the battery, power up the robot and begin using it. Remember to every time in the future you need to reconnect something first disconnect the red battery connector. The Conveyorbot assembly is now complete.