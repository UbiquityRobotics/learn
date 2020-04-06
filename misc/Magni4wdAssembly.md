
# Magni 4 Wheel Drive Assembly

The chassis is composed of  4 main compartments that are composed of
sheetmetal parts. There are 4 custom wheels with internal motors in this
4 wheel drive system.  The robot right and left side refer to
as if we were in the robot looking out the front.

The compartments are listed below.

* ```Body``` -  This is the largest compartment and holds 2 large lead acid batteries. The front battery will supply the red positive or + and the rear battery will supply the black negative or - power to each controller board.
* ```Lower Pod``` -  This holds electronics to control the rear 2 wheels. A large board called a 'MCB' has power supplies and motor driver circuits as well as a 'rear' cpu for control.  We will call this MCB the ```Rear MCB``` and a small computer called the 'rearpi' cpu.
* ```Upper Pod``` - This holds electronics to control the front 2 wheels. This also has a 'MCB' board just like the other pod that we will call the ```Front MCB``` and a small computer called the 'frontpi' cpu.
* ```Head``` (Front Compartment) - This is a 2-part electronics box that holds switches for control. The door for the head swings out to the front and then to the right side of the robot.   The part of this compartment that is attached to the front of the main body of the robot is called the 'bulkhead'.   

### Sub Assembly Of The Chassis And Wheels

At the time of this writing it is unclear to me how much of the body will be assembled at our factory. This set of assembly directions assumes that at least the body and head are assembled already along with the 4 wheels.   We will write more if that is not true.

### Placement of the batteries

The two batteries will sit with their red terminal to the left and towards the front of the robot.   THE YELLOW CABLE THAT CONNECTS THE TWO BATTERIES MUST NOT BE CONNECTED UNTIL IT IS STATED.  That way we do not have live power till near the end of the assembly.

## Sub Assembly Of Each Pod

Both the Lower and Upper Pods have just about the same electronics so they will be described together and the differences described as we discuss the assembly of the full robot.

Each pod will contain a 20cm x 16cm  ```MCB``` which contains power supplies, 3 phase motor driver circuits, a small microcomputer and a Raspberry Pi host cpu.

In discussions of a single MCB board we use directions as if the board is standing vertical as if in the Existing Magni robot and we are looking at the top of the board from the front of the Magni.  That means the power cables are at the bottom where the positive or + cables is near the lower right of the MCB board.

When inside this 4wheel drive Magni the + will be near the front right of the robot itself.

Each MCB board has 2 motor cables that will go to the 2 motors it controls.  Each MCB board also has a thick black power cable that is battery negative and a thick red power cable that will go to the + of the second large battery.

In later steps we will say when it is time to attach the MCB board into a pod after it has proper cables. The top of the MCB is the side where the large 40 and 50 pin rectangular connectors are loaded. The MCB will be on 25mm standoffs and have a long screw to hold it down to the thick soft plastic plate at the bottom of it's pod.

Each MCB will be top side up and will have the side with the two large 6mm power holes towards the right edge of the pod.  The power holes will hold M6 bolts that allow the thick black and red power wires to connect to full battery voltage.   

### Sub Assembly Of The Lower Pod

The MCB in the lower pod will hold two thick red power cables with one that will go to the upper pod as we do the assembly and on that will go all the way down to the positive side of the front battery. See picture <PIC_LMCB_POS>.   

#### Attach Power Cables To The MCB

* Place a lockwasher on a 15mm M6 hex head bolt and push the bolt from the top of the MCB through to the bottom. Screw the bolt firmly to the MCB with 3mm thick 'jam nut' attached from the bottom side.  This nut is used as a spacer for the cables.
* Find the two thick red power cables that have eyelet connectors on each side.  The two cables will be red and have the lengths of !!!FIND_CABLE_LENGTHS!!!
* Put the eyelets from the two cables onto the bolt so that both cables go off towards where the front of the robot will be after board mounting.
* Attach the cables firmly using a 5mm thick M6 nut and after that is tight add another thin 'jam nut' which are about 3mm thick and tighten it securely.

The lower pod MCB will also have two thick black negative power leads.  These thick black power leads will be of lengths !!!FIND_CABLE_LENGTHS!!!

Use the same process as above with a 20mm M6 hex head bolt except both cables are black for this step.  As before route the cables so they run towards where the front of the robot will be.  (that is toward the MCB + terminal)

#### Attaching Lower Pod MCB Motor cables

It is unclear at this time of MCB boards will have proper length cables or custom cables will have to be prepaired.   This part of the assembly process will be enhanced if explanation is required here.  !!!DO_WE_NEED_TO_DESCRIBE_CABLE_MAKING!!!

Label the motor cables with tape or something and put the label of ```rear left``` on the cable that comes from the side of the MCB near the positive + power side of the board.  Put the label way at the end by the connector that will go into the wheel cable.    Put a label of ```rear right``` at the end where the wheel connector is on the cable that comes from the side of the board near the 'BAT GND' side of the board.  

#### Attaching the MCB To The Lower Pod Soft Base

Each pod has a soft plastic 1cm thick sheet of material that we will be using to attach boards and other things for this prototype.

Locate 4 of the #6 non-threaded 1" standoffs and 4 of the 1.25" long #6 self tapping screws. (sorry for use of non metric, this is a proto and production units will not use SAE unit hardware).  You can substitute M3 hardware if required.

Using a paper pattern of a full sized MCB board place the MCB board with the bottom edge (nearest large 6mm holes) 1cm from the right wall of the pod.  The board should be all the way towards the rear leaving 1cm from the back wall to the MCB. Tape the board down so it does not move but you can still see the M3 mounting hole patterns on the right and the left edges (front and back of robot direction)

Use a pointed 'scratch awl' or other sharp pointed tool to poke through the center of the two M3 holes at the edge of the MCB that will be near the front of the robot.  Then poke a hole in the M3 hole on the edge of the MCB near the positive or + power point.   Next poke a hole in the M3 mounting hole called P711 on the same edge of the MCB, near the rear of the robot pod.

If you have a power drill you will get best results for standoffs if you pre-drill a hole of 6mm deep with a drill press or very carefully holding drill straight up. This is not required but may make fitting the board easier next.

Position the MCB with power terminals to the right of the robot and route motor and power cables from below the board towards the front of the robot.

Start with one of the M3 holes and put the #6 screw through from the top then through the 1" spacer and finally to the hole.  Screw it down MANUALLY and not with a power driver!  A power driver can easily strip the plastic.  

Do the other 3 holes so the MCB is sitting firmly on the 4 standoffs.

#### Attaching Switch Board wires to Lower pod Electronics

A small switch board will be located in each POD that controls the MCB in that pod.  The switch board will have a bundle of wires that must make its way to the Head to be connected to the Main Power switch, the ESTOP switch and the XLR charging jack.

Two red wires will go to P202 on the switch board and eventually attach to one side of the ESTOP switch in the head.

Two yellow wires will go to P201 on the switch board and be routed to attach to one side of the main robot power switch in the head.

A pair of wires that are red and brown will go to two holes on the switch board that in a normal Magni would hold a large XLR connector.   

All 6 wires form a bundle that will be routed to the head like previous other cables.

There is a 14 pin ribbon cable that plugs into the switch board and will go across the MCB and over to the MCB 14 pin jack, P501

The switch board will be located next to the MCB near the P711 M3 hole that is used to attach the MCB to the front left area inside the pod.   

A picture shows the 6 wires that will form a bundle leading off to to get routed to the head and the ribbon cable off to the MCB 14 pin jack.

### Assembly Of The Pods As One unit

We will now connect the lower pod under the upper pod.  To do this we will have to route the thick red and black power cables meant for upper to lower pod power up into the upper pod.  prior to screwing the lower to upper pod.

Also before screwing the two pods together we want to route the lower pod motor and red and black thick power cables meant to go to the battery down through the large hole in the front right of the upper pod. These cables will all go down into the head compartment but that will be explained later.

Attach lower to upper pod using suitable screws.  !!!NEED_TO_KNOW_SCREW_SIZES!!!

#### Attach Power Cables To The Upper Pod MCB

Use a process like in above ```Attach Power Cables To The MCB``` but with a few changes.   

* For the upper pod there will be only one cable as seen in picture.  The one red cable will go to the positive or + 6mm hole and the black cable will go to the 'BAT GND' 6mm hole.  
* Attach in same way as before but there will only be one cable each.
* As before route the cables so they lead to the front of the upper pod.

#### Attaching Upper Pod MCB Motor cables

Again it is unclear at this time if cables will be installed or need to be custom lengths.  The two motor cables in the upper pod go to the front wheels.
!!!DO_WE_NEED_TO_DESCRIBE_CABLE_MAKING!!!

Label the motor cables with tape or something and put the label of ```front left``` on the cable that comes from the side of the MCB near the positive + power side of the board.  Put the label way at the end by the connector that will go into the wheel cable.    Put a label of ```front right``` at the end where the wheel connector is on the cable that comes from the side of the board near the 'BAT GND' side of the board.  

#### Attaching the Front MCB To The Upper Pod Soft Base

Use a process as described in ```Attaching the MCB To The Lower Pod Soft Base``` for attaching the front MCB into the upper pod.

#### Attaching Switch Board wires to Upper Pod Electronics

!!!!SWITCH_BOARD_DESIGN_IS_NOT_YET_DECIDED_UPON!!!

There will be wires that either go to a switch board in the upper pod or go directly to the 14 pin jack, P601.  We will call them 'switch wires'
The switch wires will have to make their way into the 'head compartment' later.

### Assembly Of The Two Pods Onto The Main body

This will be best if done with two people.   It may also be useful to have what we call 2x4 pieces of wood that can be used to hold a space big enough to allow a couple hands to get into the main body to attach rear wheel motor cables and positive as well as negative battery cables.

#### First placement of Upper and Lower Pods Onto body

Make sure the pictures of cable routing are understood before you do this step.

* get the two pod assembly close to the body to allow routing of all motor, power and switch cable wires all down through the front right lower hole in the lower pod.
* Open the front door to the ```head``` all the way to be able to get your hands in to grab and route cables.   
* As you have one person lower the upper/lower pod assemblies have the other person route the battery and 4 motor cables and switch cable wires from each pod into the head compartment area
* The next step you may rest the upper and lower pods on the main body but do not screw them on yet.  
* From the front you must route the two rear wheel motor cables from the lower pod back into the main body along with the thick red and black power cables that will also have to make their way to the batteries.

### Wiring Main Power and ESTOP Switchs and front wheels

* Connect the two front wheel motor cables to the proper wheels.  These cables are very tight.  You must look for the arrows on both sides for allignment then push them together.  This can take a lot of force as they are waterproof rubber.
* Locate the 2 wires from each pod that are for main power switch and connect them to each 'side' of the main power switch after the switch has been inserted into the top of the bulkhead in the square slot.  !!!NEED_MAIN_POWER_SWITCH_WIRE_PICTURE!!!
* Locate the 2 wires from each pod that are for ESTOP switch and connect them to the two 'poles' of the estop switch.  !!!NEED_ESTOP_SWITCH_WIRING_PICTURE!!!

#### Complete wiring in the main body area

THE VERY LAST CABLE TO BE CONNECTED WILL BE THE YELLOW WIRE THAT CONNECTS THE TWO BATTERIES.   IF YOU CAN GET THE CABLE TO HAVE A 35 AMP fast blow switch your safety will be better just in case there are wiring issue that are not planned in assembly.

Again two people or long enough cables and wood spacers to hold the upper and lower pods up for wiring are important to think about before doing this step.

Route and attach both rear motor cables to the right and left wheels.

Attach the thick black power cable to the rear battery negative terminal. This will involve a screw and perhaps washers and a bolt and locknut.

Attach the thick red power cable to the front battery positive terminal.

#### Attach the batteries so the robot is enabled to power on

For the very initial test we should have estop switch activated (pushed down).  This will be safest.   

As an additional safety you can place the full robot on top of wood blocks so the wheels do not touch the ground.  We do this because it is required for the wheel self test we will run and also we do this for safety just in case something is wrong.  This is a very big and powerful robot and this is a prototype assembled in the field by non Ubiquity Robotics personnel.

Finally we will connect the yellow cable from the front battery negative (black) terminal to the rear battery positive (red) terminal.  I suggest you connect one end and just tough them for 1/8 second as a check.  It is normal there may be a small spark but it would not be normal for massive current to happen.
