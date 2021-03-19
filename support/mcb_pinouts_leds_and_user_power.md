---
layout: default
title:  "McbConnectorsAndLeds"
permalink: mcb_pinouts_leds_userpower
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

# MCB Connectors, Power Supplies, LEDs & Helpful Mods

This page tells explains the connections that the MCB, Master Control Board, make available as well as the LED indications on the board.

The Master Control Board is sometimes called ```Motor Control Board``` or ```MCB```.

This page highlights the power supplies available for user use and briefly explains the LEDs on the board.  More details on the LEDs is available elsewhere but it was felt a brief overview here combined with the main connections a user may want to use would be nice to have on one page.

## USB and the Other Connectors On The Raspberry Pi Computer

Although not technically on the MCB itself these can be extremely popular for many users.   The Ethernet can be very handy for lab development to not have to worry about WiFi or lack of WiFi.   

One common use is that users can add more serial ports using USB to serial ports.  Another common usage is to plug into a USB port for a device such as a Lidar or other expansion IO device.  Keep in mind the power supplied by USB ports varies and in a general way may be lower than your device requires so see the power jacks we discuss next rather than expect the USB to supply currents much over 100mA in a reliable way.

### Raspberry Pi Audio And Display connectors
The Raspberry Pi may be connected to two types of full displays.  

We sometimes use the Raspberry Pi 7" touch screen that plugs into the flat ribbon cable on one edge of this small CPU.  You will very soon see some Magni based products using this display.

There is an HDMI connector on the Raspberry Pi 3 and the recent shipments we have use the Raspberry Pi 4 which has two HDMI micro connectors.

All models of the Raspberry Pi have audio input and output on a 3.5mm compound jack.

## A Comprehensive Description Of MCB Connections

We have always had a separate document showing most all of the MCB connections.
To see this comprehensive document please see [Motor Controller Board Pinouts And Power Supply Ratings](https://learn.ubiquityrobotics.com/Magni_MCB_pinout.pdf)

This page is meant to highlight more common connectors being asked about but the earlier full list of the  MCB connections still exists in above link.


### PC Style Power Connectors with 5V and 12V

The picture below shows the 4 power supplies available from connectors or holes near the top of the board.  These can be of use for cases where a user may need 12V or 5V for their own circuits.

![Power Supply Connections For User Uses](PowerConnectorsForUser.jpg)

We prefer that the user use the  Auxilary 12V and 5V power first because they are separate from the robot main supplies.
The auxilary supplies should be able to supply 12V at 7 amps and 5V at 7 amps.  High transient loads may cause them to briefly sag.

There are other places to solder to the Aux power connections seen in this picture to the left of the large white PC style power connectors.

To make your own cables here are the mating connector housing and pin numbers
* Housing is `Molex 0015244048` (Available as Digikey # WM6982-ND)
* Crimp pins are `Molex 0002081201` (Available as DigiKey # WM2293CT-ND)

As the silkscreen shows for both of these connectors 5V is on the left then the two centeral pins are ground and on the right is 12V.  These are accurate, regulated supplies.


## Power Connectors On Bottom Of The Board  

Several expansion power connectors are located near the bottom of the board by the large automotive main Fuse

![USB Jacks, Fan Jack and Aux Motor Power](UsbJacksAndAuxPowerAndFanJack.jpg)

### The USB Style 5V Power

Located at the bottom of the MCB board as the board is in the Magni robot there are two 5V connectors that you may plug in a standard USB A cable to get 5V.  There is no USB data at all, just power.  If you need full USB connectors you need to plug into up to 4 USB jacks on the Raspberry Pi computer attached to the MCB.

If your USB power cables cannot plug into these jacks we recommend you get some right angle USB 3.0 adapters.  Note that you need the type where the plastic in the adapter is going to fit into our connectors thus the plastic must be above the hole for the part that plugs in.  Here is some search text to find the jack we recommend but there are other vendors.   `USB 3.0 Adapter 90 Degree Male to Female Down Angle Coupler Connector by Oxsubor`

### The 12V Fan Power Connector

Starting with MCB rev 5.2 we have added a standard 3-pin 12V Fan Power Jack where Ground is the left pin with 'G' and 12V is on the center pin.  It is 3-pins so 12V can be in the middle to avoid polarity issues.

This is a Molex 22232031 connector so to make cables these parts apply
* Housing is `Molex 0022013037` (Available as DigiKey # 900-0022013037-ND)
* Crimp pins are `Molex 0008500113` (Available as Digikey # WM1114CT-ND)

### The AUX MOTOR BOARD High Voltage Connectors

The 8-pin Aux Motor Board jack allows for high current full unregulated battery voltage usage.  We recommend you contact us if you wish to use this jack.  The jack was put on this board for an expansion jack for internal.  See the P*1001 jack description in the To see this comprehensive document please see [Motor Controller Board Pinouts](https://learn.ubiquityrobotics.com/Magni_MCB_pinout.pdf)

### Main Battery Fuse

The 35Amp  main battery fuse is just above the USB power jacks and directly supplies protection for the Aux Motor Board power. It can be a different color. For almost ALL our customers this fuse really could be 10 or 15 amps so you may wish to use a lower current fuse for added safety especially if you are going to be changing around boards and so on frequently.

We do see customers drop things on the still plugged in electronics (WHICH WE TRY TO WARN THEM NOT TO DO)  and a 35 amp fuse can lead to great damage.  For most all our customers this fuse does not have to be over 15 amps.

# Master Control Board Led Indicators

Although the LED indicators are discussed in assorted sections of this document a brief listing of them here was felt to be of value as a summary.

## Power Supply And MCB Status LED

A row of 5 leds can be seen to the lower left as you look at the MCB board.   In boards prior to rev 5.0 they were vertical but as of rev 5.0 and later they have been horizontal.    Here is a table showing the meaning of these leds from left to right.

The 4 power supply leds should always be on when power is active.  The 12V Main and 5V Main are critical for robot operation. Auxilary supplies are for user only.

|  Led Name | Description |
|-------------------------|----------------------|
|  12VA |  Lights when the 12V Auxilary supply is active |
|  12VM  |  Lights when the 12V Main supply is active|
|  5VM |  Lights when the 5V Main supply is active |
|  5VA |  Lights when the 5V Auxilary supply is active |
|  STAT | Shows status for the MCB onboard processor |

The STAT led is more recently labeled STATUS and is highlighted and is the most complex led.
Normally it is on with very brief drop-outs every 4 to 6 seconds.   The exact period of the dropouts indicates firmware version and is explained on firmware version page.
As of firmware v37 we have had a self test and battery monitoring function where the STATUS led if it shows odd blink patterns is indicating some form of warning or battery low indication.  See the 'Verification' page for details on the blink codes.

## The Main Power and Motor Power Active Leds

Starting with MCB version 5.0 there exists leds on the MCB board to indicates when either or both of the two main power feeds is active on the MCB.

When the main power is active there is an led on the far left and lower part of the board  that will be on.  This indicates the main power is active and the board is powered up.  If this is on but the black power button is off there may be a problem with our ECB, Electronic Circuit Breaker circuit.

When main power is active AND the red 'ESTOP switch is also in the out position the motor power led located on the far right and very low on the MCB will be on.  If this LED is stuck on and does not turn off it indicates a problem with the Motor power ECB, Electronic Circuit Breaker circuit.

## The 3.3V Onboard Power Regulator

Starting with MCB version 5.2 we have an onboard 3.3V power regulator so that the 3.3V circuits do not have to use the 3.3V power from the Raspberry Pi.  This is both a reliability enhancement as well as a way to have more 3.3V power even when a user uses a different CPU than the Raspberry Pi.  We do not directly support usage of this supply for user circuits yet.

There is an led that should be on whenever the board is powered up and that led is labeled 3.3V and is located about 2cm lower than the large white Main Power jack at the top of the board.

![3.3V Power Led And Serial Active Leds](SerialLedsAnd3p3VPowerLed.jpg)

## The Serial Communication indicators

Starting with version 5.2 of the MCB board we added two leds that normally blink very fast once the system has come up and is running.

These two leds are labeled ```SIN``` and ```SOUT``` and are located just below the right large white AUX power connector at the very top of the board near the center.

Normally the SOUT led will blink very fast right after power up of the robot.  The SIN led will take a minute or more to blink because the Raspberry Pi has to initialize the Linux operating system before it gets to starting to communicate with the MCB.    Unfortunately the led is a bit hard to see but be aware that until this SIN led starts to blink very fast the robot will not respond to any sort of command.   So the leds is valuable to tell when the robot is ready to go.

## I2C Expansion Jack Used For OLED display
We are trying to start to ship a small OLED display with each MCB but we are not quite ready yet in our supply chain however it is our goal to ship this OLED display which we have a node for already on our repository.

The reason I write this here is to explain that starting with MCB rev 5.2 we have a 4-pin 0.1" pin spacing female jack that brings out the Raspberry Pi I2C lines as well as our own MCB 3.3V.  Rev 5.1 board also had this jack but because power for 3.3 came from the Raspberry Pi we do not recommend using that jack for I2C devices.

We are planning on doing I2C based expansion boards in the future where we plan on plugging in an accessory/expansion board then plug the OLED display on top of that board.

![MCB I2C Expansion Jack](McbI2cJack.jpg)

From left to right here are the P2 I2C expansion jack pin definitions

|  Pin | Description |
|-------------------------|----------------------|
|  GND|  Ground for the power and I2C |
|  3V3  |  This is 3.3V power some boards call Vcc |
|  SCL |  The I2C Clock Line. No need to add more pullups |
|  SDA |  The I2C Data Line. No need to add more pullups |

## Making A Remote RF Estop switch

Starting with our SwitchBoard rev 2.2 it is now possible to include either your own ESTOP switch on your chassis cover or shell.    Besides that though here we describe using low cost RF control units that offer a latched set of relay connections so you can install an RF safety switch to turn off Magni Power remotely if you require that for your own safety needs.

I suggest one particular RF control unit that by adding a connector for 12V power and a connector to plug into our Switch board jack called P202 you can have such an RF controlled power switch.   Note that this could also be done on our rev 2.2 switch board to have the main power-on also with RF if you wish that feature.

We find the one that looks like this picture and has a dual button control unit allows for latched or momentary power modes.   You can find this style on EBay using the search string of  `DC12V 1CH Relay Receiver RF Transmitter Remote Control Switch`

![Remote ESTOP Switching](SwitchBoardRfEstopWiring.jpg)

This remote RF relay comes with black plastic box that latches closed firmly and covers the circuit board without taking much space. The case is not shown in this picture simply to show the power and switch wire connections. You could use good quality double sided flexible tape to attach this to the inside space near the switch board or make a bracket if you are going to ship this with your product.

Here are some details to help make cables for this sort of unit.   
Here is the connector for the ESTOP that goes to P202

    Plastic Housing:  Molex 0009501021.  Digi-Key Part Num WM18813-ND
    Female Pins:      Molex 0008701031.  Digi-Key part Num WM18820CT-ND

Here is the connector for the 12V power to supply it from MCB AUX power jack that is also described in detail earlier in this page.

    Plastic Housing:  Molex 0015244048   Digi-Key Part Num WM6982-ND
    Female Pins:      Molex 0002081201   Digi-Key Part NumWM2293CT-ND


## Alternative Wiring To Replace Our Switch Board

Some users mount the MCB in their chassis and do not want to use the `Switch Board` that we normally use for stock Magni units.  Recent switch boards have jacks to allow users to have remote power and ESTOP switches but some users want to avoid use of the switch board for mechanical reasons in their own product.

Below is the wiring you would have to supply to the MCB 14-pin P601 jack.  You MUST use 1/2 watt resistors as shown.  The switches themselves are low current and will only need to control under an amp of current at 30V

Be very careful if you use a ribbon cable or other wiring because that connector has direct battery access and EXTREME currents are possible.  Thus  Measure and verify and measure AGAIN before going 'live' with direct battery connect.   I have indicated off to the left the pins used for the battery charger because you would also not have the XLR jack for charger without our Switch Board.

![Switch Board Alternative Wiring](McbSwitchBoardAlternativeWiring.jpg)
