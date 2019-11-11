
# Raspberry Pi GPIO Lines In Use And Using Them For Your Own Needs

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

<!--  (TODO if you can do a table for
the list of the 4 lines in text below that would help) -->

There are many GPIO lines used for features of Magni that could also be re purposed for your own needs if that is required.     The most logical way to use these lines is use a custom 50-pin ribbon cable that would normally go to our Sonar board.  You could then make a custom board to attach to lines of this connector as long as you make sure the lines will not be used by the Magni software.

The first 40 pins of P702, the 50 pin Sensor Board connector, are the same pin numbers as the Raspberry Pi 40 pins.  Pins shown in the tables below are therefore used on both of the large dual row connectors on the Motor Controller Board.

This page explains the GPIO line usage and how to disable features if you need to use the lines yourself.   

GPIO lines not listed on this page are available for your own usage without any configuration changes to the Magni


## GPIO Lines Used For Status LED and Switches

There are two GPIO inputs used for switches and two GPIO outputs used to
drive LEDs on the Sonar Board.
If you do not use the Sonar board OR are willing to sacrifice the
features these lines support for Magni Silver and Magni Gold here is the
information required.

Look for and perhaps use P704 and P705 jacks after disabling the usage
of these by Magni.
Normally those 2 jacks and their Magni usage AND two other GPIO pins
used to drive LEDs that exist on the Magni ‘Sonar Board’ are controlled
by Magni.

| GPIO  | RasPi Pin | Default Magni Usage |
| ------------- |------------- | --------|
| 5  | 29 | Status LED on the sonar board |
| 25 | 22 | Status LED on the sonar board |
| 6  | 31 | Goes to P704 pin 2. Does a shutdown when grounded. This goes to a pushbutton on Sonar Board neqar 50 pin connector. |
| 13 | 33 | Goes to P705 pin 2. Used for PiFi and goes to a pushbutton on the Sonar Board |

To use these as GPIO you must edit /etc/pifi/pifi.conf:  
* Change the line for status_led to be set to None like this  
```status_led: None```  
* Change the line for button_device_name to None like this
```button_device_name: None```

Pifi should still work but you would not get LED indication or be able
to use the button that is used with PiFi

To be able to drive the 2 LEDs on the sonar board yourself or just use
GPIO 5 and 25 when there is no Sonar Board you will need to edit
/boot/config.txt and comment out ‘dtoverlay=ubiquity-leds-buttons’

Doing both of the above changes frees up the 4 GPIO lines for your
purposes and you should also configure them as your application requires.


GPIO lines 6 and 13 go to some 3-pin jacks, P704 and P705, where pin 1
is ground and short to pin 2 to ‘close’ or make that GPIO line low.
Pin 3 of those jacks is 3.3 volts if you need that for your sensors. Do
not short this to ground!
Pin 2 of P704 and P705 goes directly to Raspberry PI GPIO so do NOT
connect to 5V because it is for 3.3 volt maximum input.

## GPIO Lines Used For The Sonar Board

There are many other GPIO lines used for the sensor board.   If the sensor board is not in use you may ensure the configuration is not trying to use the Sonar board and then the GPIO lines would be available for your own use.   
To use the lines you must disable the sensors. As root you will need to edit robot.yaml file.

`sudo nano /etc/ubiquity/robot.yaml`  

In the robot.yaml file be sure the sonars: line is set to  None as below and if you see something other than None you need to comment out that line as shown using pound sign.   Just to be clear:  IF you set sonars: to None the sonar board will not be used although the leds and buttons explained earlier in this page may still be of value to you.   

\# Robot Configuration  
sonars: None  
\# sonars: 'pi_sonar_v1'

The Sonar Board uses many GPIO lines that are shown the table below:

| GPIO  | Pin | Comment |
| ---------- | ------------- | ------ |
| 20 | 38  |   Sonar 0 Trigger |
| 21 | 40  |   Sonar 0 Echo    |
| 12 | 32  |   Sonar 1 Trigger|
| 16 | 36  |   Sonar 1 Echo|
| 23 | 16  |   Sonar 2 Trigger|
| 24 | 18  |   Sonar 2 Echo|
 | 27 |13  |   Sonar 3 Trigger|
 | 22 |15  |   Sonar 3 Echo|
 | 19 |35  |   Sonar 4 Trigger|
 | 26 |37  |   Sonar 4 Echo|
