
# Use 4 Raspberry Pi GPIO Lines for your own needs

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

<!--  (TODO if you can do a table for
the list of the 4 lines in text below that would help) -->


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

GPIO PiPin Comment
5 27 Status LED on Sonar board
25 22 Status LED on Sonar board.
6 31 P704 pin 2 and also to push switch on the Sonar board next to the
50 pin connector. Ground this to do a shutdown.
13 33 P705 pin 2. This is for PiFi use and only monitored if PiFi is
running. This goes to switch on Sonar board farther from 50 pin connector.

To use these as GPIO you must edit /etc/pifi/pifi.conf as follows
Change the line for status_led to be set to None like this status_led: None
Change the line for button_device_name to None like this
button_device_name: None
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
connect to 5V because it is for 3.3Volt maximum input.
