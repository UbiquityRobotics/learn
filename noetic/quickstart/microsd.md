---
title: "Software Images and Micro SD Flashing"
permalink: noetic_quick_start_microsd
group: "quick start"
rosver: noetic
nav_order: 2
nav_exclude: false
--- 

# Software Images

The Magni robots use Raspberry Pi computers, and as such require a software image to be loaded onto an SD card and then inserted into the Pi to run it.

We distribute our images with ROS and Magni packages preinstalled, which you can find on the [image download page](noetic_pi_image_downloads).

Other software suites like Conveyorbot and EZ-Map also use pre-loaded images which are delivered separately.

## Flashing Images onto a Micro SD card

The first step is taking the downloaded SD card image (imagename.img.xz) and flashing it onto an SD card of at least 16GB in size. 

We recommend using [Etcher](https://www.balena.io/etcher/) to flash the image on most systems or [Win32DiskImager](https://win32diskimager.download/) as an alternative on Windows (requires .xz files to be extracted first). Under Ubuntu Linux you can also use the GNOME Disks tool to flash images. If you haven't installed it, simply run `sudo apt install gnome-disk-utility`. Then you can double click on the downloaded image file, the GNOME Disks tool will automatically come up, and you can direct it to expand the image onto an SD card drive attached to your computer.


## Getting Started

**NOTE:** When the Raspberry Pi boots for the first time, it resizes the file system to fill the SD card, this can make the first boot take some time.

If you want to use our image for something other than our robots, please read this document.

Our image comes up as a Wifi access point. The SSID is `ubiquityrobotXXXX` where `XXXX` is part of the MAC address. The wifi password is `robotseverywhere`.

Once connected, it is possible to log into the Pi with `ssh ubuntu@10.42.0.1` with a password of `ubuntu`. If you connect up a keyboard and mouse enter the password `ubuntu` at the prompt.

<hr>

## Using Our Raspberry Pi Image Without A Magni

Ubiquity Robotics makes an image for the Raspberry Pi that it
shares freely. This image is meant to support the Magni
robot that we offer and we are not able to give much
assistance to those who don't have a Magni but who wish to use the image for their own projects.
We do offer some documents that
users of the image may find helpful and so this page is
meant to offer some assistance of the most likely questions
people may have about the image.


### Connecting
Our image comes up and starts up a WiFi hotspot by default.
To learn more about how to connect with the hotspot, see [here](noetic_quick_start_connecting).

You can also directly hook up a LAN cable to a router you may have that offers a DHCP connection.  If you do that you can find software to scan your network for the new IP address and then use a tool such as  ssh  or  putty to connect a console.

Also Note that pi3-miniuart-bt is enabled by default, so bluetooth stability may be affected. Disable it if you are using bluetooth but not the serial port.

### Disabling Services
The Ubiquity Robotics images will come up and run the software required for a Magni robot by default.

To disable the Magni software you can use this line
once you connect with a linux console as discussed above.

    sudo systemctl disable magni-base

The Ubiquity Robotics images will start up roscore by default. Robot Operating System, ROS, uses a core process and the Magni software platform is based on ROS.
In some cases users may want to disable even the startup of roscore and you can use the line that follows to do so after connecting to a linux console.

    sudo systemctl disable roscore

### Using The GPIO Lines
Our image uses some of the GPIO lines to control our
Magni robot. By default many of the lines are
unused and these are the best ones to think about
using for your own uses. You should see the section
regarding [GPIO Lines Used For The Sonar Board](<https://learn.ubiquityrobotics.com/GPIO_lines>) and
use those lines as we do not use them until they are
enabled by users who order a Magni with a Sonar board.

### Using The I2C Bus
If you want to use the I2C bus be aware that
we expect and look for certain I2C devices. You should avoid
use of the I2C addresses that we use.  Also note that many
users don't realize they must supply pullup resistors to 3.3 volts
for proper I2C operation.  As we cannot know what you have
in mind a general rule is to try something on the order of
a 3.3 kiloohm resistor.  Note that many boards for I2C
devices may have the pullup resistors already on the
little sensor boards so just be aware you must think about
the pullup resistors in general.

Please see the addresses on I2C that we use and avoid
these addresses if you can as you may get our software
interacting with your software in some cases. See our
I2C discussion that is specific to Magni but will list
the addresses used in the section titled  Guidelines For
Usage Of The I2C Bus on [**THIS_PAGE**](<https://learn.ubiquityrobotics.com/diagnostics>).

<hr>

## Troubleshooting

There are a few known issues that should be noted when using Micro SD card images on a Rasberry Pi in general.

### Corruption

If the power to the Pi is cut during writing to the card (see: most of the time when running) by turning off MCB power, it can sometimes result in image corruption and the robot failing to boot. 

Often times the operating system will trigger a succesful repair at boot (in which case booting may take longer than usual) or the image is irreparably corrupted, in which case you'll have to re-flash a new copy.

### SD Cards Wearing out

As SD cards have a finite amount of read cycles it is an inevitability that your card will eventually die or switch itself to read-only mode, although it can take years depending on the usage intensity. There are some ways to alleviate that if reliability is of utmost importance in your application:

- switching to a more durable industrial SD card

- setting up USB boot from SSD*

- using a Compute Module with eMMC memory with a Pi B compatible dock*


\*Implementation left as an excercise to the reader.
