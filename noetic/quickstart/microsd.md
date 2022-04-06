---
title: "Software Images and Micro SD Flashing"
permalink: noetic_quick_start_microsd
group: "quick start"
rosver: noetic
nav_order: 3
nav_exclude: false
--- 

# Software Images 

The Magni robots use Raspberry Pi computers, and as such require a software image to be loaded onto an SD card and then inserted into the Pi to run it.

We distribute an armhf image of Ubuntu with ROS and Magni packages preinstalled, which you can find on the [image download page](https://downloads.ubiquityrobotics.com/pi.html).

Other software suites like Conveyorbot and EZ-Map also use pre-loaded images which are delivered separately.

## Flashing Images into a Micro SD card

The first step is taking the downloaded SD card image (imagename.img.xz) and flashing it onto an SD card of at least 16GB in size. 

We recommend using [Etcher](https://www.balena.io/etcher/) to flash the image on most systems or [Win32DiskImager](https://win32diskimager.download/) as an alternative on Windows (requires .xz files to be extracted first. Under Ubuntu Linux you can also use the GNOME Disks tool to flash images. If you haven't installed it, simply run `sudo apt install gnome-disk-utility`. Then you can double click on the downloaded image file, the GNOME Disks tool will automatically come up, and you can direct it to expand the image onto an SD card drive attached to your computer.

## Troubleshooting

There are a few known issues that should be noted when using Micro SD card images.

### Corruption

If the power to the Pi is cut during writing to the card (see: most of the time when running) by turning off MCB power, it can sometimes result in image corruption and the robot failing to boot. 

Often times the operating system will trigger a succesful repair at boot (in which case booting may take longer than usual) or the image is irreparably corrupted, in which case you'll have to re-flash a new copy.

### SD Cards Wearing out

As SD cards have a finite amount of read cycles it is an inevitability that your card will eventually die or switch itself to read-only mode, although it can take years depending on the usage intensity. There are some ways to alleviate that if reliability is of utmost importance in your application:

- switching to a more durable industrial SD card

- setting up USB boot from SSD*

- using a Compute Module with eMMC memory with a Pi B compatible dock*


\*Implementation left as an excercise to the reader.