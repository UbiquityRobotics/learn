<img src="/assets/ezmap/ezmap_logo.png" alt="" width="500">

<p>Welcome to EZ-Map. Following this wiki should allow you to to map a room with a lidar and set up routes to execute, or drive the robot around using video streaming and remote control.<p>

<p>We recommend viewing a demo video to get a sense of how it works first:<p>

<iframe width="640" height="360" src="https://www.youtube-nocookie.com/embed/3eAT3yVr2AM" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


## Sensors

You should have two sensors installed on the robot before starting up the software:

- Pi Camera Module v2

- Leishen N301 Lidar (shown below)

<img src="/assets/ezmap/lidar.png" alt="" width="640">

When the robot first starts you’ll be asked to provide a location and orientation for both, for best results. The menu can also be skipped and later accessed in the calibration menu.

It’s possible to run the suite with other lidars sensors, however the ROS driver for it has to be additionally installed and properly launched instead of the default one.


## Flashing the SD card

The first step is taking the downloaded SD card image (image.img.xz) and flashing it onto an SD card of at least 16GB in size. We recommend using [etcher](https://www.balena.io/etcher/) to flash the image on most systems or [Win32DiskImager](https://win32diskimager.download/) as an alternative on Win10 (requires .xz files to be extracted first. Under Ubuntu Linux you can also use the GNOME Disks tool to flash images. If you haven't installed it, simply run sudo apt install gnome-disk-utility. Then you can double click on the downloaded image file, the GNOME Disks tool will automatically come up, and you can direct it to expand the image onto an SD card drive attached to your computer.



