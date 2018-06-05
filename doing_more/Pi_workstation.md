---
layout: default
title:  "Running a Raspberry PI 3 like a workstation"
permalink: pi_workstation
---
The default Ubiquity Robotics SD card image contains the Lubuntu desktop and can be run just like 
any desktop computer, even when attached to a robot. (It might be a a bit awkward though!)  If you attach an HDMI monitor and
keyboard/mouse combination via the USB port, the Lubuntu Destopu will open up.  A desktop tour ofLubuntu is available 
[Lubuntu](https://www.youtube.com/watch?v=mvrlGmTmmbg). 

Try to disable GUI, just press Ctrl+Alt+T on your keyboard to open Terminal. When it opens, run the command(s) below:

sudo update-rc.d -f gdm remove
When you restart your computer, you’ll get text-mode login. To run GUI again:

startx
To enable GUI again:

sudo update-rc.d -f gdm defaults

The deafault web browser supplied (Mozilla) is often problematical, so if you plane to use a browser try installing Chrome. 
