---
layout: default
title:  "Connecting the Robot to Your Network"
permalink: connect_network
---

#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/)

# Connecting the Robot to Your Network

The robot has the ability to connect to your own local WiFi network. For debug or development use you can also directly connect your robot to your wired Ethernet using the jack on the raspberry Pi.

 By default the robot is configured to use the WiFi channels used in the United States.  Countries that use WiFi channels on their WiFi router that are not part of the normal channels used in the US will need to set their country code which is explained later or the robot may not see their WiFi router.

Once you have connected your robot to either the WiFi network or decided to run the robot with it's own WiFi hotspot you are likely going to want to connect your workstation to the robot as described in [Connecting a Workstation and Starting the Robot](connecting).  If you haven't connected the robot to your network yet, the robot is still connected to its own network, which is called `ubiquityrobotXXXX`, and your workstation is connected to that.

Before you go on, you should change the hostname of your robot, to distinguish your robot from others. Open a new terminal window, and log in to the robot with ssh:

    ssh ubuntu@ubiquityrobot.local

using the password which is "ubuntu".

OPTIONAL: In the interests of security, you can change the user password. Just type:

    passwd

and follow the prompts.

To change the hostname you can use pifi. Type the command:

    sudo pifi set-hostname NEWHOSTNAME

Note: "sudo" is a Linux command that allows administrative actions.  
Linux will often ask you for your password (it's "ubuntu", if you haven't changed it) when you use sudo (`sudo` stands for Super-User DO).

If you now reboot the robot the new hostname will be used

    sudo reboot

Now you can log in to the robot with NEWHOSTNAME:

    ssh ubuntu@NEWHOSTNAME.local

Use pifi to list the nearby networks:

    pifi list seen

If you are sure you have a WiFi router that should be seen but it does not show up it may be due to the router using a WiFi channel that is not used in the United States which is the default country code on our images.   We are implementing a way to set the country code that will be available in late 2020 where below we set China country code.  Refer to www.iban.com/country-codes or other reference.

    sudo pifi set-country CN

>MyNetwork   
Neighbor's network  
Other Network

We want to switch to MyNetwork, and we have now verified that it's present. So we can command:

    sudo pifi add MyNetwork password

NOTE: `MyNetwork` is SSID and `password` is password of your wireless network.

Now reboot the robot again.

    sudo reboot

The robot will reboot and try to attach to the “MyNetwork” wifi network. But your workstation is not connected to “MyNetwork”, because we left it connected to `ubiquityrobotXXXX`.  So, on a Linux machine, connect your workstation to "MyNetwork".

If your workstation is a virtual machine, it accesses the network through its host. So on the host, change the network from `ubiquityrobotXXXX` to "MyNetwork". Than return to the virtual machine.



To test,

    ping NEWHOSTNAME.local

The ping result shows the network address of the robot:

 >PING NEWHOSTNAME.local (10.0.0.113) 56(84) bytes of data.  
 64 bytes from 10.0.0.113: icmp_seq=1 ttl=64 time=97.6 ms  
 64 bytes from 10.0.0.113: icmp_seq=2 ttl=64 time=5.70 ms  

 Press control-c to stop the pinging.

If something goes wrong here, the robot may come back up in Access Point mode--that is, on the network named `ubiquityrobotXXXX`.  Reboot everything and start over.

Now ssh into the robot.

    ssh ubuntu@NEWHOSTNAME.local

As before:
>The authenticity of host '10.0.0.113 (10.0.0.113)' can't be established.
ECDSA key fingerprint is SHA256:sDDeGZzL8FPY3kMmvhwjPC9wH+mGsAxJL/dNXpoYnsc.
Are you sure you want to continue connecting (yes/no)?

    yes

>ubuntu@10.0.0.113's password:

    ubuntu

>Welcome to Ubuntu 16.04.3 LTS (GNU/Linux 4.4.38-v7+ armv7l)
  * Documentation:  https://help.ubuntu.com
  * Management:     https://landscape.canonical.com
  * Support:        https://ubuntu.com/advantage
 22 packages can be updated.
 12 updates are security updates.
 Last login: Thu Feb 11 16:31:06 2016 from 10.42.0.143

<!--
# Changing the hostname

  ```$ sudo pifi set-hostname loki227d
Changing hostname from ubiquityrobot to loki227d```

This is recommended. We suggest you keep the last two digits of the MAC address at the end of the robot's name to make it unique, but this is optional. You can chage the hostname to whatever you wish.

to change your robot's name (hostname):
-->


There is some housekeeping that you can perform at this point, to keep your robot up to date.  Begin by checking the date.

    date
 >Mon Aug 14 17:16:26 UTC 2017

Now that you have the correct date you can update the robot to get changes that have been made since the robot was manufactured.

    sudo apt-get update

    sudo apt-get upgrade

This may take some time, since it may have been a while since the original image was made.

# Finding The IP Address Of The Robot

A common problem that comes up is to find the IP address of the robot when it is not acting as the Access Point (AP) that you connect to from your workstation or other device.

## IP address Fixed If The Robot Supplies the WiFi AP
When the robot sets up it's access point and you connect to its WiFi the IP address of the robot will always be 10.42.0.1   So after you connect to the Robot Access Point (its network) you always can use ssh directly using an IP address like this:

    ssh ubuntu@10.42.0.1

## Finding The Robot IP Address on Your Own network

When you have told the robot to connect to your own wifi by setting up pifi OR when you have a lan cable from the robot to your own hard network cables we have different ways to find the robot's IP address depending on the device you are using to connect to the robot using SSH.  Suppose you found the IP address the SSH command would look like the example below but with the IP address you found

    ssh ubuntu@192.168.1.123

So lets say that you used pifi to add YOUR wifi network with SSID of ‘mynetwork’ so I can talk about what to do.
If the Magni connected to ‘mynetwork’ it will no longer present an ‘access point’ for it’s OWN network because there is no need. The Magni either uses the Pi wifi to present it’s OWN wifi Access Point, AP, OR if it is told to join your network it simply joins and gets an IP address from YOUR network access point (normally this is some Wifi Router that connects to the outside Internet.

So IF the Magni connects to your Wifi with your unique SSID (network name) the magni will be on your own network.

The other case is in the lab sometimes it is easiest to use a real ethernet cable plugged right into the robot to avoid the possible issues with WiFi.


## Use a Network Scanner To Search For All Devices

What you then must do is have some OTHER device like laptop or Windows machine or an android phone and then have a tool on that device to ‘SCAN’ and show you the IP address of the Magni.

Some scanning software will say it is a Raspberry Pi and some will say the name on the network but all will show the IP address.  There are many network scanners, I am just going to mention a few.

### Scan The Network On Windows Machine

For Windows on that network I install and then use an app called ‘Advanced IP Scanner’. Once installed IF your windows machine is also on the same wifi of ‘mynetwork’ you just hit ‘SCAN’ and look for any raspberry Pi or look for hostname like ubiquityrobot to find it’s IP address that your own router assigned to it on your network.

### Scan The Network On An Android device

If you have an android phone you can connect your phone to ‘my network’ and then use an IP scanner such as my favorite, ‘Fing’ available on google Play Store. This too will scan your network IF the android phone is in this case connected to ‘mynetwork’. Look for a raspberry pi with a name starting with ‘ubiquity’.

### Scan The Network On A Linux Workstation

If you connect a linux laptop to ‘mynetwork’ you may then use ‘iwconfig’ to look up your first 3 numbers of your IP address and then use ‘arpscan’ so this is the most ‘unfriendly’ method but it does work. Lets say you connect your Linux Laptop to the same network SSID (name) you told PiFi to connect to. Lets say you run ‘iwconfig’ and it tells you your WIFI IP starts with 192.168.1.x  where x is just what you ended up with and is a number from 1 to 253 or so.  We need the first 3 numbers in our command so to run the Linux scan this would be the command

    sudo arp-scan 192.168.1.1/24

I know this appears MAJOR cryptic but that in fact is HOW linux is in general. Geeks LOVE cryptic command line magical tools!

### Finding The Robot IP Address If you have an OLED display

Starting in late 2020 we had formed host software to drive a low cost OLED display that is usable on rev 5.2 or more recent MCB boards.  This is not yet built into our images so it would require manually downloading and configuring our OLED display in one of our github repositories. Some MCB boards were shipped with the display but some were not so this applies if you see a small plug-in board in mid right of the MCB, that would be the OLED display.  To use it you would need to look at our [OLED Display github repository](https://github.com/UbiquityRobotics/oled_display_node)
