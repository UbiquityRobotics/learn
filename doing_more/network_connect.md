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
