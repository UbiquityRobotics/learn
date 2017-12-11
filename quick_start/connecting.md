---
layout: default
title:  "Connecting a Workstation for the First Time"
permalink: connecting
---
# Connecting a Workstation for the First Time

In this manual it is assumed that your workstation, whether a laptop, desktop, or virtual machine, is running Ubuntu 16.04 LTS.  ("16.04" stands for "2016, April" and "LTS" stands for "Long Term Support" which means that the software is supported for 5 years.

*{Wayne:   We probably need to also mention that Windows and Mac platform need to
run VirtualBox (or something that can run a `.vdi` file.  Also, it should be mentions, that most workstations
do not have the ability to log into a WiFi access point. }*
*{Wayne: This section chould be called WiFi configuration, or robot provisioning, or something other than connection. }*
*{Wayne: A little more overview here would really help.  Explain that we need to configure and set up the robot WiFi. }*

#### The Network

If you have received a Magni with the Raspberry Pi already installed, or loaded the default Raspberry Pi 3 image from downloads.ubiquityrobotics.com, the robot will boot up in WiFi Access Point mode. This provides its own network to which you can connect your workstation.  The SSID (network name) is `ubiquityrobotXXXX` where XXXX is a number letter combination; the password to connect is `robotseverywhere`. *{Wayne: Many customers do not know what access point mode means. }*

#### Connecting a Virtual Machine

If you are running under VirtualBox, you will have installed this virtual machine with a bridged network.  Thus, the VM will see whatever network your host system is connected to. If your workstation is running, shut it down, and shut down the VirtualBox program. Connect your host system to the `ubiquityrobotXXXX network`.  Now start VirtualBox and the workstation (that is, the Ubuntu system running under VBox); it will be connected to the robot's network `ubiquityrobotXXXX`.
*{Wayne: This is pretty brief.  I'm not sure that you need to shut down VirtuaBox.  I think that all that is required is to use the standard WiFi connectivity for you Laptop.  Note, I use ther `laptop` instead of `workstation` because most desktops do not have the ability to log into a WiFi access point. }*

#### Connecting a Linux Workstation

If you are using a Linux system instead of VirtualBox, connect to the `ubiquityrobotXXXX` network using the linux system's facilities.
*{Wayne: Perhaps use wording like, 'If you are running, Ubuntu 16.04LTS natively on your laptop...'  Notice that I say
`laptop` and not workstation, since laptops have WiFi support and desktops tend not to. }*

#### Connecting to the Robot and Logging In

Now that you are on the robot's network, you can connect to the robot itself. On your workstation, start a terminal window (ctrl-alt-t if you are using linux). In that window, log in by typing

```ssh ubuntu@ubiquityrobot.local```

You may see:

>The authenticity of host '10.42.0.1 (10.42.0.1)' can't be established.  
    ECDSA key fingerprint is SHA256:sDDeGZzL8FPY3kMmvhwjPC9wH+mGsAxJL/dNXpoYnsc.  
    Are you sure you want to continue connecting (yes/no)?

Answer `yes`

>Failed to add the host to the list of known hosts (/somepath/.ssh/known_hosts).

You will be asked for the password, which initially is "ubuntu".

```ubuntu@10.42.0.1's password:ubuntu```

Finally,

```
Welcome to Ubuntu 16.04.3 LTS (GNU/Linux 4.4.38-v7+ armv7l)
  * Documentation:  https://help.ubuntu.com
  * Management:     https://landscape.canonical.com
  * Support:        https://ubuntu.com/advantage  
  0 packages can be updated.  
  0 updates are security updates.  
 Last login: Thu Feb 11 16:30:39 2016 from 10.42.0.143
 ```

This robot's clock will have never synchronized with a time-server, so disregard the date. If you can't connect to a network, but you want to run the robot, you should synchronize dates on both the laptop and robot:

```
ssh ubuntu@ubiquityrobot.local
sudo -S date -s @`( date -u +"%s" )`
```

*{Wayne: The pifi description needs to be enabled. *}

<!--
Next, use pifi to list the nearby networks and to connect your robot to your local area network:

 ```ubuntu@ubiquityrobot:~$ pifi status```

 >Network Mangager reports AP mode support on B8:27:EB:2B:3F:6B  
 Device is currently acting as an Access Point

 ```ubuntu@ubiquityrobot:~$ pifi list seen```

>MyNetwork  
Neighbor's network  
Other Network

```ubuntu@ubiquityrobot:~$ sudo pifi add “MyNetwork”  “password”```

Note: "sudo" is a linux command that allows administrative actions.  Linux will often ask you for your password ("ubuntu", if you haven't changed it) when you use it.

Now reboot.

```sudo reboot```

The robot will reboot and attach to the “ssid” wifi network. To test,

```$ping ubiquityrobot####.local```

The ping result shows the network address of the robot:

 >PING ubiquityrobot.local (10.0.0.113) 56(84) bytes of data.  
 64 bytes from 10.0.0.113: icmp_seq=1 ttl=64 time=97.6 ms  
 64 bytes from 10.0.0.113: icmp_seq=2 ttl=64 time=5.70 ms  

Now ssh into 10.0.0.113

 ```$ ssh ubuntu@10.0.0.113```

As before:
>The authenticity of host '10.0.0.113 (10.0.0.113)' can't be established.  
ECDSA key fingerprint is SHA256:sDDeGZzL8FPY3kMmvhwjPC9wH+mGsAxJL/dNXpoYnsc.
Are you sure you want to continue connecting (yes/no)?

```yes```

>Failed to add the host to the list of known hosts (/somepath/.ssh/known_hosts).
ubuntu@10.0.0.113's password:

 ```ubuntu```

>Welcome to Ubuntu 16.04.3 LTS (GNU/Linux 4.4.38-v7+ armv7l)
  * Documentation:  https://help.ubuntu.com
  * Management:     https://landscape.canonical.com
  * Support:        https://ubuntu.com/advantage
 22 packages can be updated.
 12 updates are security updates.
 Last login: Thu Feb 11 16:31:06 2016 from 10.42.0.143

Verify that Magni is running and you are connected:

```rostopic list```

If things are ok you should see a list of topics including /joy which means you can drive with a joystick.

Now check the date.

```ubuntu@ubiquityrobot:~$ date```
 >Mon Aug 14 17:16:26 UTC 2017

Now that you have the correct date you can update the robot to get changes that have been made since the robot was manufactured.

```sudo apt-get update```  
```sudo apt-get upgrade```

This should take some time, since it may have been a while since the original image was made.
-->

Occasionally, the update/upgrade fails to complete. Usually this is due to the presence of a 'lock' file. To fix this:

        sudo rm /var/lib/dpkg/lock
