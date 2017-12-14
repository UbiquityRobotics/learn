---
layout: default
title:  "Connecting the Robot to Your Network"
permalink: connect_network
---
# Connecting the Robot to Your Network

Connect your workstation to the robot as described in [Connecting a Workstation for the First Time](connecting).  If you haven't done this step yet, the robot is connected to its own network, which is called ubiquityrobot, and your workstation is connected to that. We assume you have ssh'ed into the robot.

Use pifi to list the nearby networks:

 ```ubuntu@ubiquityrobot:~$ pifi status```

 >Network Mangager reports AP mode support on B8:27:EB:2B:3F:6B
 Device is currently acting as an Access Point

 ```ubuntu@ubiquityrobot:~$ pifi list seen```

>MyNetwork
Neighbor's network
Other Network

```ubuntu@ubiquityrobot:~$ sudo pifi add “MyNetwork”  “password”```

Note: "sudo" is a linux command that allows administrative actions.  
Linux will often ask you for your password (it's "ubuntu", if you haven't changed it) when you use it.

Now reboot the robot.

```sudo reboot```

The robot will reboot and attach to the “MyNetwork” wifi network. But your workstation is not connected to “MyNetwork”, because we left it on ubiquityrobot.  So attach your workstation to "MyNetwork".

If your workstation is a virtual machine, it accesses the network through its host.  So to change its network attachment, you must shut it down, close the virtual machine, change the host network attachment, then start the workstation again.

To test,

```$ping ubiquityrobot.local```

The ping result shows the network address of the robot:

 >PING ubiquityrobot.local (10.0.0.113) 56(84) bytes of data.
 64 bytes from 10.0.0.113: icmp_seq=1 ttl=64 time=97.6 ms
 64 bytes from 10.0.0.113: icmp_seq=2 ttl=64 time=5.70 ms

 Press control-c to stop the pinging'

Now ssh into the robot.

 ```$ ssh ubuntu@ubiquityrobot.local```

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

Verify that the robot is running and you are connected:

```rostopic list```

You should see a list of topics including /joy which means you can drive with a joystick.

At this point you can drive the robot from your workstation's
keyboard, just as in the Quick Start section called [Driving a Magni with a keyboard](keyboard_teleop).

There is some housekeeping that you can perform at this point, to keep your robot up to date.  Begin by checking the date.

```ubuntu@ubiquityrobot:~$ date```
 >Mon Aug 14 17:16:26 UTC 2017

Now that you have the correct date you can update the robot to get changes that have been made since the robot was manufactured.

```sudo apt-get update```
```sudo apt-get upgrade```

This may take some time, since it may have been a while since the original image was made.
