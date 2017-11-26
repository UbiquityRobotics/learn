
# Using Robot and Workstation Together 

Once you have a working workstation and a working robot you will want to make them work together


Make sure you've changed the hostname

`ssh ubuntu@ubiquityrobot.local`

login with the the password which is

`ubunutu`

## Change the hostname

The very first thing you should do is change the hostname of your robot. While you can leave this as your default setup this is not recommended. To change this you edit a single file /etc/hostname that only contains the name of your robot computer on the network. You can change this using your favorite editor but its convienient (e.g. pico, nano, vi, vim).

`sudo echo NEWHOSTNAME > /etc/hostname`

The next step is to edit /etc/hosts to add two lines

` echo "  128.0.0.1 NEWHOSTNAME" >> /etc/hosts `
` echo "  128.0.0.1 NEWHOSTNAME.local" >> /etc/hosts `

If you now reboot this will come up

`sudo reboot`

Now the robot should come up with the NEWHOSTNAME as 

`ssh ubuntu@NEWHOSTNAME.local`

run pifi <documented>

connect to external 

## Set environment variables on local workstation

Now you need to go to your workstation and set environment variables. When you set up ROS it assumes that the computer that it is set up on is the robot. This is not what you want, you want 

`export ROS_MASTER_URI=http://NEWHOSTNAME.local:11311`

`source ~./bashrc`








## An alternative method
on the workstation you append to your ~/.bashrc

`ip="$(hostname) -I|cut -d ' ' -f 1)"`
`export ROS_IP=$ip`

then type

`source ~/.bashrc`

on the robot you need to type

`ip="$(hostname) -I|cut -d ' ' -f 1)"`
`export ROS_IP=$ip`
`export ROS_MASTER_URI=http://$ip:11311`
then use
`ifconfig -a`
and this gives you a set of information in particular it will give you the IP address of the robot which you will use in a moment.

on the workstation you need to type on a command line

`export ROS_MASTER_URI=http:// [ROBOT'S IP NUMBER]:11311` where you substitute [ROBOTS IP NUMBER] is the robots IP number



## Running RViz on the workstation

RViz is ROS's vizualization system and is very powerful. It allows you to vizualize practically any aspect of the robot's behavior. It can show the robot in space as well as graph practically any robot parameter. To make it work you need to type:


`rviz`

RViz is highly configurable so its possible to save a previous configuration and reuse it again and again. 

<insert some description of the configuration file, and how to make use of it and insert a link to a previously utilized configuration file>





