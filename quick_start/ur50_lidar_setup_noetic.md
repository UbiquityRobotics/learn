---
layout: default
title:  "Setting up ur50 lidar Noetic"
permalink: ur50_lidar_setup_noetic
---
# Setting up ubiquity robotics default lidar

#### &uarr;[top]( https://ubiquityrobotics.github.io/learn/)

Ubuntu 20.04

ROS Noetic

**This tutorial works only with magni_robot branch: [noetic-devel](https://github.com/UbiquityRobotics/magni_robot/tree/noetic-devel/)**

In this document it is described how to set up the ur50 lidar on the Magni.

It is assumed that the ur50 lidar is already mounted and the network is setup following tutorial [Setting up UR50 lidar](ur50_lidar_setup_common.md)

# Setup to work with RaspberryPi

## Network
The lidar should come pre-configured with static IP: 192.168.42.222 and only answering to requests coming from IP 192.168.42.125, so we need to configure that on the RPI.

    sudo nano /etc/systemd/network/10-eth-dhcp.network

and replace everything in there with 

    [Match]
    Name=eth*

    Address=192.168.42.125/24
    [Network]

and then `sudo reboot` or `sudo systemctl restart systemd-networkd`. After that the IP on eth interface should always be set to 192.168.42.125 when any device is plugged into ethernet port. You can check that with

    ubuntu@pi-focal ~$ ifconfig eth0
    eth0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
            inet 192.168.42.125  netmask 255.255.255.0  broadcast 192.168.42.255
            inet6 fe80::e65f:1ff:fe33:ef3f  prefixlen 64  scopeid 0x20<link>
            ether e4:5f:01:33:ef:3f  txqueuelen 1000  (Ethernet)
            RX packets 243450  bytes 302746896 (302.7 MB)
            RX errors 0  dropped 0  overruns 0  frame 0
            TX packets 5618  bytes 2593522 (2.5 MB)
            TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0



# Compiling

    cd ~/catkin_ws/src
    git clone https://github.com/UbiquityRobotics/ls_lidar_driver
    cd ~/catkin_ws/
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make
    
    Can someone check if this works on ARM32 and ARM64?  I needed to
    
    catkin_make --pkg ls_lidar_n301
    From ARM64 SERVER Magni Build instructions

## please check

this fails to make the whole package on arm32 and arm64 noetic
    catkin_make --pkg ls_lidar_n301
    catkin_make --pkg ls_lidar_n301_driver
    catkin_make --pkg ls_lidar_n301_decoder
    
    was needed, and then I needed to create a launch file the specified ls_lidar, the default is LD.
    

This gets to a working robot, but doesn’t include the ros and magni_base service, neither does it have PiFi.

starting from 

https://ubuntu.com/download/raspberry-pi/thank-you?version=20.04.4&architecture=server-arm64+raspi


 need to burn to an SD card, once created first bringup on a RPI3, get into u-boot by attaching to a monitor and keyboard and then at the u-boot prompt (You can get the prompt by continouosly pressing any key while powering up:

setenv bootdelay -2
saveenv

Reboot and restart (you can put it into an rpi 4 now, but yu need a keyboard and monitor. PiFi doesn’t work for an ARM64 image and you need this networked for the next steps.)


20.04 server  need screen, keyboard for first login and set up.



will force password change on first login




****netplan wifi install   from CLI**********

https://linuxconfig.org/ubuntu-20-04-connect-to-wifi-from-command-line

  follow these instructions. Ubuntu ARM64 uses netplan instead of ifup/ifdown.



sudo apt update
sudo apt upgrade


install net-tools, (openssh-server if not already there)

test, sign in remote (get ip from ifconfig, ssh to ip with changed password.

It is best to do the following while attached to a monitor/keyboard/mouse, once installed in a robot, even if a screen and keyboard is attached, it is too difficult to adjust screen timeouts and screen size from an ssh attached terminal.

sudo apt install gnome-session gdm3


lubuntu https://linuxconfig.org/how-to-install-lubuntu-desktop-on-ubuntu-20-04-focal-fossa-linux

sudo apt install tasksel

sudo tasksel install lubuntu-desktop

takes about 30 minutes


I suggest you set screen time out and computer idle time to never, and login to desktop with no password as automatic.



http://wiki.ros.org/noetic/Installation/Ubuntu


do desktop-full


   22  sudo apt install mlocate (suggested)
   25  sudo apt install python-setuptools python3-setuptools
   28  sudo apt install unzip make
   
   
   37  sudo apt install build-essential

ROS + catkin install via typical instructions,  but note  ros-noetic-desktop-full does not exist for the arm32 version, but does for ARM64.

46  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   47 curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

   48  sudo apt update
          sudo apt upgrade

   50  sudo apt install ros-noetic-desktop-full

   51  sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool

source ros environment

INSTALL support for MMAL

Build ARM64 Rpi Userland,  Follow instructions from user email:


1. Userland


https://gist.github.com/satmandu/c462ab301cbe09bd6e7cf4db7f626727


This is a script you download and run as sudo

------------
#!/bin/bash  -x

workdir="${HOME}/workdir"
[[ ! -d "$workdir" ]] && ( mkdir -p "$workdir" || exit 1)
[[ ! -d "$workdir"/tmp ]] && ( mkdir -p "$workdir"/tmp || exit 1)
[[ ! -d "$workdir"/output ]] && ( mkdir -p "$workdir"/output || exit 1)
echo "workdir is ${workdir}"
tmpdir=$(mktemp -d deb_XXXX -p "$workdir"/tmp)
deb_temp=${tmpdir}/deb
extract_tmp=${tmpdir}/extract
echo "tmpdir is ${tmpdir}"

rm -rf "${workdir}"/rpiuserland.compile.log
sudo apt install -y git patchelf
is_git_repo="$(
	cd "${workdir}"/rpi-userland || exit
	git rev-parse --is-inside-work-tree 2>/dev/null
)"
if [ "$is_git_repo" ]; then
	cd "$workdir"/rpi-userland && git clean -d -x -f
else
		rm -rf "${workdir}"/rpi-userland
		cd "$workdir" && git clone --depth 1 https://github.com/raspberrypi/userland rpi-userland
fi

USERLANDREV=$(git -C "${workdir}"/rpi-userland rev-parse --short HEAD) >/dev/null
mkdir -p "${workdir}"/scripts
(
	cd "${workdir}"/scripts && curl -OL https://github.com/satmandu/docker-rpi4-imagebuilder/raw/master/scripts/patch_rpi-userland.sh
	chmod +x patch_rpi-userland.sh
)
echo "* Compiling Raspberry Pi userland source."
cd "${workdir}"/rpi-userland/ || exit
[[ -e "${workdir}"/scripts/patch_rpi-userland.sh ]] && {
	cd "${workdir}"/rpi-userland/ && "${workdir}"/scripts/patch_rpi-userland.sh
	true
}
sed -i 's/__bitwise/FDT_BITWISE/' "${workdir}"/rpi-userland/opensrc/helpers/libfdt/libfdt_env.h
sed -i 's/__force/FDT_FORCE/' "${workdir}"/rpi-userland/opensrc/helpers/libfdt/libfdt_env.h
echo "compiling rpi userland"
PKGVERSION="$(date +%d%m%Y):${USERLANDREV}"
cat <<-EOF | tee -a "${workdir}"/rpi-userland/CMakeLists.txt
SET(CPACK_GENERATOR "DEB")
SET(CPACK_PACKAGE_NAME "rpiuserland")
SET(CPACK_SET_DESTDIR TRUE)
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "noone")
SET(CPACK_PACKAGE_VERSION_MAJOR "$PKGVERSION")
SET(CPACK_PACKAGE_VERSION_MINOR "0")
SET(CPACK_PACKAGE_VERSION_PATCH "1~alpha1")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libgcc1-armhf-cross,libstdc++6-armhf-cross,libc6-armhf-cross")
include(CPack)
EOF
CROSS_COMPILE=aarch64-linux-gnu- ./buildme --aarch64 &>>"${workdir}"/rpi_userland_build.compile.log
echo "configuring package"
cd "${workdir}"/rpi-userland/build/raspberry/release/ || exit
make package
sudo rm -rf "${deb_temp}"
mkdir -p "${deb_temp}"
sudo rm -rf "${extract_tmp}"
mkdir -p "${extract_tmp}"
dpkg-deb -R "$workdir"/rpi-userland/build/raspberry/release/vmcs_host_apps-1.0-Source.deb "${extract_tmp}"/
cd "${workdir}"/rpi-userland/build/raspberry/release/ && sudo make DESTDIR="${deb_temp}"/ install

sed -i "s/1.0/$PKGVERSION/" "${extract_tmp}"/DEBIAN/control
mv "${extract_tmp}"/DEBIAN "${deb_temp}"/
rm -rf "${extract_tmp}"

mkdir -p "${deb_temp}"/etc/ld.so.conf.d/
echo '/opt/vc/lib' >"${deb_temp}"/etc/ld.so.conf.d/vc.conf

mkdir -p "${deb_temp}"/etc/environment.d
cat <<-EOF >"${deb_temp}"/etc/environment.d/10-vcgencmd.conf
	# /etc/environment.d/10-vcgencmd.conf
	# Do not edit this file
	
	PATH="/opt/vc/bin:/opt/vc/sbin"
	ROOTPATH="/opt/vc/bin:/opt/vc/sbin"
	LDPATH="/opt/vc/lib"
EOF
chmod +x "${deb_temp}"/etc/environment.d/10-vcgencmd.conf

mkdir -p "${deb_temp}"/etc/profile.d/
cat <<-'EOF' >"${deb_temp}"/etc/profile.d/98-rpi.sh
	# /etc/profile.d/98-rpi.sh
	# Adds Raspberry Pi Foundation userland binaries to path
	export PATH="$PATH:/opt/vc/bin:/opt/vc/sbin"
EOF
chmod +x "${deb_temp}"/etc/profile.d/98-rpi.sh

cat <<-EOF >"${deb_temp}"/etc/ld.so.conf.d/00-vmcs.conf
	/opt/vc/lib
EOF
SUDOPATH=$(sudo sed -n 's/\(^.*secure_path="\)//p' /etc/sudoers | sed s'/.$//')
SUDOPATH="${SUDOPATH:-/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/snap/bin}"
SUDOPATH+=":/opt/vc/bin:/opt/vc/sbin"
# Add path to sudo
mkdir -p "${deb_temp}"/etc/sudoers.d
echo "* Adding rpi util path to sudo."
cat <<-EOF >>"${deb_temp}"/etc/sudoers.d/rpi
	Defaults secure_path=$SUDOPATH
EOF
chmod 0440 "${deb_temp}"/etc/sudoers.d/rpi
sudo chown root:root "${deb_temp}"/etc/sudoers.d/rpi

# Get non-arm64 vcdbg and patch for use on armhf
mkdir -p "${deb_temp}"/usr/arm-linux-gnueabihf/lib
cd "${deb_temp}"/usr/arm-linux-gnueabihf/lib || exit
curl -OL https://github.com/Hexxeh/rpi-firmware/raw/master/vc/hardfp/opt/vc/lib/libdebug_sym.so
curl -OL https://github.com/Hexxeh/rpi-firmware/raw/master/vc/hardfp/opt/vc/lib/libelftoolchain.so
curl -OL https://github.com/Hexxeh/rpi-firmware/raw/master/vc/hardfp/opt/vc/lib/libvcos.so
cd "${deb_temp}"/opt/vc/bin/ || exit
sudo curl -OL https://github.com/Hexxeh/rpi-firmware/raw/master/vc/hardfp/opt/vc/bin/vcdbg
sudo chmod +x "${deb_temp}"/opt/vc/bin/vcdbg
sudo cp "${deb_temp}"/opt/vc/bin/vcdbg "${deb_temp}"/opt/vc/bin/vcdbg.orig
sudo patchelf --force-rpath --set-rpath "/usr/arm-linux-gnueabihf/lib" "${deb_temp}"/opt/vc/bin/vcdbg
sudo patchelf --set-interpreter /usr/arm-linux-gnueabihf/lib/ld-linux-armhf.so.3 "${deb_temp}"/opt/vc/bin/vcdbg

cd "${workdir}"/rpi-userland/build/raspberry/release/ || exit
sudo dpkg-deb --root-owner-group -b "${deb_temp}" "${workdir}"/output/  &>>"${workdir}"/rpi_userland_build.compile.log
sudo rm -rf "${workdir}"/rpi-userland/build/
sudo rm -rf "${tmpdir}"
echo "Package is at ${workdir}/output/rpiuserland_${USERLANDREV}_arm64.deb  ."
@anfederman
 

2.  get vc library into /opt

you need to build mmal package:
cd ~/catkin_ws/src
git clone https://github.com/6by9/userland.git
cd userland
git checkout 64bit_mmal
./buildme --aarch64
sudo cp -a ~/catkin_ws/src/userland/build/lib/. /opt/vc/lib/
sudo cp -r /opt/vc/. /usr
Now,
You can use the following command to test:
raspistill -o test.jpg



     sudo apt install ros-noetic-usb-cam  (if not using raspberry Pi camera
?   
?    sudo apt install network-managerl
  sudo apt install ros-noetic-rosbridge*
  sudo apt install python-is-python3
  sudo apt install ros-noetic-tf2*
  sudo apt install ros-noetic-teleop-twist*          navigation*  image-view rqt-image
  sudo apt install compressed transport
  sudo apt-get install ros-noetic-position-controllers

  sudo apt-get install ros-noetic-velocity-controllers
  
catkin


?sudo apt install chrony



for pi_sonar:

wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip 
cd pigpio-master/
make
sudo make install


enable and start the pigpiod service (You might need to move the executable to the appropriate directory.)


mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
 

   git clone http://github.com/ubiquityrobotics/ubiquity_motor
   git clone http://github.com/ubiquityrobotics/magni_robot
   git clone http://github.com/ubiquityrobotics/pi_sonar
   git clone http://github.com/ubiquityrobotics/oled_display_node
   
   


edit pi_sonar.cpp  comment out first line #ifdef  __arm__ and last lines at elseifdef to end


????

edit test_motor.py in ubiquity_motor  replace 2  ‘,e’  to ‘as e’

?????


sudo apt install python3-smbus   (for oled_dispaly_node)


edit /boot/frmware/config.txt

init_uart_baud=38400
dtparam=i2c_arm=on
dtoverlay=miniuart-bt
core_freq=250

dtoverlay=i2c-rtc,mcp7940x


Tested on Magni Silver  with LS-lidar, all nodes are working.

    

# Running

    cd ~/catkin_ws/
    source devel/setup.bash
    roslaunch lslidar_n301_decoder lslidar_n301_config.launch device_IP:=192.168.42.222

# Default lidar extrinsics

The system is setup so that lidar extrinsics can be set in two places with following priorities:
    
1st priority in `~/.ros/extrinsics/lidar_extrinsics_<POSITION>.yaml`
    
2nd priority in package `magni_description/extrinsics/lidar_extrinsics_<POSITION>.yaml`

Where `<POSITION>` is taken from `~/.ros/ubiquity/robot.yaml`, parameter `lidar_position`.

Example: 

This means that if in `~/.ros/ubiquity/robot.yaml` the parameter `lidar_position` is set to `top_plate`, like it is by default ([see default robot.yaml settings](https://github.com/UbiquityRobotics/magni_robot/blob/noetic-devel/magni_bringup/config/default_robot.yaml)), the system will first search for `~/.ros/extrinsics/lidar_extrinsics_top_plate.yaml`. If that does not exists it will search for `magni_description/extrinsics/lidar_extrinsics_top_plate.yaml`. If none of those can be found the lidar extrinsics will not be loaded.

The difference between the two locations is that inside `~/.ros/extrinsics/` the extrinsics files will be robot-specific and will not be overwritten by git pulls or automatic updates - its meant for users to store custom extrinsics files. Where as `magni_description/extrinsics/` location will be overwriten with both git pulls and updates - its meant as a storage of widely used default extrinsics.


# Example Adding a custom lidar location

You can mount the lidar in a custom location on the robot, but then you need to indicate the extrinsics (x, y, z, roll, pitch, yaw) of the custom location in robots configuration files. That can be easily done by:

1.) adding an additional extrinsics configuration into `~/.ros/extrinsics/lidar_extrinsics_<POSITION>.yaml`, where `<POSITION>` is an arbitrary name for the new configuration. Lets say a new lidar location with name "backward" must be added - lidar turned backwards. In this case we would:

    nano ~/.ros/extrinsics/lidar_extrinsics_backward.yaml

and into it insert the coordinates of the lidar turned by 180 degrees in yaw:

    # This file must be formated in the following way
    #
    # x: 0.0
    # y: 0.0
    # z: 0.0
    # roll: 0.0
    # pitch: 0.0
    # yaw: 0.0
    #
    # Otherwise you might experience "No such key" errors when running robot description urdfs

    x: 0.2
    y: 0.0
    z: 0.0
    roll: 0.0
    pitch: 0.0
    yaw: 3.14 #pi

2.) now the newly created extrinsic configuration `lidar_extrinsics_backward.yaml` must be set to be used the next time rpi boots:

    sudo nano /etc/ubiquity/robot.yaml

where the `lidar_position` must be set to "backward"

    lidar_position: 'backward' # to disable insert "None"

Now raspberry pi can either be restarted OR the systemctl reloaded with:

    sudo systemctl restart magni-base.service

The lidar is now set at a custom location which can be seen in Rviz. To see all available extrinsics files both for camera and lidar please check directories `~/.ros/extrinsics/` (user specified custom extrinsics) and `~/catkin_ws/src/magni_robot/magni_description/extrinsics/` (system default extrinsics - will be overridden with every git update)

***
