# Site Survey

A site survey is required for each location where you intend to run your robot.
The site survey determines the site computational assets (i.e. laptops, desktops, etc.),
the network environment (i.e. Wifi, access points, cables, etc.), and a few miscellaneous
other issues.  The site survey is organized as simple text file that is printed out
and filled in.  Print out one survey form for each robot site and fill it in.  This
document will walk you through filling in the form.

Start by visiting the
[Site Survey Form](site_survey_form/site_survey_form.md)
and printing out one form for each site you intend to survey.

## Heading

Start by writing the site name/location, date, and your name on the heading of the form.

    Site Name: _________________________________  Date:_____________  Surveyor:_________

That was easy.

## Computational Assets

In order to develop code and run robot applications, your computer(s) must have
ROS (Robot Operating System) running on it.  Since ROS only runs on a 64-bit
Linux Ubuntu operating system, it will be necessary to get this version of Linux
running on your platform.

The strategies for running 64-bit Ubuntu Linux are:

* Replace Current OS with Linux:
  You can simply replace Windows/MacOS/FreeBSD with Ubuntu Linux.  This is a pretty
  extreme solution that we do not recommend.  If you go down this route you are on
  your own.

* Partition your disk to support multiple operating systems.  This is less extreme
  and is still pretty challenging.  If you go down this route you are on your own.

* Run virtualization software on your native operating system.  We strongly recommend
  the virtualization strategy.

There are a significant number of virtualization vendors.  We provide a prebuilt
virtual image that contains Ubuntu Linux 14.04, ROS, and all the Ubiquity Robotics
software preinstalled.  This virtual image is in 
[Open Virtualization Format](https://en.wikipedia.org/wiki/Open_Virtualization_Format),
specifically it is a
[Open Virtual Appliance](https://en.wikipedia.org/wiki/Virtual_appliance)
file (i.e. an `.ova` file.)  There are approximately 10 vendors that support `.ova`
file format.  Of these virutalization vendors, we use
[Oracle VirtualBox](https://en.wikipedia.org/wiki/VirtualBox)
because it is well supported and freely available.
You are welcome to use an alternative virtualization vendor, but you will be on
your own, since we can only provide support for the vendor that we actually use
(i.e. VirtualBox.)

We partition computation assets into desktops and laptops.  At some sites you
will choose to use both a laptop and a desktop.  We only walk through the laptop
forms since they desktop form is a proper subset of the laptop form:

      [ ] Laptop:
          Operating System:
             [ ] Windows Version: [ ] 10, [ ] 8, [ ] 7, [ ] Windows Other
             [ ] MacOS Version: [ ] High Sierra, [ ] Sierra, [ ] El Capitan, [ ] Other
             [ ] Linux Distro: [ ] Ubuntu, [ ] Fedora, [ ] Other
                 Ubuntu Version: [ ] 16.04, [ ] 16.10, [ ] 17.04, [ ] 17.10, [ ] Other
          Processor: [ ] 64-bit x86, [ ] 32-bit x86, [ ] PowerPC, [ ] ARM, [ ] Other
          Virtualization Software Installed: [ ] None, [ ] VirtualBox, [ ] Other
	  Available Disk Space: ___ GB
          RJ45 Ethernet Connector: [ ] Yes, [ ] No

Please perform the following steps:

1. Check off:

      [X] Laptop:

   if you are going to use a laptop at the site.

2. Determine which operating system version you running.

          Operating System:
             [ ] Windows Version: [ ] 10, [ ] 8, [ ] 7, [ ] Windows Other
             [ ] MacOS Version: [ ] High Sierra, [ ] Sierra, [ ] El Capitan, [ ] Other
             [ ] Linux Distro: [ ] Ubuntu, [ ] Fedora, [ ] Other
                 Ubuntu Version: [ ] 16.04, [ ] 16.10, [ ] 17.04, [ ] 17.10, [ ] Other

   Depending upon your operating system you are running the version can be found as follows:

   * Windows:

     *{ waynegramlich: Not a clue *}

   * MacOS:

     *{ waynegramlich: Not a clue *}

   * Linux:

     Run the following command from a shell:

        lsb_release -d -s

     and you will get something that looks like:

        Ubuntu 16.04.2 LTS

     In this instance you would check off:

        [X] Linux Distro: [X] Ubuntu, [ ] Fedora, [ ] Other
 	    Ubuntu Version: [X] 16.04, [ ] 16.10, [ ] 17.04, [ ] 17.10, [ ] Other

     Just for your information, `LTS` stands for Long Term Support.

3. Depending upon which operating system you are running, processor architecture
   is determined as follows:

   * Windows:

     *{ waynegramlich: Not a clue *}

   * MacOS:

     *{ waynegramlich: Not a clue *}

   * Linux:

     Run the following command from a shell:

        uname -p

     and you will get something that looks like:

        x86_64

     In this instance you would check off:

        [X] Linux Distro: [X] Ubuntu, [ ] Fedora, [ ] Other
            Processor: [ ] 64-bit x86, [ ] 32-bit x86, [ ] PowerPC, [ ] ARM, [ ] Other

        Processor: [X] 64-bit x86, [ ] 32-bit x86, [ ] PowerPC, [ ] ARM, [ ] Other

   Please note that ROS only runs on 64-bit x86 at this point in time.  If you do
   not have a 64 bit x86 processor architecture you can stop now.

4. If you have previously installed some virtualization software on your operating
   system, please check off the appropriate box below:

       Virtualization Software Installed: [ ] None, [ ] VirtualBox, [ ] Other

   If you do not know if any virtualization software is installed,
   please check off `[X] None`.

5. Now determine how much available disk space you have:

       Available Disk Space: ___ GB

   Depending upon your operating system, you can determine the available disk space
   as follows:

   * Windows:

     *{ waynegramlich: not a clue}*

   * MacOS:

     *{ waynegramlich: not a clue}*

   * Linux:

     Run the following command from a shell:

         df -kh .

     and you will get something that looks as follows:

         Filesystem      Size  Used Avail Use% Mounted on
         /dev/nvme0n1p2  465G  271G  171G  62% /

     The number you want is under `Avail` and in this particular case is `171GB`.

6. Look around the edges of you laptop to see if you have an RJ45 Ethernet connector.
   In the image below, the connector on the left is an RJ45 Ethernet connector:

   ![Female RJ45 Connector](female_rj45.jpg)

   If you see such a connector, please check off `[X] Yes` as follows:

       RJ45 Ethernet Connector: [X] Yes, [ ] No

If there is a desktop machine at the site, please do a similar set of steps.

## Raspberry Pi:

Currently, all Ubiquity Robotics platforms use Raspberry Pi 3 processor to
run ROS and control the robot.  The next section looks as shown below:

    Raspberry Pi:
      [ ] Raspberry Pi: [ ] Raspberry Pi3, [ ] Other, [ ] None
      [ ] Raspberry Pi Camera: [ ] Version 1, [ ] Version 2, [ ] 3rd Party
          MicroSD Card: [ ] <8GB, [ ] 8GB, [ ] 16 GB, [ ] 32GB, [ ] >32GB
          MicroSD Card Speed: [ ] class 10, [ ] UHS 1, [ ] UHS 2, [ ] UHS 3
      [ ] HDMI display: [ ] Plugged into desktop, [ ] Unused [ ] Unavailable
      [ ] USB Keyboard: [ ] Plugged into desktop, [ ] Unused [ ] Unavailable
      [ ] USB Mouse:    [ ] Plugged into desktop, [ ] Unused [ ] Unavailable
      [ ] USB to MicroSD adapter

and identifies information your Raspberry Pi (if you have one.)

Please follow the following steps:

1. Determine which Raspberry Pi you have (if any):
   If you do not have a Raspberry Pi, you should check off `[X] None`.

   If you have a Raspberry Pi, compare it to the image immediately below:

   ![Raspberry Pi 3](raspberry_pi3.jpg)

   If your Raspberry Pi does not look exactly like the image above, please check
   off `[X] Other`; other checkoff as follows:

       [X] Raspberry Pi: [ ] Raspberry Pi3, [ ] Other, [ ] None

2. Camera: