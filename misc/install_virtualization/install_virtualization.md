# Installing Virtualization Software on the Host Computer

Unless your host computer is already natively running Ubuntu 16.04LTS
you need to do some work to get Ubuntu 16.04 LTS running on your host
computer.

## Overview

The steps involved are:

1. You need to figure out if you need to install virtualization software.
   (Most of you will need virtualization software.)

2. Download and install the virtualization software.  Ubiquity Robotics only supports
   the VirtualBox virtualization software from Oracle.

3. Create a virtual machine using VirtualBox.

4. Load Ubuntu 16.04 into the virtual machine.

5. Install the virtual box guest extensions into the virtual machine.

With no further adeau, you can get started now.

*{ waynegramlich @ 2017Sep10 : [?.?] (#?) Test issue/defect report:
Here is some random test.}*

## Is Virtualization Software Needed

For this document, we partition native operating systems into ones
that are base Linux operating systems and non-Linux based operating systems
(e.g. Windows, Mac OS, Solaris, etc.).

You can skip reading the rest of this section,
if your host computer is running non-Linux native operating system.
All non-Linux native operating systems require the installation of
virtualization software.

If you are running Linux natively on your host computer please run the
following command in a terminal window:

        lsb_release -i -r

and it will output:

        Distributor ID: DDDDDD
	Release:        RRRRRR

where `DDDDDD` is the distributor name (e.g. Ubuntu, Redhat, etc.)
and `RRRRRR` is the release.

* If your distributor id is `Ubuntu` and the release number is 16.04, you are
  done and no virtualization software needs to be installed.

* If your distributor id is not `Ubuntu` you *must*
  install the virtualization software.

* If your distributor id is `Ubuntu`, but the release number is less than 16.04,
  you can upgrade your Ubuntu release to 16.04.  LTS releases always occur
  on in April of even years (i.e. 1x.04, where x is even.)  To upgrade,
  always go to the closest new LTS release and then upgrade LTS releases
  until you get to 16.04.  For example, to upgrade Ubuntu 11.10, first upgrade
  to 12.04, then 14.04 and finally to 16.04.  Use your web favorite search
  engine an search for "Upgrade Ubuntu xx.xx to yy.yy" to get instructions
  on how to do this.

* If your distributor id is `Ubuntu` and the release number is greater than 16.04,
  you are running a newer version that 16.04.  In theory, ROS Kinetic will
  run on such a system, but it is not heavily tested.  While it should work,
  Ubiquity Robotics does not support this configuration.  It is recommended
  that you install the virtualization software so you can run 16.04 in
  the virtualization software.
  
## Installing the VirtualBox Virtualization Software

While there are multiple virtualization software systems out there,
Ubiquity Robotics has selected
[VirtualBox](https://www.virtualbox.org/)
from Oracle for its virtualization software.  This software is
currently available with out further charge.  You ware welcome
to try one of the others virtualization software system (e.g VMWare, etc.),
but Ubiquity Robotics can not provide with *any* support anything other
than VirtualBox.

You are basically going to be working through the first few chapters of the
[VirtualBox User Manual](https://www.virtualbox.org/manual/UserManual.html).
To better understand what is going on, selected portions of chapters 1 through 4
should be read.

* Please preread chapter 1 up to and including the section on extension packs.

* Please preread appropriate section chapter 2 that corresponds to 
  which native operating system you are running.

* Please skim through chapter 3 to see what sorts of things you need to
  configure for your virtual machine.

* Please skim through chapter 4 (Guest Additions).

The remaining chapters in the user manual contain useful information, but
you do not need to preread them.

Now it is time to get started:

* Download VirtualBox for your 64-bit platform:

    https://www.virtualbox.org/wiki/Downloads

* Install VirtualBox.  Instructions are at:
    
    https://www.virtualbox.org/manual/ch02.html#install-linux-host
	
## Create Virtual Machine

Creating a virtual machine is actually pretty simple, most of the
windows can be clicked through 

1. Start VirtualBox.

   After VirtualBox has started you should see window that sort of looks like what
   is shown below:

   ![vb_start.png](install_images/vb_start.png)

   Please click on the `[New]` icon/button to start creating a virtual machine.

2. Name and operating system window

   Next you will get the "Name and operating system" window as below:

   ![vb_name_os](install_images/vb_name_os.png)

   Please enter "UR_SDE_16.04" as the `Name`, select "Linux" as the `Type`,
   make sure the `Version` is set to "Ubuntu (64-bit)".

   When that is done, click on the `[Next>]` button.

3. Memory size window

   When you get the "Memory size" window, it should look as follows:

   ![vb_memory_size](install_images/vb_memory_size.png)

   Make sure that it says 1024MB and then click on the `[Next>]` button.

4. Hard disk window

   When you get the "Hard disk" window, it should look as follows:

   ![vb_hard_disk](install_images/vb_hard_disk.png)

   Make sure that "Create a virtual hard disk now" is selected and click on the
   `[Next>]` button.

6. Hard disk file type window

   When you get the "Hard disk file type" window, it should look as follows:

   ![vb_hard_disk_file_type](install_images/vb_hard_disk_file_type.png)

   Please make sure that `VDI (VirtualBox Disk Image)` is selected and click on the
   `[Next>]` button.

7. Storage on physical hard disk window

   When you get the "Storage on physical disk" window, it should look as follows:

   ![vb_storage](install_images/vb_storage.png)

   Please make sure that `Dynamically allocated` is selected and click on the
   `[Next>]` button.

8. File location and size window

   When you get the "File location and size" window, it should look as follows:

   ![vb_file_size](install_images/vb_file_size.png)

   Please leave the location set to "UR_SDE_16.04" and adjust the file size to
   be greater than 30GB.  Please note that the slide bar is non-linear as you
   move left and write.  Please slide it until you get a file size of around 30GB.
   Afterwards, please click on the `[Create]` button.  This will cause the virtual machine
   to be created.

9. Virtual machine created

   After the virtual machine is created, the VirtualBox window should look as follows:

   ![vb_done](install_images/vb_done.png)

You have successfully created a new virtual machine.  The next task is to load
a version of ubuntu 16.04 into the virtual machine.

## Install Ubuntu 16.04 into Virtual Machine

Please perform the following steps to install Ubuntu 16.04 into your newly created
virtual machine.

1. Download Lubuntu 16.04

   `lubuntu` is a version of ubuntu that includes LXDE.  What is LXDE?
   LXDE the abbreviation for "Lightweight X11 Desktop Environment".
   What is really going on here is that ubuntu supports multiple different
   desktop graphical user interfaces.  LXDE is one of the smaller desktop
   graphical user interfaces.  The UR team selected LXDE because it is small.
   The bottom line is you need download the correct lubuntu file an follow
   the remaining steps to get it installed into your virtual machine.

   Using your web browser, please download the file 
   [`lubuntu-16.04-desktop-amd64.iso`](http://cdimage.ubuntu.com/lubuntu/releases/16.04/release/lubuntu-16.04-desktop-amd64.iso)
   and put it somewhere in your file system where you can find it.
   It is needed for the next step.

2. Start the virtual machine

   To start the virtual machine, bring up virtual box just double click
   on the icon that says `UR_SDE_16.04`:

   ![vb_done](install_images/vb_done.png)

   In short order, you should see window that looks as follows:

   ![lubuntu_disk_select](install_images/lubuntu_disk_select.png)

   The key thing here is to use the file chooser to specify the file that
   you just downloaded (i.e. `lubuntu-16.04-desktop-amd64.iso`)
   in the previous step into the window.

   Once you have selected the file please click on `[Start]`.

3. Let lubuntu 16.04 Boot

   After click on `[Start]`, the virtual machine will quickly start to boot `lubuntu`.

   Relatively quickly, you will see the screen below:

   ![lubuntu_start](install_images/lubuntu_start.png)

   This screen has a 30 second time-out on it.  Just let it time-out
   without touching your keyboard or mouse.

   After the time out the next major screen you will see looks as follows:

   ![lubuntu_mounted](install_images/lubuntu_mounted.png)

   At this point in time, your virtual machine is running lubuntu 16.04.  However,
   lubuntu has not yet been installed.  The actual installation steps come next.

4. Start lubuntu Installation

   On the virtual machine display, there are two icons at the upper left corner.
   One is labeled `Trash` and the other is labeled `Install`.  Please double click
   on the `Install` icon.  When you do this you will get the following screen:

   ![lubuntu_welcome](install_images/lubuntu_welcome.png)

   Please click on the `[Continue]` button to start the install process.

5. Prepare to Install lubuntu

   The next screen is the prepare screen and it looks as follows:

   ![lubuntu_prepare](install_images/lubuntu_prepare.png)

   Please do *NOT* check off either `Download updates while installing Lubuntu` or
   `Install third-party software for graphics and WiFi hardware, Flash, MP3 and other media`.
   Leave both of these check boxes unchecked.

   Now you can click on the `[Continue]` button.

6. Prepare virtual disk drive for lubuntu.

   The next screen you will see looks as follows:

   ![lubuntu_installation_type](install_images/lubuntu_installation_type.png)

   Be sure select `Erase disk and install Lubuntu`.  Yes, the warning is scary
   sounding, but in fact the only disk that will be effected is the virtual
   disk associated with your virtual machine.

   Please click on the `[Install Now]` button.

   You will immediately get the next screen:

   ![lubuntu_installation_type2](install_images/lubuntu_installation_type2.png)

   Please click on the `[Continue]`.

7. Set Time Zone

   You should get a time zone selection screen that looks as follows:

   ![lubuntu_time_zone](install_images/lubuntu_time_zone.png)

   Please select the time zone that make the most sense for your
   geographic location.

   Please click on the `[Continue]` button.


8. Set Keyboard

   Next, you should get a keyboard selection screen that looks as follows:

   ![lubuntu_keyboard](install_images/lubuntu_keyboard.png)

   Please select the keyboard that matches the keyboard you actually have.
   If in doubt, please leave it set to `English`.

   Next, please click on the `[Continue]` button.

9. Initial Account

   Next you should get an initial account screen that looks as follows:

   ![lubuntu_account](install_images/lubuntu_account.png)

   Please type in an account name of your choosing (the example shows `alice`)
   into the `Your Name:` field.
   In addition, please type a password into both the `Choose a password:` field.
   Please type the same exact password into the `Confirm your password:` field.
   Do *NOT* check out `Log in automatically`.  Instead leave
   `Require my password to log in` checked off.  Leave the `Encrypt my home folder`
   unchecked as well.

   Please remember your account name and password, you will need them later on.

   Please click on the `[Continue]` button.

10. Installation Window

    As lubuntu is installed you will see a screen that looks similar to
    what is shown below.

    ![lubuntu_install](install_images/lubuntu_install.png)

    It will take a while to install everything, so just lean back and enjoy
    the slide show.

11. Restart Window

    After everything is installed you will get a restart window that looks as follows:

    ![lubuntu_restart](install_images/lubuntu_restart.png)

    Please click on the `[Restart Now]` button.

    Very quickly your virtual machine should shrink its display to be small and
    and look as follows:

    ![lubuntu_restart](install_images/lubuntu_restart.png)

    If this screen, does not disappear in 5 to 10 seconds, it will be necessary
    to do a some simple extra steps to finish shutting down the virtual machine.

    Please go to your virtual box window and find the `UR_SDI_16.04` icon.
    With your mouse over the the icon, please right click your mouse button
    to get the following  pull down menu:

    ![lubuntu_machine_menu](install_images/lubuntu_machine_menu.png)

    Select the `Close>` menu and you should see a further pull right menu
    that looks as follows:

    ![lubuntu_power_off_menu](install_images/lubuntu_power_off_menu.png)

    Please select the `Power Off` menu.  Next, you will get a window that looks as follows:

    ![lubuntu_power_off_window](install_images/lubuntu_power_off_window.png)

    Please click on the `[Power Off]` button to finish powering the virtual machine.

Your VirtualBox window should now look as follows:

![vb_done](install_images/vb_done.png)

Congratulations, you have just successfully installed lubuntu 16.04 into your
`UR_SDE_16.04` virtual machine for VirtualBox.

## Install VirtualBox Guest Additions

The next set of steps is required to install the VirtualBox guest extensions.
With these extensions, you will be able to do such things such as resize the
virtual window size, etc.

1. Log In to Lubuntu 16.04

   Restart the virtual machine by double clicking on the `UR_SDE_16.04` icon
   VirtualBox window.  After a small delay you should get the following
   login window:

   ![lubuntu_login](install_images/lubuntu_login.png)

   Please type in the account name and password that previously supplied
   when you installed Lubuntu 16.04.  Now you can click on the `[Login In]`
   button to login.

   After you log in, you will get the following rather plain window:

   ![lubuntu_logged_in](install_images/lubuntu_logged_in.png)

2. Insert Guest Additions CD

   The next step is to get the Guest Additions CD installed into your virtual
   CD drive on your virtual machine.  This is done by going to the `Devices`
   pull down menu and selecting `Insert Guest Additions CD Image...` from
   the menu shown below:

   ![lubuntu_insert_guest_cd](install_images/lubuntu_insert_guest_cd.png)

   After you do that, you will get a display that looks as follows:
   
   ![lubuntu_guest_cd_inserted](install_images/lubuntu_guest_cd_inserted.png)

   Since the file manager is not needed, please click on the `[Cancel]` button
   to make it go away.

5. Bring Up A Terminal Window

   The next task requires that you type a command inside a terminal window.
   Just so you know, Linux in general and ROS in particular tends to be
   a little heavy on terminal window use.

   In order to bring up a terminal window, you first depress and hold down
   both the `[Ctrl]` and `[Alt]` keys and then depress the letter `[T]` key.
   This keyboard judo is abbreviated as Ctrl-Atl-T.  After you done these
   keyboard gymnastics you will git the following screen:

   ![lubuntu_terminal](install_images/lubuntu_terminal.png)

   Once the terminal window comes up, move the mouse pointer inside the
   terminal window and click the left mouse button to make sure that
   keyboard keystrokes will be directed into the terminal window.

   Now type the following command:

   ```sudo apt-get install -y build-essential```

   What this command does is install the standard Linux compiler programs
   (collectively called `gcc`) and another program we are going to need called

   After you type the command above, you will prompted for a password.
   Please type in the password you used to login with.

   ![lubuntu_terminal1](install_images/lubuntu_terminal1.png)

   This command will grind away for a little while and then stop.

7. Install Guest Edition Software

   Next you will install the guest edition software.  This is done with the
   following command:

   ```sudo /media/*/*/VBoxLinuxExtensions.run```

   It will cause the VirtualBox guest additions to be installed.  You may be asked
   to type in a password again.

   ![lubuntu_teriminal1](install_images/lubuntu_terminal1.png)

   Again, the compute will grind away for a little while and the it will stop.
   At this point, the guest extensions are installed.

8. Log Out and Shut Down

   Now we can logout and shutdown the virtual machine.  Do do this, you need to
   move your mouse curse to the lower right corner and   click on the power button
   icon.  This will pop a window that looks as follows:

   ![lubuntu_shutdown](install_images/lubuntu_shutdown.png)

   Please select the `[Shutdown]` button to force a log out and virtual machine
   shut down.

That concludes the guest additions installation.

## Conclusion

That is all that is required to install VirtualBox, Lubuntu 16.04, and the
VirtualBox guest extensions.

The next step is to install ROS Kinetic and the rest of the UR SDE.
