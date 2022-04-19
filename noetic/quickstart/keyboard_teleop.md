---
title: "Driving the robot"
permalink: noetic_quick_keyboard_driving
group: "quick start"
rosver: noetic
nav_order: 5
nav_exclude: false
--- 

# Driving the Robot

Now that you have your robot all assembled, the first test should be to try driving it around manually by using either a keyboard on your workstation or a logitech controller.

## Keyboard Teleop

Connect to your robot and log in, following the instructions in [Connecting the Robot to Your Network](noetic_quick_start_connecting). Log in to the robot from an ssh window, as before.

After logging in, the  magni_base program will be running under ROS. To verify this, type:

    rostopic list

Next, start the keyboard teleop program.  In the ssh window, type:

    rosrun teleop_twist_keyboard teleop_twist_keyboard.py

This will launch the teleop program in the robot. There will be a screen that shows the commands in a sort of crude graphical way.

Because the robot has a 'dead man timer' you must rapidly continue to press keys or the robot will automatically stop as a safety measure.  Sometimes due to spotty wifi connectivity the received commands may be too slow and you may see the robot do jerky movements but that is the deadman timer kicking in for too slow of a rate of keypresses.


Moving around:

|  u | i  | o  |
|  j | k  | l  |
|  m | ,  | .  |

The robot will respond to keystrokes as follows:


| u | circle left |
| I | straight ahead |
| o | circle right |
| j | rotate counter clockwise |
| k | all stop |
| l | rotate clockwise |
| m | circle backwards left |
| , | straight back |
| . | circle backwards right |


You can increase or decrease the speed that will be used also from keys on the keyboard but it is often desired to first stop the robot and then when you issue the increase or decrease the robot will not keep moving.  You may want to increase speed AS the robot is moving and that is ok as well.


| k | optionally stop the robot |
| q | increase speed |
| z | decrease speed |

<br>

## Running teleop-twist from a workstation

As of right now you're running the teleop node on the actual robot, and as such are sending keypress packets through the SSH tunnel to the robot. To run the node on your workstation and send messsages through ROS sockets, then you'll need to first [connect to the robot as a workstation and set it up as a ROS master](noetic_quick_start_workstation).

You'll also need to install the actual teleop node in order to run it:

     sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard

There is no major difference in running the node locally, however the principle is a good starting point and test for more advanced usage.


## Using the Optional Logitech Controller

A standard Magni Silver equipped with a Raspberry Pi with a stock Ubiquity Robotics SD card image is by default set up to work with a Logitech gamepad Controller out of the box without bothering to connect via laptop over a network. In particular the robot is tested to work with the logitech F710 controller that is widely available. It may work with other similar logitech gamepads although these are not officially supported.

If you have such a controller plug the dongle in to any USB port on the Raspberry Pi that is attached to the front of the robot. If you booted up with a Ubiquity Robotics RPi image, and have the Logitech controller USB dongle installed, the joystick should start to work in a couple of minutes. The dongle is automatically paired to the Logitech controller it does not require separate software installation.

You can test this by using the Joystick and seeing if the robot responds to commands. For safety sake, raise the robot's wheels above the floor first. (SAFETY TIP).

If the wheels don't move, check the troubleshooting section further below.

![Logitech Controller](/assets/joystick.jpg)

To operate the controller, hold down the deadman button (LB) and move the joystick.

|  Up | Drive Forward  |
|  Down | Drive Backwards  |
|  Left | Rotate counter clockwise  |
|  Right | Rotate clockwise  |

The controller is set to be very slow. You can increase the speed by using the (LT) button below the deadman, this will increase the maximum speed by about 25%.  Moving the joytick to a position between full up and full right will move the robot in a curved path.

### Modify The Logitech Joystick Default Parameters

The Logitech F710 is an approved joystick controller for the Magni robot. The default scale for the joystick is 0 to 1.0. Output values can be found in the file ```/opt/ros/$ROS_DISTRO/share/magni_teleop/param/logitech.yaml```. The values most commonly modified are these:

* ```scale_linear```     This value controls the maximum velocity the joystick will control for maximum forward or reverse joystick movement when the ```turbo``` button is not pressed.

* ```scale_angular```    This value controls the maximum turning rate in radians per second.  Some users lower this to around 1 for less sensitive turning so driving straight is easier.

### Troubleshooting

If nothing happens, you'll need to go back to the unboxing section and check to see
if the robot is gettting power, the MCB is working and you can connect to the robot via a network
(see connecting to your robot for the first time.) Check to see that your dongle is installed in the Raspberry Pi, there are fresh batteries in the Logitech Controller, and there is power going to both the Raspberry Pi and the motors. If the LEDs and the power switched are lit, you can additionally test to see if the wheels resist being turned.

Next, see if you can connect to your robot using access point (AP) mode:

    ping 10.42.0.1

(See the section on connecting to the robot.)

the robot responds by giving you its IP number. Type Control-C
to kill off the `ping` command.


Then use the following command to login to the robot for example  from a laptop networked:


    ssh ubuntu@10.42.0.1


You may get asked to authorize your key, say `yes`.

You next should be prompted for your password, type it in and you’ll be logged in

upon success:

    ---------
    Welcome to Ubuntu 16.04.2 LTS (GNU/Linux 4.4.43-v7+ armv7l)


    * Documentation: https://help.ubuntu.com

    * Management: https://landscape.canonical.com

    * Support: https://ubuntu.com/advantage


    0 package can be updated.

    0 updates are security updates.


    Last login: Sat May 6 15:57:19 2017 from 10.53.5.19

To see if the joystick is running, type:

        rostopic list

If you see `/joy`  you are good to go!

If you don't see any topics you need to launch the logitech on the robot:


    roslaunch magni_bringup base.launch >/dev/null 2>&1 &

(the `/dev/null 2 >&2 &` puts into background and stops output to screen)


    roslaunch magni_teleop logitech.launch >/dev/null 2>&1 &




The folowing command

    rostopic list

    /cmd_vel
    /diagnostics
    /joint_states
    /joy
    /left_error
    /motor_node/parameter_descriptions
    /motor_node/parameter_updates
    /right_error
    /rosout
    /rosout_agg
    /tf
    /tf_static
    /ubiquity_velocity_controller/cmd_vel
    /ubiquity_velocity_controller/odom

Shows everything is running. You now should be able use the controller to drive the robot.
