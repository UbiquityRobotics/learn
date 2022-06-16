Understanding how to add or change how your robot works.

After you understand how to connect to your robot and have a basic understanding how ROS works, you may wish to enable, add or change how your robot works.

Ubiquity Robotics Magni or other robots follow the same general process to start.  First they look at files in /etc/ubiquity to set up the environment and start service
calls to bring up roscore and then launch ROS nodes. there is a file called robot.yaml that set the default setup. This file or another similar file called default_robot.yaml 
in '-bringup/config' controls which features are enabled.

for example:

raspicam: {'camera_installed' : 'True', 'position' : 'forward'}  
lidar: {'lidar_installed' : 'True', 'position' : 'top_plate'}

#sonars: None
sonars: 'pi_sonar_v1' # Use this to enable sonars



force_time_sync : 'True'
oled_display: {'controller': 'SH1106'}  #enbled
# oled_display: {'controller': None} # Use this to disable OLED display

by changing how the lines are commented or specified in this file, we can enable or disable various fetures.

after editing a reboot or service restart would be required for any changes to take effect.
