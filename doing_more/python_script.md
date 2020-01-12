---
layout: default
title:  "Use A Script To Control Robot Navigation"
permalink: python_script
---

#### &uarr;[top](https://ubiquityrobotics.github.io/learn/)

# Use A Script To Control Robot Navigation
Once the robot is configured and running a navigation stack based on ceiling
fiducials we can tell the robot where to go within the global map of the robot's
work area. The prerequisite for this section is the section for setup of fiducials
so you can set waypoints and goals seen in
[**THIS LINK**](https://learn.ubiquityrobotics.com/fiducials).

## Running the Script Along with Navigation
A python script that shows how to send new waypoint goals to move_basic is
shown on this page. Place this text into a file we will call move_basic_demo.py.
In one console window we need to run the navigation as discussed in prior
paragraph but as an example that window would run this command.

    roslaunch magni_demos simple_navigation.launch

Then in a second console window run the script use the following command.

    python move_basic_demo.py

## What the Script Is Doing To Command move_basic actions
The script walks through a small table of waypoints which is a simple line on
the map and waits for the robot to finish each movement before waiting for the
user to hit ENTER to go to the next point. You can disable the prompt so the
robot will navigate on it's own from higher level commands from the script by
setting waitAtEachVertex to 0.

A simple line in the form of two X,Y points is defined and then the program will
continuously cycle through telling the robot to go to one and then the next.
Users can define other patterns with more points as well as use the 3 rd
parameter in each line which is the angle the robot should assume once it
reaches the X,Y point.

The script has a scaling parameter to grow the pattern in the table and also has
an X and Y offset in meters to have the pattern be in another location in your
map.

## A Few Words About High Level Operations For The Script
A **MoveBaseGoal** class is used to describe the next goal to be moved to where
the new X and Y location as well as final orientation or 'pose' to be assumed
once at the endpoint. We form a **SimpleActionClient** and then send the new
goal to the move_base server which is the endpoint move_basic receives new
goals. Then we ask our just used client interface to wait for a result from the
move_basic server. If all goes well we have a good result and can do other
things in the script.
It is not my intent here to dig deeper but in short the client will send out a ROS
message on a ROS topic and receive the result on another ROS topic. For
details on these interactions see
[**THIS_PAGE**](http://wiki.ros.org/move_basic).

## The Python Program itself with Comments

```
#!/usr/bin/python
"""
Example client program for sending move_basic commands in a sequence
"""
import rospy
# our custom messages for the commands we will be using
import getopt, sys
python3 = True if sys.hexversion &gt; 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import tf
import traceback
import time
def printUsage():
  print "-h --help - This help menu
  print "-s --scale - Scale for the pattern in the table"
  print "-x --offsetX - Offset for X points in the pattern"
  print "-y --offsetY - Offset for Y points in the pattern"
class Controller:
  # Define a list of waypoints for the pattern to trace
  # X,Y values in meters and robot angular yaw pose in radians
  # X and Y are in meters and the 3rd value is rotation in radians
  # Here is a 1 meter long straight line
  figureLine = [\
      [ 0.00, 0.00, 0.000, "MOVE: Leg A" ], \
      [ 1.00, 0.00, 0.000, "MOVE: Leg B" ] \
      ]
"""
Constructor for our class
"""
def __init__(self):
  rospy.init_node('controller')
  # Time per loop for the main control
  self.loop_msec = 50
  waitAtEachVertex = 1 # Set this to 0 for continual movements
  waypointName = 'Line'
  waypointList = self.figureLine

  # Grow or shrink the X,Y pattern in the table
  scaleX = 1.0
  scaleY = 1.0
    # Move each X,Y point by these offsets
    offsetX = 0.0
    offsetY = 0.0

    # read commandline arguments
    try:
      opts, args = getopt.getopt(sys.argv[1:], 'hs:x:y:h', \
        ['help','scale=','ofstX=','ofstY='])

    except getopt.GetoptError as err:
      # prints something like "option -a not recognized"
      print "Error in recognized options"
      printUsage()
      sys.exit(2)
    for o, a in opts:
      # evaluate given options
      if o in ("-h", "--help"):
        print ("displaying help")
        printUsage()
        sys.exit(2)
      elif o in ("-s", "--scaleX"):
        scaleX = float(a)
        scaleY = float(a)
      elif o in ("-x", "--offsetX"):
        offsetX = float(a)
      elif o in ("-y", "--offsetY"):
      offsetY = float(a)
  print ("Scale %f offsetX %f offsetY %f" %(scaleX,offsetX,offsetY))
  print "%d waypoints is in the list " % (len(waypointList))
  # continue going through waypoints over and over.
  # If you only want to do list once exit after first for loop
  while (True):
    for waypoint in waypointList:
      x,y,yaw,comment = waypoint
      x = (x * scaleX) + offsetX
      y = (y * scaleY) + offsetY
      now = rospy.get_rostime()
      print "[%i.%i] Waypoint: %s X %f Y %f yaw %f" % \
        (now.secs,now.nsecs,comment,x, y, yaw)
      # now publish the waypoint
      moveResult = self.publishMoveBaseGoalWaitForReply( x, y, yaw, comment)
      if moveResult == True:
        print "ERROR RETURN for Waypoint: X %f Y %f yaw %f " % (x, y, yaw)
      print "[%i.%i] Waypoint: %s X %f Y %f yaw %f reached" % \
        (now.secs,now.nsecs,comment,x, y, yaw)

      # optionally wait at each vertex before going to next one
      if (waitAtEachVertex == 1):
        raw_input("Hit ENTER to go to next waypoint ... ")

# A publisher for sending commands to the follower node
# Fiducial nav use frame_id of "map", odom nav use frame_id of "odom"

def publishMoveBaseGoalWaitForReply(self, x, y, yaw, comment):
  goal = MoveBaseGoal()
  goal.target_pose.header.frame_id = "map"
  goal.target_pose.header.stamp = rospy.Time.now()
  goal.target_pose.pose.position.x = x
  goal.target_pose.pose.position.y = y

  # to send orientation with a yaw we need quaternion transform
  x , y, z, w = tf.transformations.quaternion_from_euler(0, 0, yaw)
  goal.target_pose.pose.orientation.x = x
  goal.target_pose.pose.orientation.y = y
  goal.target_pose.pose.orientation.z = z
  goal.target_pose.pose.orientation.w = w
  now = rospy.get_rostime()
  print "[%i.%i] PubMove: %s x,y,z,w of %f %f %f %f yaw %f" % \
    (now.secs,now.nsecs,comment,x,y,z,w,yaw)

  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  client.wait_for_server()

  # publish the goal to the topic
  client.send_goal(goal)

  now = rospy.get_rostime()
  print "[%i.%i] Waiting for result ..." % (now.secs, now.nsecs)
  wait = client.wait_for_result()
  if not wait:
    rospy.logerr("Action server not available!")
    rospy.signal_shutdown("Action server not available!")
  else:
    now = rospy.get_rostime()
    print "[%i.%i] Received result" % (now.secs, now.nsecs)
    return client.get_result()
"""

Main loop
"""
  def run(self):
    print "ROS publisher publishing goals to move basic"
    print "Goals sent "
if __name__ == "__main__":
  # Create an instance of our goal class
  node = Controller()
  # run it
  node.run()
```
## Summary Of What Was Shown
A simple python script was shown that sends new waypoints or destination X,Y
locations to the robot. The robot has already had a map of the working space
made so when it gets new goals to move it moves to those locations.

The simple table with the 2 endpoints of a line can be grown to support any
sort of pattern a user wants to have the robot follow. We have used an
enhanced version of this script to make the robot move in assorted patterns for longer term tests.
