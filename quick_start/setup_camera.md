Begin editing

The Ubiquity Fiducial Follow and waypoint navigation applications require the use of the Raspicam, the camera specifically designed for the Raspberry Pi. Both version 1 and 2 cameras will work, but we strongly recomment version 2 cameras. Most issues with cameras are due to a poor connection with the ribbon cable, as it is easy to get confuded and install is backwards. the 'blue' part of the cable faces away from the camera and on the Pi faces toward the USB connectors. Make sure the cable is firmly attached with the 'lugs' pressed in.

To test for proper operation of the camera hardware, open an ssh session to the robot. at the command prompt type:

raspistill -o test.jpg

If the camera hardware and firmware are installed correctly, a image file will be produced with no error messages.
If however, you recieve an mmal error message, this means the camera was not detected by the Pi. the causes of this error are almost always due to a broken camera or rarely a poorly soldered connector on the Pi. If you have a known good camera, you might try to repeat the test to isolate a cable or hardware issue. If the proble persists, you can further isolate a firmware problem on the Pi by:

sudo rpi-update

Once yu have confirmed the camera is working, you can test the camera

by starting a camera node on the robot:

roslaunch raspicam_node camerav2_1280x720.launch &

and then:

rostopic echo /raspicam_node/image/compressed

you should see data being streamed to the screen.

The fiducial follow and waypont programs should work once started on the robot.


