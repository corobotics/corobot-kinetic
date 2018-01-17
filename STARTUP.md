# How to start up the corobots:

First, make sure the battery situation is correct:

* Unplug the Create from its charger
* Find a Kinect battery, plug in the Kinect and turn it on (the connector along the Kinect cable should light up)

Next, open up a terminal and run:

* `roslaunch corobot_bringup robot_base.launch`
* Make sure that it seems to complete without errors (warnings are OK) - otherwise, the Kinect or Create may not be powered/connected.

In another terminal, run:

* `roslaunch corobot_bringup apps.launch`

This should cause the robot to launch its GUI (among other things) and appear on the web server's list of available robots.

