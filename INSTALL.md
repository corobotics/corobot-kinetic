How to install the corobot code on a fresh laptop
-----------------

* Install Ubuntu 16.04
* Install ros-kinetic-desktop per wiki.ros/Install 
	* we use `~/corobot_ws/` as the base directory
	* don't forget to add `source ~/corobot_ws/devel/setup.bash` to `~/.bashrc`
* sudo apt-get ros-kinetic-freenect-launch ros-kinetic-map-server ros-kinetic-turtlebot
* Patch `/opt/ros/kinetic/lib/create_node/turtlebot_node.py` with `c._robot_run_passive()` at the end of `main()`
* `sudo apt-get autoconf`
* Install zbar per its instructions
	* `Note change to configure.ac: remove -Werror from "AM_INIT_AUTOMAKE([-Wall -Werror..."  and from AM_CFLAGS`
* Install opencv (for face recognition) per 
* Also install opencv-contrib (`git clone https://github.com/opencv/opencv_contrib.git`)
* Install udev rules for create, cameras
* Install corobot software:
	* `cd ~/corobot_ws/src`
	* `git clone https://github.com/corobotics/corobot-kinetic.git`
	* `cd ..`
	* `catkin_make`
