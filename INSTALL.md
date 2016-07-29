How to install the corobot code on a fresh laptop
-----------------

Will this help?

* Install Ubuntu 16.04 (including latest updates)

* Install `ros-kinetic-desktop` per wiki.ros/Install 

* Install other ROS packages: 

		sudo apt-get install ros-kinetic-freenect-launch ros-kinetic-map-server ros-kinetic-turtlebot

* Patch `/opt/ros/kinetic/lib/create_node/turtlebot_node.py` by adding `c._robot_run_passive()` at the end of `main()`

- Install zbar

		sudo apt-get install autoconf python-gtk2-dev v4l-utils gettext git xmlto acpi
		cd /usr/local/src
		sudo git clone https://github.com/corobotics/ZBar.git zbar
		cd zbar
		sudo autoreconf --install
		sudo ./configure --without-imagemagick --without-gtk --without-qt
		sudo make
		sudo make install

		echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
		echo "export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libv4l/v4l1compat.so" >> ~/.bashrc


	
* Install opencv (for face recognition) 
	* Download source from [here](http://sourceforge.net/projects/opencvlibrary) into `/usr/local/src/opencv`

	* Also get opencv-contrib and compile them together

			cd /usr/local/src
			sudo git clone https://github.com/opencv/opencv_contrib.git opencv-contrib
			cd opencv
			sudo mkdir build
			cd build
			sudo cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv-contrib/modules  ..
			sudo make -j5
			sudo make install

* Install udev rules for create, cameras - in /etc/udev/rules.d
	* Start with neither webcam plugged in, then plug the left one in.  Should be at /dev/video1
	* Run `udevadm info -q all /dev/video1 | grep SERIAL`
	* There should be an 8-digit hex number there (part or all of the serial number)
	* Put the serial number into the following line (replacing `serial`) :	`KERNEL=="video*", ATTRS{serial}=="serial", SYMLINK+="videoleft" MODE="0666"`
	This goes in the file /etc/udev/rules.d/10-video.rules
	* Repeat for the right camera
	* /etc/udev/rules.d/52-corobot.rules: contents are `SUBSYSTEMS=="usb", ATTRS{idProduct}=="2303", ATTRS{idVendor}=="067b", MODE="666", GROUP="corobot"`


* Install corobot software:

		cd ~/corobot_ws/src
		git clone https://github.com/corobotics/corobot-kinetic.git
		cd ..
		catkin_make
		echo "source ~/corobot_ws/devel/setup.bash" >> ~/.bashrc
