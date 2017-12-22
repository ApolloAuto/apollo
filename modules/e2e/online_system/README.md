# Sensor Data Module

Use the sensor data module to collect the sensor data and vehicle parameters in the ROS of PX2 environment.

# Prerequisites
1.	Ubuntu 16.04 LTS
2.	ROS Kinetic – recommend ros-kinetic-desktop-full.
3.	Px2 driveworks0.2 version

- [ ] First, Ubuntu 16.04 LTS2 *must* be installed.

	Apollo recommends using ROS Kinetic (ros-kinetic-desktop-full.3) with Px2 driveworks, version 0.1 to build and install the module using the ROS catkin tool.

- [ ] Next, Install ROS.

	It is assumed that ROS packages will be installed at `~/ros`. Add the following to the `~/.bash_profile` to use the catkin tool in the new bash shell:

	```
	if [ -f /opt/ros/kinetic/setup.bash ]; then source /opt/ros/kinetic/setup.bash
	```

- [ ] Create a catkin workspace to build and install that code.

    For example , create `~/catkin_ws` , and then:

	```
	cd ~/catkin_ws
	mkdir src
	cd src
	```

- [ ] gitclone the source code to the `~/catkin_ws/src`.


- [ ] Compile the source code in the `~/catkin_ws` with the `catkin_make`.  All the `.exe` and `.lib` files will be generated in the  `~/catkin_ws/dev`.

- [ ] Add the following to your `~/.bash_profile` to enter the module environment in the bash shell:

  ```
  source ~/catkin_ws/devel/setup.bash
  ```

  **NOTE：**
  An error may appear saying that *msg not found*. If this occurs , compile the `msg` module first, and then compile the rest of the modules.

## Module usage:

To test:

- [ ] Open a fresh bash shell.

- [ ] Start up a ROS core: `$ roscore` .

- [ ] Open a second bash shell and initialize the camera node using the following:

	```
	rosrun px2_camera_image front_image ./ 
    ```


- [ ] Open a third bash shell, and then open the `rviz` to visualize the data coming from the camera.

**LICENSE:**  Please see the License file.
