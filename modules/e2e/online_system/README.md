#Module introduction
This module is to collect the sensor data and vehicle parameter in the environment of ROS of PX2.

#Prerequisites
1.	Ubuntu 16.04 LTS
2.	ROS Kinetic – recommend ros-kinetic-desktop-full.
3.	Px2 driveworks0.1 version

#Building and Installing the Module
Building and installing this module is accomplished by utilizing the ROS  catkin tool.

First, we should install ROS. It is assumed that ROS packages will be installed at `~/ros`. For convenience, we add the following to the `~/.bash_profile` to use the catkin tool in the new bash shell:

```
	if [ -f /opt/ros/kinetic/setup.bash ]; then
       source /opt/ros/kinetic/setup.bash
    fi
```

Next, a catkin workspace should be created to build and install that code. For example, we could create `~/catkin_ws`, and then:

```
	cd ~/catkin_ws
	mkdir src
	cd src
```

Then , the source code could be gitcloned to the `~/catkin_ws/src`.

Finally , the source code could be complied in the `~/catkin_ws` with the `catkin_make`.

All the exe and lib file will be generated in the  `~/catkin_ws/devel`
For convenince, we add the following to our `~/.bash_profile` to enter the module environment in the bash shell.


```
	source ~/catkin_ws/devel/setup.bash
```

Attention：

sometimes erros will appear saying that "msg not found". In this case , the msg module could be complied first, and then other modules could be complied afterwards.



#Module usage:
To test , a fresh bash shell could be opened to start up a ROS core:

```
	roscore
```
And open another bash shell and initialize the camera node:

```
	rosrun px2_camera_image px2_camera_node
```

 Finally open another bash shell and open the rviz to visualize the data coming from the camera.

#LICENSE
Please see the license file.
