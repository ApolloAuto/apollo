In order to extend applicable scenario and convenience of TX2, we port some tools to TX2.

Camera calibration tool works outside the Docker environment, so all the commands below are executed outside Docker environment.
After flashing TX2 board using JetPack, the system version is Ubuntu16.04, and the corresponding
 version of ros is kinetic.


#### Install rosdep
```bash
$ pip install rosdep
```

Possible error:

"pip: command not found"

Solution:

Please refer to [https://pip.pypa.io/en/stable/installing/](https://pip.pypa.io/en/stable/installing/) to install pip


#### Install ros
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
--recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-ros-base
```

Possible error:
"the following packages have unmet dependencies"


Solution:
```bash
$ sudo vim /eta/apt/source.list
uncomment `universe, xenial-backports, xenial-security` and save the file.
$ sudo apt-get update
```

Package `python-rosdistro-modules` may be unable to be installed, the tip says that execute command `apt-get -f install`. 
But this command may also result to a new error: `rosdistro-modules0.7-1-all.deb has premature `.
This is because dkg has an incompatible version.


Solution:
```bash
$ sudo rm /etc/apt/sources.list.d/ros-latest.list
$ sudo apt-get -f install
$ sudo apt-get upgrade
```
And then reinstall ros.


### Install camera_calibration
(In many other tutorials, we are told to execute command `rosdep install camera_calibration`,
but we may encounter this error: `ERROR: Rosdep cannot find all required resources to answer your query`. 
Please follow these steps to solve the issue.）
```bash
$ echo 'source /opt/ros/kinetic/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ sudo rosdep init
$ rosdep update
$ sudo apt-get install ros-kinetic-camera-calibration
$ rosdep install camera_calibration
```

We use gscam to launch camera device on TX2
#### Install gscam
```bash
$ sudo apt-get install ros-kinetic-gscam
```

And then define macro GSCAM_CONFIG to set parameters. Taking calibrating video1 as an example, execute the command as shown below:
```bash
$ export GSCAM_CONFIG="v4l2src device=/dev/video1 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace"
```

And then launch gscam:
```bash
$ rosrun gscam gscam image_raw:=/camera/image_raw set_camera_info:=/camera/set_camera_info
```

Possible error:
"no element "v4l2src""

Solution:
```bash
$ sudo apt-get install gstreamer1.0-plugins-good。
```

Note:
Please make sure that there is no other program accessing to camera device. Otherwise, we may encounter this error:
```bash
libv4l2: error setting pixformat: Device or resource busy
[FATAL] [1551076251.590808356]: Failed to PAUSE stream, check your gstreamer configuration.
[FATAL] [1551076251.590954466]: Failed to initialize gscam stream!
```
The correct output of gscam is:
```bash
[ INFO] [1551074338.728073189]: Loaded camera calibration from
[ INFO] [1551074338.812774242]: Time offset: 1551068580.535
[ INFO] [1551074339.095952896]: Publishing stream...
[ INFO] [1551074339.096722645]: Started stream.
```

After launching gscam, use command `rostopic list` to check these two topics:
```bash
/camera/image_raw
/camera/camera_info
```
Cause command `rostopic` needs to communicate with `ros master`, so we should launch `roscore` first. 
Inside Docker environment, enter the /apollo directory, execute `./script/roscore.sh` start to launch `roscore`.

#### Launch camera_calibration
```bash
$ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera
```
The parameter 8x6 is the count of junction between two squares, and 0.108 is the size of each square, in mters.

As to the rest of steps for calibration, please refer to website:

[http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
