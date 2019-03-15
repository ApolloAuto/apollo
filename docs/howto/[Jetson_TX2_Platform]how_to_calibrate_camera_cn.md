为了完善TX2的使用场景和方便性，对apollo的周边工具进行适配。

摄像机标定工作是在docker外进行，所以下述指令都是在docker外环境下执行。
使用JetPack对TX2刷机后，TX2的系统版本为ubuntu16.04，对应的ros的版本为kinetic。


#### 安装rosdep
```bash
$ pip install rosdep
```

可能的错误：

"pip: command not found"

解决方法：

参考网址[https://pip.pypa.io/en/stable/installing/](https://pip.pypa.io/en/stable/installing/)安装pip


#### 安装ros
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
--recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-kinetic-ros-base
```

可能的错误：
"the following packages have unmet dependencies"


解决方法：
```bash
$ sudo vim /eta/apt/source.list
取消universe，xenial-backports，xenial-security源站配置的注释后保存退出
$ sudo apt-get update
```

可能会出现`python-rosdistro-modules`无法安装的问题，提示执行`apt-get -f install`
但是执行 -f install 依然会报错`rosdistro-modules0.7-1-all.deb has premature`.
这是dpkg的版本不兼容的问题。

解决方法：
```bash
$ sudo rm /etc/apt/sources.list.d/ros-latest.list
$ sudo apt-get -f install
$ sudo apt-get upgrade
```
然后重新安装ros。


### 安装camera_calibration
（在很多教程中，直接告诉开发者执行命令`rosdep install camera_calibration`，但是可能会出现错误：
ERROR: Rosdep cannot find all required resources to answer your query。按照下面的步骤就可以了。）
```bash
$ echo 'source /opt/ros/kinetic/setup.bash' >> ~/.bashrc
$ source ~/.bashrc
$ sudo rosdep init
$ rosdep update
$ sudo apt-get install ros-kinetic-camera-calibration
$ rosdep install camera_calibration
```

在TX2上我们使用gscam启动摄像头
#### 安装gscam
```bash
$ sudo apt-get install ros-kinetic-gscam
```

然后需要定义gscam的GSCAM_CONFIG宏以设置参数，例如以标定video1为例，执行如下命令：
```bash
$ export GSCAM_CONFIG="v4l2src device=/dev/video1 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace"
```

然后可以使用如下指令启动gscam：
```bash
$ rosrun gscam gscam image_raw:=/camera/image_raw set_camera_info:=/camera/set_camera_info
```

可能的错误：
"no element "v4l2src""

解决方法：
```bash
$ sudo apt-get install gstreamer1.0-plugins-good。
```

注意事项：
启动gscam时请确保没有其他读取摄像机的进程正在运行，否则可能会出现如下的错误：
```bash
libv4l2: error setting pixformat: Device or resource busy
[FATAL] [1551076251.590808356]: Failed to PAUSE stream, check your gstreamer configuration.
[FATAL] [1551076251.590954466]: Failed to initialize gscam stream!
```
gscam正常运行的输出信息为：
```bash
[ INFO] [1551074338.728073189]: Loaded camera calibration from
[ INFO] [1551074338.812774242]: Time offset: 1551068580.535
[ INFO] [1551074339.095952896]: Publishing stream...
[ INFO] [1551074339.096722645]: Started stream.
```

gscam启动成功后，使用`rostopic list`指令能查看到如下的两个topic：
```bash
/camera/image_raw
/camera/camera_info
```
因为`rostopic`指令需要和`ros master`通讯，所以需要启动`roscore`。可以在docker环境下进入/apollo目录，执行命令`./script/roscore.sh start`启动`roscore。`

#### 启动camera_calibration
```bash
$ rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera
```
其中8x6是是方格间焦点的个数，其中0.108是每个方格的大小，单位是米。

剩余的标定步骤请参考：

[https://blog.csdn.net/weixin_43331257/article/details/82932904](https://blog.csdn.net/weixin_43331257/article/details/82932904)

[https://blog.csdn.net/junshen1314/article/details/44831033](https://blog.csdn.net/junshen1314/article/details/44831033)

[http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration)
