# 模块简介
此模块主要是在PX2 的硬件环境下,在ROS环境中给end to end 系统收集传感器数据以及汽车相关状态参数。

# 安装环境
1.	Ubuntu 16.04 LTS
2.	ROS Kinetic – 建议 ros-kinetic-desktop-full.
3.	Px2 driveworks 的0.2 版本

# 编译以及安装此模块
编译和安装此模块都是通过ROS catkin的工具来完成的。

首先应该安装ROS环境，例如安装到 `~/ros` 目录下面，为了方便起见，可以将下面的代码加入到 `~/.bash_profile`，以便每次打开一个新的bash shell都能进入到ROS环境当中去以便使用catkin的编译工具:

```
	if [ -f /opt/ros/kinetic/setup.bash ]; then
       	    source /opt/ros/kinetic/setup.bash
    fi
```
然后应该建立catkin的工作空间来编译以及安装源代码。例如可以建立`~/catkin_ws`.然后：

```
	cd ~/catkin_ws
	mkdir src
	cd src
```
这样可以将源代码 gitclone 到 `~/catkin_ws/src` 目录下面。

最后进行源代码的编译。编译的路径是在 `~/catkin_ws` 下面，用 `catkin_make` 指令进行编译安装。

生成的可执行文件以及库文件都会生成到 `~/catkin_ws/devel` 目录下面。为了方便调用模块，可以在 `~/.bash_profile` 加入下面的指令，以便每次打开一个新的bash shell都能进入到模块环境当中去：

```
	source ~/catkin_ws/devel/setup.bash
```

注意：

编译的时候有些模块会报错找不到msg，可以先将msg那个子模块编译出来，然后再将其他的子模块加入到src中进行编译。


# 使用模块
为了进行测试，首先打开一个bash shell，然后开启一个ROS core:

```
	roscore
```

然后打开另外一个bash shell，启动camera的node：

```
	rosrun px2_camera_image front_camera ./ 
```

最后打开另外一个bash shell，打开可视化工具rviz观察是否有image消息发布出来。

# LICENSE
请看相应的license 文件。
