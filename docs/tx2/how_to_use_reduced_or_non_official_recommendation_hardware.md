本文档介绍使用简配或者非官方推荐的传感器硬件进行配置Apollo时可能遇到的问题及修改方法。


使用Apollo实现自动驾驶方案需要使用的传感器硬件包括雷达、激光雷达、摄像头、IMU&GPS，另外还需要的硬件为IPC（中控机）。

*简配是指不全部使用上述4种传感器硬件，只是用3种或者更少的硬件实现自动驾驶方案。*

关于Apollo官方推荐的硬件方案请参考：
[https://github.com/ApolloAuto/apollo/tree/master/docs/specs](https://github.com/ApolloAuto/apollo/tree/master/docs/specs)对硬件的介绍和文档[apollo_2_5_hardware_system_installation_guide_v1.md](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_5_hardware_system_installation_guide_v1.md)

*非官方推荐是指使用了其他型号的机器作为中控机，或者使用的传感器硬件为非Apollo文档介绍或未经过Apollo官方验证的型号。*

#### 备注：本文档中只介绍在连接和启用硬件设备时可能遇到的问题，对于传感器硬件的标定不做介绍。


本文档从以下几个方面展开讨论：

* 摄像头
* IMU&GPS
* 简配后的perception配置
* 简配后的安全模式



### 摄像头
首先操作步骤和正常的结果为：

1.	将摄像头连接到中控机
2.	在`/dev`文件夹下会生成`video*`的设备文件，使用指令v4l2-ctl --list-devices可以查看到摄像头的设备信息
3.	在docker环境下，使用指令`./apollo.sh build_usbcam`编译camera的启动程序
4.	在docker环境下，进入文件夹`apollo/docker/setup_host/`并执行脚本`setup_host.sh`
5.	在docker环境下，执行命令`sudo service udev restart`
6.	在`/dev`下生成`camera`文件夹，`camera`文件夹下包含了三个软链接文件`lanemark`,`obstacle`和`trafficlights`，分别指向`video*`设备文件。
7.	根据需求配置`modules/calibration/data/mkz_example/camera_params`文件夹和`modules/drivers/usb_cam/launch`文件夹下的launch文件，如摄像头分辨率等
8.	在docker环境下，启动dreamview，在`Module Cotroller`面板中启动`Camera`，`Hardware`项目栏中`CameraL`和`CameraS`状态变为OK，表示启动成功。在`Tasks`面板中的`Others`项目栏中，启动`Camera Sensor`，则面板的左上角会出现一个小窗体显示摄像头的实时图像。


可能存在的问题：

1.	步骤2中在`/dev`下没有生成`video*`文件。需要检查硬件连接和摄像头驱动，不同的摄像头硬件或者中控机可能要求安装驱动
2.	步骤3中编译失败。使用非Apollo官方推荐的中控机时可能出现的问题，需要根据情况修改`usb_cam.cpp`代码和对应`CMakeLists.tx`t文件
3.	步骤6中在`/dev`下未生成`camera`文件夹，且不存在设备软件链接文件。这是创建设备软链接时出错，查看文件：
`apollo/docker/setup_host/etc/udev/rules.d/99-webcam.rules`，该文件中配置了生成设备软链接的规则。对于非官方推荐型号的摄像头和中控机，可能需要对其中的配置进行更改。假设摄像头的设备文件为video1，则使用如下指令查看摄像头的设备属性：
`udevadm info -a -p $(udevadm info -q path -n /dev/video1)`
根据该指令的输出配置`rules`文件中需要的属性的值
4.	步骤8中无法启动成功或者显示不了图像，请检查步骤7中的配置，确保摄像头支持`yuyv`编码且分辨率配置正确

### IMU&GPS

首先操作步骤和正常的结果为：

1.	将IMU&GPU连接到中控机
2.	在`/dev`文件夹下会生成`ttyUSB*`设备文件
3.	在docker环境下，进入文件夹`apollo/docker/setup_host/`并执行脚本`setup_host.sh`
4.	在docker环境下，执行命令`sudo service udev restart`
5.	在`/dev`文件夹下会生成`novatel*`软链接文件，分别指向于`ttyUSB*`文件
6.	修改文件`modules/calibration/data/mkz_example/gnss_params/gnss_conf.pb.txt`配置GPS的基站信息
7.	在docker环境下，启动dreamview，在`Module Controller`面板中启动`GPS`，左侧`Hardware`项目栏中`GPS`状态变为OK或者WARN，表示启动成功。启动`Localization`模块，dreamview主窗口中会显示车辆在地图中的实际位置

可能存在的问题：

1.	步骤2中未生成`ttyUSB*`设备文件。请检查硬件连接和设备驱动
2.	步骤5中未生成`novatel*`软链接文件。这是创建设备软链接时出错，查看文件：`apollo/docker/setup_host/etc/udev/rules.d/99-usbtty.rules`，该文件中配置了生成设备软链接的规则。对于非官方推荐型号的摄像头和中控机，可能需要对其中的配置进行更改。假设GPS的设备文件为`ttyUSB0`，则使用如下指令查看GPS的设备属性：
`udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0)`
查看该指令的输出配置`rules`文件中需要的属性的值

### 简配后的perception配置

Apollo已经提供了简配方案的perception配置文件，可以参考文件`dag_streaming_lowcost.config`和`perception_lowcost.sh`。我们假设简配方案中读取的配置文件为`dag_streaming_lowcost.config`。则我们需要根据简配方案中对传感器的选择修改该配置文件。
备注：`dag_streaming_lowcost.config`已经是减少了激光雷达的配置方案，请参考文件`dag_streaming.config`。

如果选择不使用雷达，只使用摄像头。那么需要将`dag_streaming_lowcost.config`文件中关于雷达的配置全部注释掉。并且需要对摄像头的配置进行如下的更改：

1.	将`subnode_config`中的`CameraProcessSubnode`配置的`reserve`选项的`pb_obj`值改为1。表示camera节点会publish消息
2.	将`subnode_config`中的`LanePostProcessSubnode`配置的`reserve`选项增加`publish:2`。表示lanepost节点会publish关于`perceptionobstacle`的消息

### 简配后的安全模式

在自动驾驶模式中，monitor会实时的检测各个硬件设备是否正常，如果不正常则会进入安全模式。所以在简配的方案中，需要更改配置以使monitor不检测没有使用的传感器。请参考文件`dreamview/conf/hmi.conf`，在`modes`的配置中，我们假设使用`Standard`模式，则需要将不用传感器配置删除掉。

例如，如果不使用激光雷达，则需要删除掉如下配置：

`live_hardware: “velodyne”`

