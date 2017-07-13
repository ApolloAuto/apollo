# Apollo 快速入门指南  1.0
## 目录
* [关于这个文档](#关于这个文档)
    * [文档规约](#文档规约)
* [概览](#概览)
* [车辆环境描述](#车辆环境描述)
* [硬件安装](#硬件安装)
* [软件安装](#软件安装)
    * [下载Apollo源代码](#下载Apollo源代码)
    * [设置Docker支持](#set-up-docker-support)
    * [设置Apollo发布的Docker映像（image）](#set-up-apollo-release-docker)
    * [自定义你的发布容器](#customize-your-release-container)
* [在车辆上运行示例](#run-the-demo-on-vehicle)
    * [启动本地版本Docker映像](#launch-the-local-release-env-docker-image)
    * [记录驾驶轨迹](#record-driving-trajectory)
    * [执行自动驾驶](#perform-autonomous-driving)
    * [关闭](#shut-down)
* [运行离线演示](#run-offline-demo)

# 关于这个文档

_Apollo 快速入门指南 1.0_ 提供了所有关于了解、安装以及构建Apollo的基本说明

## 文档规约

下表列出了本文档中使用的归约：

| **Icon**                            | **描述**                          |
| ----------------------------------- | ---------------------------------------- |
| **粗体**                            | 重要                                 |
| `等宽字体`                   | 代码，类型数据                        |
| _斜体_                            | 文件标题，章节和标题使用的术语 |
| ![info](images/info_icon.png)       | **Info**  包含可能有用的信息。忽略信息图标没有消极的后果 |
| ![tip](images/tip_icon.png)         | **Tip**. 包括有用的提示或可能有助于您完成任务的捷径。 |
| ![online](images/online_icon.png)   | **Online**. 提供指向特定网站的链接，您可以在其中获取更多信息 |
| ![warning](images/warning_icon.png) | **Warning**. 包含**不**能忽略的信息，或者执行某个任务或步骤时，您将面临失败风险 |

# 概览

Apollo已经开始为汽车和自主驾驶行业的合作伙伴提供开放，全面，可靠的软件平台。合作伙伴可以使用Apollo软件平台和通过Apollo认证的参考硬件模板来定制自己的自主车辆研发。

Apollo 1.0, 也被称为 _Automatic GPS Waypoint Following(自动GPS跟随)_, 使用在封闭的区域内，如测试轨道或停车场。它可以准确地以人类驾驶员在封闭的平坦区域的速度复现一个驾驶轨迹。

在这个开发阶段, Apollo 1.0 **无法** 察觉到邻近的障碍物, **不要**在公共道路或没有GPS信号的区域行驶。

# 车辆环境描述

The Lincoln MKZ, enhanced by Autonomous Stuff, 为用户提供了一个无障碍的自动车辆平台。该平台为用户提供了一整套硬件和软件解决方案。

用户可以直接获得车辆某些模块控制权限，如档位，速度和指示灯。平台已经为转向，刹车，加速和档位创建了接口，为开发人员提供了可使用的用户界面。

包含的其他功能:

- 电源分配器终端
- 集成PC与ROS预安装和配置
- 线控驱动的紧急停止系统
- 以太网和USB连接 (to PC)

# 硬件安装

请参考 [Apollo 1.0 Hardware and System Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_hardware_system_installation_guide.md)
中的步骤来安装硬件组件以及系统软件

# 软件安装

本节包括：

- 下载Apollo发行包
- 设置Docker支持
- 自定义你的发布容器

在开始之前，请确保您已经按照
[Apollo 1.0 Hardware and System Installation Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_hardware_system_installation_guide.md)中的步骤
安装了Ubuntu Linux 14.04.3和Apollo Kernel。
## 下载Apollo源代码

1. 从[github source](https://github.com/ApolloAuto/apollo/)下载Apollo的源代码：

```
git clone git@github.com:ApolloAuto/apollo.git
cd apollo

```

2. 参考以下命令设置环境变量 `APOLLO_HOME`:

```
echo "export APOLLO_HOME=$(pwd)" >> ~/.bashrc && source ~/.bashrc
```

3. 在一个新的终端或者已有的终端中输入`source ~/.bashrc`


![tip](images/tip_icon.png) 在以下部分中，假设Apollo目录位于 `$APOLLO_HOME`.

## 设置Docker支持

Docker容器是设置Apollo构建环境的最简单方法。

有关更多信息，请参阅Docker详细教程 [here](https://docs.docker.com/).

1. 运行以下命令来安装Docker：


```
cd $APOLLO_HOME
bash docker/scripts/install_docker.sh
```

2. 脚本完成后，注销并重新登录系统以启用Docker。


3. （可选）如果您已经安装了Docker（在安装Apollo内核之前），请在其中添加以下行 `/etc/default/docker`:

```
DOCKER_OPTS = "-s overlay"
```

## 自定义你的发布容器

1. 通过运行以下命令下载并启动Apollo 发布的 Docker映像：

```
cd $APOLLO_HOME
bash docker/scripts/release_start.sh
```

2. 通过运行以下命令登录Apollo 发布的 Docker映像：

```
bash docker/scripts/release_into.sh
```

3. 通过修改文件中的以下行来设置全球导航卫星系统（GNSS）驱动程序的区域编号 `./ros/share/gnss_driver/launch/gnss_driver.launch`.

```
<arg name="proj4_text" default="+proj=utm +zone=10 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs " />
```
你只需修改上面一行的`+zone=10`的值即可。请参考
[Apollo's Coordinate System](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/coordination.pdf) 找到您当地的区号。例如，如果你在北京，中国，你必须设置`+zone=50`。

5. 通过修改文件，为GNSS驱动程序设置实时运动（RTK）基站：
 `./ros/share/gnss_driver/conf/gnss_conf_mkz.txt`

   有关典型的RTK设置，请参阅以下示例：

```
rtk_from {
	format: RTCM_V3
		ntrip {
		address: <provide your own value>
		port: <provide your own value>
		mount_point: <provide your own value>
		user: <provide your own username>
		password: <provide your own password>
		timeout_s: <provide your own value, e.g., 5>
	}
}
rtk_to {
	format: RTCM_V3
	serial {
		device: <provide your own value, e.g., "/dev/ttyUSB1">
		baud_rate: <provide your own value, e.g., 115200>
	}
}
```

The `rtk_from` is  used for RTK base station information.

The `rtk_to` is used to send the RTK differential data to the receiver.

6. Add ESD CAN Support

   Please refer to [ESD CAN README](https://github.com/ApolloAuto/apollo/blob/master/third_party/can_card_library/esd_can/README.md)
   to setup the ESD CAN library.

7. Follow these steps to persist your local changes:

```
# RUN OUT OF DOCKER ENV
# commit your docker local changes to local docker image.
exit # exit from docker environment
cd $APOLLO_HOME
bash docker/scripts/release_commit.sh
```

# Run Demo on Vehicle

This section provides the instructions to run the Apollo 1.0 Demo on Vehicle.
1. Set up the hardware:

- Power-on the platform vehicle.
- Power-on the Industrial PC (IPC).
    ![](images/ipc_power_on.png)
- Power-on the modem by pressing and holding the power button until the lights turn on.
- Set up the network configuration for the IPC: static IP (for example, 192.168.10.6), subnet mask (for example, 255.255.255.0), and gateway (for example, 192.168.10.1)
   - Configurate your DNS server IP (for example, 8.8.8.8).
   - Use a tablet to access **Settings** and connect to MKZ wifi:

   ![](images/ipad_config_wifi.png)

2. Start the HMI in Docker **using Chrome only**:

   ![warning](images/warning_icon.png)**Warning:** Make sure that you are not starting HMI from two Docker containers concurrently.

## Launch the Local release env Docker Image

Run the following commands:
```
cd $APOLLO_HOME
bash docker/scripts/release_start.sh local
```
When Docker starts, it creates a port mapping, which maps the Docker internal port 8887 to the host port 8887. You can then visit the HMI web service in your host machine browser:

Open the Chrome browser and start the Apollo HMI by going to **192.168.10.6:8887**.
 ![](images/start_hmi.png)

## Record the Driving Trajectory

Follow these steps to record the driving trajectory:

1. In the Apollo HMI, under Quick Record, click **Setup** to start all modules and perform the hardware health check.
   ![](images/hmi_record_setup.png)
2. If the hardware health check passes, click the **Start** button to start to record the driver trajectory.
   ![](images/hmi_record_start.png)
3. After arriving at a destination, click the **Stop** button to stop recording.
   ![](images/hmi_record_stop.png)
4. If you want to record a *different* trajectory, click the **New** button to initiate recording again.
   ![](images/hmi_record_reset.png)

## Perform Autonomous Driving

Follow these steps to perform autonomous driving:

1. In the Apollo HMI, under Quick Play, click **Setup** to start all modules and perform a hardware health check.
   ![](images/hmi_play_setup.png)

2. If the vehicle successfully passes the Setup step, it is ready to enter the Autonomous mode. **MAKE SURE DRIVER IS READY!**  Click the **Start** button to start the autonomous driving.

   ![](images/hmi_play_start.png)

3. After arriving at your destination,  click the **Stop** button to stop replaying the recorded trajectory.
   ![](images/hmi_play_stop.png)

## Shut Down

1. Shut down the system from a terminal:
    ```sudo shutdown now```

2. Power-off the IPC (locate the icon on the top right of the desktop to click **Shut Down**).

3. Turn off the modem by pressing and holding the power button until the lights turn off.

4. Turn off the car.

# Run Offline Demo
Refer to [Offline Demo Guide](https://github.com/ApolloAuto/apollo/blob/master/docs/demo_guide/README.md)
