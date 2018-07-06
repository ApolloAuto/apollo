# Apollo 1.0 Hardware and System Installation Guide

* [关于本篇指南](#关于本篇指南)
    * [文档编写规则](#文档编写规则)
* [引言](#引言)
    * [文档说明](#文档说明)
* [核心硬件部件](#核心硬件部件)
    * [额外的部件](#额外的部件)
    * [车载计算机系统-IPC](#车载计算机系统-ipc)
        * [IPC的配置](#ipc的配置)
        * [IPC前后视图](#ipc前后视图)
    * [控制器局域网络（CAN）卡](#控制器局域网络（-can）卡)
    * [全球定位系统(GPS)和惯性测量装置(IMU)](#全球定位系统(gps)和惯性测量装置(imu))
        * [选项1: NovAtel SPAN-IGM-A1](#选项1:nocatel-span--igm--a1)
        * [选项2: NovAtel SPAN ProPak6 和 NovAtel IMU-IGM-A1](#选项2:novatel-span-propak6和-imu--igm--a1)
    * [GPS接收器和天线](#gps接收器和天线)
* [安装任务概览](#安装任务概览)
* [安装任务步骤](#安装任务步骤)
    * [上车前的准备工作](#上车前的准备工作)
        * [IPC的准备工作](#ipc的准备工作)
        * [为IPC安装软件](#为ipc安装软件)
    * [上车安装](#上车安装)
        * [前提条件](#前提条件)
        * [主要部件安装图](#主要部件安装图)
        * [安装GPS的接收器和天线](#安装gps的接收器和天线)
        * [安装IPC](#安装ipc)
        * [配置GPS和IMU](#配置gps和imu)
* [配置网络](#配置网络)
    * [一些建议](#一些建议)
* [其它安装任务](#其它安装任务)
* [下一步](#下一步)


# 关于本篇指南

本篇指南提供了所有安装硬件部分和Apollo项目所需的教程。系统安装信息包括下载和安装Apollo Linux内核的过程。

## 文档编写规则

下表列出了本文使用的编写规则：

| **图标**                            | **描述**                          |
| ----------------------------------- | ---------------------------------------- |
| **加粗**                            | 强调。                                 |
| `Mono-space 字体`                   | 代码, 类型数据。                         |
| _斜体_                            | 文件、段落和标题中术语的用法。   |
| ![信息](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/info_icon.png)       | **信息**  提供了可能有用的信息。忽略此信息可能会产生不可预知的后果。 |
| ![提示](https://github.com/ireeX/apollo/blob/master/docs/quickstart/images/tip_icon.png)         | **提醒** 包含有用的提示或者可以帮助你完成安装的快捷步骤。 |
| ![在线](https://github.com/ireeX/apollo/blob/master/docs/quickstart/images/online_icon.png)   | **在线** 提供指向特定网站的链接，您可以在其中获取更多信息。 |
| ![警告](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/warning_icon.png) | **警告** 包含 **不能** 被忽略的内容，如果忽略，当前安装步骤可能会失败。 |

# 引言

**Apollo**项目旨在为汽车和自动驾驶行业的合作伙伴提供开放，完整和可靠的软件平台。该项目的目的是使这些企业能够开发基于Apollo软件栈的自动驾驶系统。

## 文档说明

以下文档适用于Apollo 1.0:

- ***<u>[Apollo Hardware and System Installation Guide]</u>***  ─ 提供用于安装车辆的硬件部件和系统软件的教程:

    - **车辆**:

      - 工业用计算机 (IPC)
      - 全球定位系统 (GPS)
      - 惯性测量单元 (IMU)
      - 控制器局域网络 (CAN) 卡
      - 硬盘
      - GPS 天线
      - GPS 接收器

    - **软件**:
      - Ubuntu Linux
      - Apollo Linux Kernel

- ***<u>[Apollo Quick Start Guide]</u>*** ─ 文档和蓝图的组合提供了完整的端到端教程。本文还提供了一些其它文档链接描述了如何将一辆普通汽车改装成一辆自动驾驶车辆。


# 核心硬件部件

需要安装的核心硬件部件包括:

- 车载计算机系统 ─ Neousys Nuvo-5095GC
- CAN卡 ─ ESD CAN-PCIe/402-1
- GPS和IMU ─
  可选项如下:
  - NovAtel SPN-IGM-A1
  - NovAtel SPAN® ProPak6™ and NovAtel IMU-IGM-A1

## 额外的部件

你需要额外的部件支持额外的任务：
- 联网需要4G路由器
- 在车上debug需要显示器，键盘和鼠标
- VGA连接器，DVI线（可选）
- 苹果 iPad Pro: 9.7寸, Wi-Fi (可选）

核心硬件部件的功能将在后续章节中介绍。

## 车载计算机系统-IPC

车载计算机系统是用于自动驾驶车辆的工业PC（IPC），并使用由第六代Intel Skylake core i7-6700 CPU强力驱动的 **NeousysNuvo-5095GC**。

Neousys Nuvo-5095GC 是自动驾驶系统（ADS）的中心单元。

### IPC的配置

IPC配置如下：

- 32GB DDR4 RAM
- MezIO-V20-EP module (具有车用点火装置)
- PO-160W-OW 160W 交流、直流电源适配器
- CSM2 module (x16 PCIe expansion Gen3 8-lane cassette)

### IPC前后视图

安装了GPU的IPC前后视图如下:

Nuvo-5095GC的前视图:

![ipc_front](https://github.com/ireeX/apollo/blob/master/docs/quickstart/images/ipc_front.png)

Nuvo-5095GC的后视图:

![ipc_back](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/ipc_back.png)

更多关于Nuvo-5095GC的信息，请参考:

![online](https://github.com/ireeX/apollo/blob/master/docs/quickstart/images/online_icon.png)
Neousys Nuvo-5095GC 的产品页:

[http://www.neousys-tech.com/en/product/application/gpu-computing/product/nuvo-5095gc-gpu-computer](http://www.neousys-tech.com/en/product/application/gpu-computing/product/nuvo-5095gc-gpu-computer)
![online](https://github.com/ireeX/apollo/blob/master/docs/quickstart/images/online_icon.png)


Neousys Nuvo-5095GC-手册:

[http://www.neousys-tech.com/en/support/resources/category/162-manual](http://www.neousys-tech.com/en/support/resources/category/162-manual)

## 控制器局域网络（CAN）卡

IPC中使用的CAN 卡型号是 **ESD** **CAN-PCIe/402**.

![can_card](https://github.com/ireeX/apollo/blob/master/docs/quickstart/images/can_card.png)

更多 CAN-PCIe/402的信息，请参考:

![online](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/online_icon.png)    ESD CAN-PCIe/402 产品页:

[https://esd.eu/en/products/can-pcie402](https://esd.eu/en/products/can-pcie402)



##全球定位系统(GPS)和惯性测量装置(IMU)

有 **两种** GPS-IMU **选择** ，您只需根据您的需求进行选择:

- **选项 1: NovAtel SPAN-IGM-A1**
- **选项 2: NovAtel SPAN® ProPak6™ 和 NovAtel IMU-IGM-A1**

###选项1: NovAtel SPAN-IGM-A1

NovAtel SPAN-IGM-A1 是一个集成的，单盒的解决方案，提供紧密耦合的全球导航卫星系统（GNSS）定位和具有NovAtel OEM615接收机的惯性导航功能。 

![novatel_imu](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/Novatel_imu.png)

更多关于NovAtel SPAN-IGM-A1的信息，请参考:

 ![online](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/online_icon.png) NovAtel SPAN-IGM-A1 产品页:

[https://www.novatel.com/products/span-gnss-inertial-systems/span-combined-systems/span-igm-a1/](https://www.novatel.com/products/span-gnss-inertial-systems/span-combined-systems/span-igm-a1/)

###选项2: NovAtel SPAN ProPak6和NovAtel IMU-IGM-A1

NovAtel ProPak6是独立的GNSS接收机，它与NovAtel提供的独立IMU（本例中为NovAtel IMU-IGM-A1）相融合以提供定位。

ProPak6提供由NovAtel生产的最新最先进的外壳产品。

IMU-IGM-A1是与支持SPAN的GNSS接收器（如SPAN ProPak6）配对的IMU。 

![novatel_pp6](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/Novatel_pp6.png)

更多关于NovAtel SPAN ProPak6 和 the IMU-IGM-A1的信息，请参考:

  ![online](https://github.com/ireeX/apollo/blob/master/docs/quickstart/images/online_icon.png) NovAtel ProPak6 安装与操作手册:

[https://www.novatel.com/assets/Documents/Manuals/OM-20000148.pdf](https://www.novatel.com/assets/Documents/Manuals/OM-20000148.pdf)

  ![online](https://github.com/ireeX/apollo/blob/master/docs/quickstart/images/online_icon.png)NovAtel IMU-IGM-A1 产品页:

[https://www.novatel.com/products/span-gnss-inertial-systems/span-imus/span-mems-imus/imu-igm-a1/#overview](https://www.novatel.com/products/span-gnss-inertial-systems/span-imus/span-mems-imus/imu-igm-a1/#overview)

## GPS接收器和天线

GPS-IMU组件的GPS接收器、天线使用的是 **NovAtel GPS-703-GGG-HV**。

**注意:** GPS NovAtelGPS-703-GGG-HV与上文中提到的两个GPS-IMU选项的任一型号配合使用。 

![gps_receiver](https://github.com/ireeX/apollo/blob/master/docs/quickstart/images/gps_receiver.png)

更多关于 NovAtel GPS-703-GGG-HV的信息，请参考:

  ![online](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/online_icon.png) NovAtel GPS-703-GGG-HV 产品页:

[https://www.novatel.com/products/gnss-antennas/high-performance-gnss-antennas/gps-703-ggg-hv/](https://www.novatel.com/products/gnss-antennas/high-performance-gnss-antennas/gps-703-ggg-hv/)



# 安装任务概览

安装硬件和软件组件涉及以下任务：

**上车前的准备工作**
1. 准备IPC：
    a. 检查图形处理单元（GPU）磁带，以确定是否需要卸下GPU卡（如果已预安装）
    b. 在将卡插入插槽之前，首先重新定位CAN卡端接跳线，准备并安装控制器局域网（CAN）卡。

2. IPC（如果未预装）安装硬盘，推荐安装固态硬盘； 如果你愿意，也可以更换预装的硬盘； 

    **推荐** :
    - 为了更好的可靠性，安装固态硬盘；
    - 如果需要收集驾驶数据，需要使用大容量硬盘；

3. 准备IPC加电：
    a. 将电源线连接到电源连接器（接线端子）
    b. 将显示器，以太网，键盘和鼠标连接到IPC
    c. 将IPC连接到电源

4. 在IPC安装软件（需要部分Linux经验）：
    a. 安装Ubuntu Linux.
    b. 安装Apollo Linux kernel.

**上车安装:**

- 确保所有在前提条件中列出的对车辆的修改，都已执行。

- 安装主要的组件：
    - GPS 天线
    - IPC
    - GPS接收器

安装所有硬件和软件组件的实际步骤详见安装任务步骤。

# 安装任务步骤

该部分包含：
- 关键组件的安装
- 车辆硬件的安装

## 上车前的准备工作

有如下任务:

- 准备IPC：
    - 安装CAN卡
    - 安装或者替换硬盘
    - 准备为IPC供电

- 为IPC安装软件：
    - Ubuntu Linux
    - Apollo Kernel

### IPC的准备工作

有如下步骤：

1. 在IPC中，检查GPU卡槽，如果有预先装好的GPU卡，需要先把它移除：

    a. 在IPC底部找到四个固定GPU卡槽的螺丝并拧下来（如图中紫框内所示）:

    ![ipc_gpu_cassette](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/ipc_gpu_cassette.png)

    b. 把GPU卡槽从IPC上取出:

    ![ipc_gpu_cassette_remove](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/ipc_gpu_cassette_remove.png)

    c. 把GPU卡槽从IPC上取下来：拧下底部其它三个螺丝以打开盒盖（如图中紫框所示）：

    ![ipc_gpu_cassette_unscrew](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/ipc_gpu_cassette_unscrew.png)

    d. 取下GPU卡（如果安装了）：

    ![ipc_gpu_remove](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/ipc_gpu_remove.png)




2. 准备并安装CAN卡

    a. 通过从其默认位置移除红色跳线帽（以黄色圆圈显示）并将其放置在其终止位置，设置CAN卡端接跳线：
    ![prepare_can_card](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/prepare_can_card.png)

    **![warning](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/warning_icon.png)WARNING**: 如果端接跳线设置不正确，CAN卡将无法正常工作。

    b. 将CAN卡插入IPC的插槽:

    ![insert_can_card](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/insert_can_card.png)

    c. 把GPU卡槽重新装回IPC：

    ![reinstall_gpu_cassette](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/reinstall_gpu_cassette.png)

3. 安装或替换硬盘

    IPC（如果未预装）安装硬盘，您需要安装1至2个2.5”的SSD或硬盘。推荐安装固态硬盘； 如果您愿意，也可以更换预装的硬盘； 

    ![tip_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/tip_icon.png) 为了更好的可靠性，强烈推荐您安装SSD。如果您需要手机数据，建议您使用大容量硬盘。

    安装硬盘：

    a. 拧下三个螺丝（如图紫框内所示），打开盖子

    ![hard_drive_unscrew](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/hard_drive_unscrew.png)

    b. 把硬盘装上（图内为Intel SSD）:

    ![tip_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/tip_icon.png) 观察把硬盘装进盒子里的方式，串行高级技术附件（SATA）和电源连接器应放置在面向有 **两个**螺孔的那端。

    ![hard_drive](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/hard_drive.png)

       现在硬盘就装好了：

    ![hard_drive_connect](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/hard_drive_connect.png)

    c. 把SSD重新装入IPC中：

    ![reinstall_ssd](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/reinstall_ssd.png)

4. 准备IPC启动:

    a. 将电源线连接到IPC的电源连接器（接线端子）：

    ![warning_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/warning_icon.png)**WARNING**: 确保电源线的正极（红色用 **R**表示）和负极（黑色用 **B**表示）正确的插入电源端子块上的插孔中。

    ![ipc_power_RB](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/ipc_power_RB.png)

    b. 连接显示器，以太网线，键盘和鼠标到IPC上：

    ![ipc_power_back](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/ipc_power_back.png)

    ​

    ![tip_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/tip_icon.png)出于以下原因，建议您为显示器使用视频图形阵列（VGA）连接器：

- 如果在IPC启动时没有看到任何屏幕显示，请切换到VGA输入。 即使没有连接显示器，Neousys Nuvo-5095GC IPC也 **总是**输出到VGA端口。 因此，Linux安装程序可能“选择”输出到 **VGA端口**而不是DVI端口。
- 如果在使用双显示器设置时在安装过程中没有看到对话窗口，请尝试在VGA和DVI之间切换以找到它。 Linux安装程序可能会检测到两个监视器并同时使用它们。

为了获得更好的显示质量，您可以选择：

- 使用DVI线或带DVI-HMI适配器的高清晰度多媒体接口（HMI）连接到另一台显示器
- 使用同一台显示器上的DVI / HDMI端口

    c. 连接电源:

    ![ipc_connect_power](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/ipc_connect_power.png)




### 为IPC安装软件

这部分主要描述以下的安装步骤：

- Ubuntu Linux
- Apollo 内核

![tip_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/tip_icon.png)您最好具有使用Linux成功安装软件的经验，如果这是您的第一次安装，有可能会失败。

#### 安装Ubuntu Linux
步骤如下：

1. 创建一个可以引导启动的Ubantu Linux USB闪存驱动器：

    下载Ubuntu（或Xubuntu等分支版本），并按照在线说明创建可引导启动的USB闪存驱动器。

![tip_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/tip_icon.png) 推荐使用 **Ubuntu 14.04.3**.

![tip_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/tip_icon.png)开机按 F2 进入 BIOS 设置菜单，建议禁用BIOS中的快速启动和静默启动，以便捕捉引导启动过程中的问题。 建议您在BIOS中禁用“快速启动”和“安静启动”，以便了解启动过程中遇到的问题。

获取更多Ubuntu信息，可访问: 
![online_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/online_icon.png) Ubuntu 桌面站点:

[https://www.ubuntu.com/desktop](https://www.ubuntu.com/desktop)

2. 安装 Ubuntu Linux:

    a.  将Ubuntu安装驱动器插入USB端口并启动IPC。
    b.  按照屏幕上的说明安装Linux。

3. 执行软件更新与安装:
    a.  安装完成，重启进入Linux。
    b.  执行软件更新器（Software Updater）更新最新软件包，或在终端执行以下命令完成更新。
    ```shell
    sudo apt-get update; sudo apt-get upgrade
    ```

    c.  打开终端，输入以下命令，安装Linux 4.4 内核：

```shell
sudo apt-get install linux-generic-lts-xenial
```
![tip_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/tip_icon.png)IPC必须接入网络以便更新与安装软件，所以请确认网线插入并连接，如果连接网络没有使用动态分配（DHCP），需要更改网络配置。


#### 安装Apollo内核

车上运行Apollo需要 [Apollo Kernel](https://github.com/ApolloAuto/apollo-kernel). 强烈建议安装预编译内核。

##### 使用预编译的 Apollo 内核

你可以依照如下步骤获取、安装预编译的内核。

1. 从realease文件夹下载发布的包
```
https://github.com/ApolloAuto/apollo-kernel/releases
```
2. 安装内核
After having the release package downloaded:
```
tar zxvf linux-4.4.32-apollo-1.0.0.tar.gz
cd install
sudo bash install_kernel.sh
```
3. 使用 `reboot`命令重启系统；
4. 根据[ESDCAN-README.md](https://github.com/ApolloAuto/apollo-kernel/blob/master/linux/ESDCAN-README.md)编译ESD CAN驱动器源代码

##### 构建你自己的内核
如果内核被改动过，或预编译内核不是你最佳的平台，你可以通过如下方法构建你自己的内核：

1. 从代码仓库克隆代码：
```
git clone https://github.com/ApolloAuto/apollo-kernel.git
cd apollo-kernel
```
2. 根据 [ESDCAN-README.md](https://github.com/ApolloAuto/apollo-kernel/blob/master/linux/ESDCAN-README.md)添加 ESD CAN 驱动源代码。

3. 按照如下指令编译：
```
bash build.sh
```
4. 使用同样的方式安装内核。

##### 可选：测试ESD CAN设备端
在重启有新内核的IPC以后：

a. 使用以下指令创建CAN硬件节点：

```shell
cd /dev; sudo mknod –-mode=a+rw can0 c 52 0
```
b. 使用从ESD Electronics获取到得的ESD CAN软件包的一部分的测试程序来测试CAN设备节点。


至此，IPC就可以被装载到车辆上了。

## 上车安装

执行以下任务：

- 根据先决条件列表中的所述，对车辆进行必要的修改
- 安装主要的组件：Install the major components:
    - GPS 天线
    - IPC
    - GPS 接收器


### 前提条件

**![warning_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/warning_icon.png)WARNING**: 在将主要部件（GPS天线，IPC和GPS接收器）安装在车辆之前，必须按照先决条件列表所述执行必要修改。 列表中所述强制性更改的部分，不属于本文档的范围。

安装的前提条件如下：

- 车辆必须由专业服务公司修改为“线控”技术。 此外，必须在要安装IPC的中继线上提供CAN接口连接。
- 必须在后备箱中安装电源插板，为IPC和GPS-IMU提供电源。电源插板还需要服务于车上的其他硬件，比如4G的路由器。电源插板应连接到车辆的电源系统。
- 必须安装定制的机架，将GPS-IMU天线安装在车辆的顶部。
- 必须安装定制的机架，以便将GPS-IMU安装在后背箱中。
- 必须将4G LTE路由器安装在后备箱中才能为IPC提供Internet访问。路由器必须具有内置Wi-Fi接入点（AP）功能，以连接到其他设备（如iPad），以与自主驾驶（AD）系统相连接。例如，用户将能够使用移动设备来启动AD模式或监视AD状态。

### 主要部件安装图

以下两图显示车辆上应安装三个主要组件（GPS天线，IPC，GPS接收机和LiDAR）的位置： 示例图：



![major_component_side_view](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/major_component_side_view.png)

车辆与后备箱侧视图 

![major_component_rear_view](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/major_component_rear_view.png)

车辆与后备箱后视图 

### 安装GPS的接收器与天线

以下组件 **二选一**:

- **选项 1:** GPS-IMU: **NovAtel SPAN-IGM-A1**
- **选项 2:** GPS-IMU: **NovAtel SPAN® ProPak6™ and NovAtel IMU-IGM-A1**

#### 选项1：安装NovAtel SPAN-IGM-A1

安装说明描述了安装，连接和采取GPS-IMU NovAtel SPAN-IGM-A1的杠杆臂测量的过程。

##### 安装

可以将GPS-IMU NovAtel SPAN-IGM-A1放置在车辆的大部分地方，但建议您遵循以下建议：

- 将NovAtel SPAN-IGM-A1放置并固定在行李箱内，Y轴指向前方。
- 将NovAtel GPS-703-GGG-HV天线安装在位于车辆顶部的视野范围内。

##### 接线

您必须连接的两根电缆：

- 天线电缆 - 将GNSS天线连接到SPAN-IGM-A1的天线端口
- 主电缆：
     - 将其15针端连接到SPAN-IGM-A1
     - 将其电源线连接到10至30V直流电源
     - 将其串行端口连接到IPC。如果电源来自车载电池，请添加辅助电池（推荐）。


![imu_main_cable_connection](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/imu_main_cable_connection.png)

主电缆连接

更多信息参见 *SPAN-IGM™ 快速入门指南*, 第三页, 详细图:

![online_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/online_icon.png)SPAN-IGM™ 快速入门指南

[http://www.novatel.com/assets/Documents/Manuals/GM-14915114.pdf](http://www.novatel.com/assets/Documents/Manuals/GM-14915114.pdf)

##### 采取杠杆臂测量


当SPAN-IGM-A1和GPS天线就位时，必须测量从SPAN-IGM-A1到GPS天线的距离。 该距离标识为：X偏移，Y偏移和Z偏移。

偏移误差必须在1厘米以内才能实现高精度。 有关详细信息，请参阅 * SPAN-IGM™快速入门指南*，第5页，详细图。

更多有关SPAN-IGM-A1的信息参见:

![online_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/online_icon.png)SPAN-IGM™ 用户手册:

[http://www.novatel.com/assets/Documents/Manuals/OM-20000141.pdf](http://www.novatel.com/assets/Documents/Manuals/OM-20000141.pdf)



#### 选项2：NovAtel SPAN® ProPak6™ 和 NovAtel IMU-IGM-A1

安装说明描述了安装，连接和采取GPS NovAtelSPAN®ProPak6™**和** NovAtel IMU-IGM-A1的杠杆臂测量的步骤。

##### 组件

安装所需的组件包括：

- NovAtel GPS SPAN ProPak6

- NovAtel IMU-IGM-A1

- NovAtel GPS-703-GGG-HV天线

- NovAtel GPS-C006电缆（将天线连接到GPS）

- NovAtel 01019014主电缆（将GPS连接到IPC的串行端口）

- 数据传输单元（DTU） - 类似于4G路由器

- 磁性适配器（用于天线和DTU）

- DB9直通电缆

##### 安装

你可以将 ProPak6 和 IMU 放置在车辆以下建议的位置：

- 将ProPak6和IMU并排固定在行李箱内，Y轴指向前方。
- 将NovAtel GPS-703-GGG-HV天线安装在车辆顶部或行李箱盖顶部，如图所示：

![gps_receiver_on_car](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/gps_receiver_on_car.png)

- 使用磁性适配器将天线紧固到行李箱盖上。
- 通过打开主干并将电缆放置在行李箱盖和车身之间的空间中，将天线电缆安装在主干箱中。

##### 接线

按照以下步骤将ProPak6 GNSS接收器和IMU连接到Apollo系统：

1. 使用IMU-IGM-A1附带的分接电缆连接IMU主端口和ProPak6 COM3/IMU端口。
2. 使用USB-MicroUSB转换线，连接IPC的USB端口和ProPak6的MicroUSB端口。
3. 将IMU-IGM-A1分离电缆的另一端连接到车辆电源。
4. 将GNSS天线连接到Propak6。
5. 连接Propak6电源线。

![wiring](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/wiring.png)

更多有关 NovAtel SPAN ProPak6的信息, 参见:

![online_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/online_icon.png)NovAtel ProPak6安装操作手册:

[https://www.novatel.com/assets/Documents/Manuals/OM-20000148.pdf](https://www.novatel.com/assets/Documents/Manuals/OM-20000148.pdf)



### 安装IPC

步骤如下：

1.   用电源线把IPC连接到车辆电源上：

将其电源连接器作为一端![ipc_power_RB](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/ipc_power_RB.png)，另一端连接至车上的电源板面（参见前提条件部分）。

2.   把5059GC车载计算系统放在车上（推荐）。

例如，Apollo 1.0使用4x4自攻螺钉将5059GC固定在后备箱里。 ![IPC-bolt_down-936x720](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/IPC-bolt_down-936x720.jpg)

3. 安装IPC，使其正面和背面（所有端口都在背面）面向后备箱的右侧（乘客那侧）和左侧（驾驶员那侧）。

这样放置更容易连接所有电缆线。

如需更多信息，请参考

![online_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/online_icon.png)Neousys Nuvo-5095GC – 手册:

[http://www.neousys-tech.com/en/support/resources/category/162-manual](http://www.neousys-tech.com/en/support/resources/category/162-manual)

4. 连接所有线，包括:

- 电源线

- CAN线

- 用以太网线将4G路由器连接到IPC

- 将GPS接收器连接到IPC

- （可选）显示器，键盘，鼠标


a.   将电源线连接至IPC（如图所示）:

![IPC-power-842x636](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/IPC-power-842x636.jpg)

b. 将电源线的另一端连至车辆电池（如图所示）：

![IPC-power-cable](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/IPC-power-cable.jpg)

c. 将DB9线连接到IPC，使它可以与CAN交流：

![DB9_cable](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/DB9_cable.png)

d. 连接:

- 从4G路由器到IPC的以太网电缆（标签上写着“Router”）

- 从GPS接收器到IPC的线缆（标签上写着“GPSIMU”）

- （可选)显示器（标签上写着“Monitor”）

![ipc_other_cables](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/ipc_other_cables.png)



#### 杠杆臂测量

步骤如下:

1.    在接受测量之前，打开IPC。


2.    当IMU和GPS天线就位时，必须测量从IMU到GPS天线的距离。距离测量应为：X偏移，yoffset，和Z偏移。

偏移误差必须在一厘米以内，以达到定位和定位的高精度。

更多信息，参见：

![online_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/online_icon.png)NovAtel ProPak6 安装操作手册:

[https://www.novatel.com/assets/Documents/Manuals/OM-20000148.pdf](https://www.novatel.com/assets/Documents/Manuals/OM-20000148.pdf)

![online_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/online_icon.png)NovAtel SPAN-IGM-A1 产品页:

[https://www.novatel.com/products/span-gnss-inertial-systems/span-combined-systems/span-igm-a1/](https://www.novatel.com/products/span-gnss-inertial-systems/span-combined-systems/span-igm-a1/)

### 配置GPS和IMU

GPS 和IMU 配置如下：

```
WIFICONFIGSTATE OFF
UNLOGALLTHISPORT
SETIMUTOANTOFFSET0.00 1.10866 1.14165 0.05 0.05 0.08
SETINSOFFSET0 0 0
LOGCOM2 GPRMC ONTIME 1.0 0.25
EVENTOUTCONTROLMARK2 ENABLE POSITIVE 999999990 10
EVENTOUTCONTROLMARK1 ENABLE POSITIVE 500000000 500000000
LOGNCOM1 GPGGA ONTIME 1.0

logbestgnssposb ontime 0.5
logbestgnssvelb ontime 0.5
logbestposb ontime 0.5
logINSPVASB ontime 0.01
logCORRIMUDATASB ontime 0.01
logINSCOVSB ontime 1
logmark1pvab onnew

logimutoantoffsetsb once
logvehiclebodyrotationb onchanged

SAVECONFIG
```

ProPak6配置如下:

```
WIFICONFIG STATE OFF
CONNECTIMU COM3 IMU_ADIS16488
INSCOMMAND ENABLE
SETIMUORIENTATION 5
ALIGNMENTMODE AUTOMATIC  
SETIMUTOANTOFFSET 0.00 1.10866 1.14165 0.05 0.05 0.08
VEHICLEBODYROTATION 0 0 0
 
COM COM1 9600 N 8 1 N OFF OFF
COM COM2 9600 N 8 1 N OFF OFF
INTERFACEMODE COM1 NOVATEL NOVATEL OFF
LOG COM2 GPRMC ONTIME 1 0.25
PPSCONTROL ENABLE POSITIVE 1.0 10000
MARKCONTROL MARK1 ENABLE POSITIVE
EVENTINCONTROL MARK1 ENABLE POSITIVE 0 2
 
interfacemode usb2 rtcmv3 none off
rtksource auto any
psrdiffsource auto any
 
SAVECONFIG
```

**![warning_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/warning_icon.png) 警告:** 基于真实的测量值（GPS天线、IMU的偏移量）修改 **<u>SETIMUTOANTOFFSE</u>T** 行

例如：

```
SETIMUTOANTOFFSET -0.05 0.5 0.8 0.05 0.05 0.08
```

# 配置网络

本节提供了一种建立网络的建议。

运行阿波罗软件的IPC必须访问互联网获取实时运动学（RTK）数据，以便精确定位。移动设备还需要连接到IPC来运行Apollo软件。

## 推荐配置

建议您根据下图设置网络：

![4G_network_setup](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/4G_network_setup.png)

步骤如下：

1.   安装并配置4G网络   


2.   通过以太网线连接IPC到路由器


3.   配置路由器使用LTE蜂窝网络接入互联网


4.   配置LTE路由器的AP功能，使iPad Pro或其他移动设备可以连接到路由器，然后连接到IPC

![tip_icon](https://raw.githubusercontent.com/ireeX/apollo/master/docs/quickstart/images/tip_icon.png)
建议您配置一个固定的IP，而不是在IPC上使用DHCP，以使它更容易从移动终端被连接。

# 其它安装任务

需要使用自己提供的组件来执行以下任务:

1.   使用DVI或HDMI电缆连接显示器，并连接键盘和鼠标，以便在现场的汽车上执行调试任务。

2.   在Apple iPad Pro上建立Wi-Fi连接，以访问HMI并控制IPC上运行的Apollo ADS。

# 下一步

完成硬件部分的安装之后，可以参考快速入门的教程
 [Apollo Quick Start](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_1_0_quick_start.md) 完成软件部分的安装。
