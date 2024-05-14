# Apollo 3.0 硬件与系统安装指南

* [关于本指南](#关于本指南)
    * [文档约定](#文档约定)
* [引言](#引言)
    * [软硬件信息](#软硬件信息)
* [关键硬件](#关键硬件)
    * [附加组件](#附加组件)
* [安装步骤](#安装步骤)
    * [实验室安装](#实验室安装)
    * [车辆安装](#车辆安装)
        * [先决条件](#先决条件)
        * [主要组件安装示意图](#主要组件安装示意图)
* [附加组件](#附加组件)
* [后续步骤](#后续步骤)

## 关于本指南

*Apollo3.0硬件与系统安装指南*详细介绍了Apollo计划的硬件组件、系统软件安装，安装过程包含下载和安装Apollo Linux内核。

### 文档约定

下述表格列举了在本文档中使用的标识符约定：

| **图标**                            | **描述**                          |
| ----------------------------------- | ---------------------------------------- |
| **粗体**                            | 重点强调                                 |
| `等宽字体`                   | 代码，类型数据                         |
| _斜体_                            | 标题、章节和标题使用的术语 |
| ![info](images/info_icon.png)       | **Info**  包含可能有用的信息。忽略信息图标没有消极的后果。 |
| ![tip](images/tip_icon.png)         | **Tip**. 包括有用的提示或可能有助于完成任务的快捷方式。 |
| ![online](images/online_icon.png)   | **Online**. 提供指向特定网站的链接，您可以在其中获取更多信息。 |
| ![warning](images/warning_icon.png) | **Warning**. 包含**不能忽略**的信息，否则在执行某个任务或步骤时，您将面临失败的风险。 |

## 引言

 **Apollo Project** 是为汽车和自主驾驶行业的合作伙伴提供开放，完整和可靠的软件平台。该项目的目的是使合作伙伴能够基于Apollo软件套件开发自己的自动驾驶系统。
 
 ### 软硬件信息
 
- ***<u>【Apollo硬件与系统安装指南】</u>*** ─ 链接到Specs中的硬件开发平台文档
 
    - **车辆**：
     
      - [工业级PC](../../../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E4%BC%A0%E6%84%9F%E5%99%A8%E5%AE%89%E8%A3%85%20sensor%20installation/IPC/Nuvo-6108GC_Installation_Guide_cn.md)
      - [全球定位系统（GPS）](../../../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E4%BC%A0%E6%84%9F%E5%99%A8%E5%AE%89%E8%A3%85%20sensor%20installation/Navigation/README_cn.md)
      - [惯性计算单元（IMU）](../../../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E4%BC%A0%E6%84%9F%E5%99%A8%E5%AE%89%E8%A3%85%20sensor%20installation/Navigation/README_cn.md)
      - 区域网络控制卡（CAN）
      - GPS天线
      - GPS接收器
      - [激光雷达（LiDAR）](../../../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E4%BC%A0%E6%84%9F%E5%99%A8%E5%AE%89%E8%A3%85%20sensor%20installation/Lidar/README.md)
      - [摄像机](../../../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E4%BC%A0%E6%84%9F%E5%99%A8%E5%AE%89%E8%A3%85%20sensor%20installation/Camera/README.md)
      - [雷达](../../../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E4%BC%A0%E6%84%9F%E5%99%A8%E5%AE%89%E8%A3%85%20sensor%20installation/Radar/README.md)
      - [Apollo传感器单元（ASU）](../../../11_Hardware%20Integration%20and%20Calibration/%E8%BD%A6%E8%BE%86%E9%9B%86%E6%88%90/%E4%BC%A0%E6%84%9F%E5%99%A8%E5%AE%89%E8%A3%85%20sensor%20installation/Apollo_Sensor_Unit/Apollo_Sensor_Unit_Installation_Guide_cn.md)
      
    - **Software**: 
      - Ubuntu Linux
      - Apollo Linux 内核
      - NVIDIA GPU 驱动
    
- ***<u>【Apollo快速入门指南】</u>*** ─ 包含了教程和提供有从头至尾完整指令序列的执行路径图的文档。快速入门指南同时链接到描述将普通车辆改装成自动驾驶车辆的操作步骤的文档。

## 关键硬件

需要安装的关键硬件包括：

- 车载计算机系统 ─ Neousys Nuvo-6108GC
- 区域网络控制卡（CAN） ─ ESD CAN-PCIe/402-B4
- 全球定位系统（GPS）和惯性计算单元（IMU） ─ 使用者可以从下述选择中任选一种：
  - NovAtel SPAN-IGM-A1
  - NovAtel SPAN® ProPak6™ and NovAtel IMU-IGM-A1
  - Navtech NV-GI120
- 激光雷达 ─ 使用者可以从下述选择中任选一种：
    - Velodyne HDL-64E S3
    - Velodyne Puck series
    - Innovusion LiDAR
    - Hesai's Pandora
- 摄像机 — 使用者可以从下述选择中任选一种：
    - Leopard Imaging LI-USB30-AR023ZWDR with USB 3.0 case
    - Argus Camera
    - Wissen Camera
- 雷达 — 使用者可以从下述选择中任选一种：
    - Continental ARS408-21
    - Delphi ESR 2.5
    - Racobit B01HC

### 附加组件

使用者需要提供附加组件以完成额外的安装任务：

- Apollo传感器单元（ASU）
- 提供网络接入的4G路由器
- 提供额外USB接口的USB集线器
- 供在车辆现场调试使用的显示器、键盘和鼠标
- 数据线：数字视频接口线（DVI）（可选），供GPS和激光雷达进行时间同步的定制数据线
- 苹果iPad Pro:9.7寸（可选）

这些关键硬件的特性将在后续章节中介绍。

## 安装步骤

本章节介绍的安装步骤包括：

- 关键软硬件组件
- 车辆中的硬件

### 实验室安装

执行如下安装步骤：

- 准备IPC
    - 安装CAN卡
    - 安装或替换硬盘驱动器
    - 安装为IPC加电的组件
- 为IPC安装软件：
    - Unbuntu Linux
    - Apollo内核
    - NVIDIA GPU驱动
    
至此，可以将IPC挂载到车辆上。

### 车辆安装

执行如下安装步骤：

- 确保所有在先决条件中列出的对车辆的修改都已执行
- 安装主要组件
    - GPS天线
    - IPC
    - GPS接收器和IMU
    - 激光雷达
    - 摄像机
    - 雷达
    
#### 先决条件

**![warning_icon](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/images/warning_icon.png)注意**：在将主要部件（GPS天线，IPC和GPS接收器）安装在车辆之前，必须按照先决条件列表所述执行必要修改。 列表中所述强制性更改的部分，不属于本文档的范围。

先决条件为：

- 车辆必须由专业服务公司修改为“线控”技术。 此外，必须在要安装IPC的中继线上提供CAN接口连接。
- 必须在后备箱中安装电源插板，为IPC和GPS-IMU提供电源。电源插板还需要服务于车上的其他硬件，比如4G的路由器。电源插板应连接到车辆的电源系统。
- 必须安装定制的机架，将GPS-IMU天线安装在车辆的顶部。
- 必须安装定制的机架，以便将GPS-IMU安装在后背箱中。
- 必须安装定制的机架，以便将前向雷达安装在车辆前部。
- 必须将4G LTE路由器安装在后备箱中才能为IPC提供Internet访问。路由器必须具有内置Wi-Fi接入点（AP）功能，以连接到其他设备（如iPad），以与自主驾驶（AD）系统相连接。例如，用户将能够使用移动设备来启动AD模式或监视AD状态。

#### 主要组件安装示意图

以下两图中标明了三个主要组件（GPS天线，IPC，GPS接收器和LiDAR）在车辆上的安装位置：

![major_component_side_view](https://github.com/ApolloAuto/apollo/blob/master/docs/demo_guide/images/Hardware_overview.png)

![major_component_rear_view](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/images/Car_Rearview.png)

## 额外安装任务

使用者使用已提供的额外组件实现如下的安装任务：

1.   使用DVI线或HDMI线连接显示器，并连接鼠标和键盘实现车辆现场调试功能。
1.   在Apple iPad Pro上建立一个Wi-Fi连接以访问HMI和控制在IPC上运行的Apollo自动驾驶系统（ADS）。

## 后续步骤

当完成车辆的的硬件安装后，参考[Apollo 快速入门指南](../../../02_Quick%20Start/apollo_3_0_quick_start_cn.md)以获取完整的软件安装步骤。
