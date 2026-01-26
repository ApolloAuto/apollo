本文档以一台完成线控改造的 林肯 MKZ 为例，介绍如何对 Apollo 自动驾驶车辆硬件系统进行集成安装。

您可以了解如何在车辆上完成相机、激光雷达、惯性导航系统以及计算单元 Hardware3.0 的集成安装。本次集成的传感器有 Hardware 3.0（HW 3.0）、Expansion Box（EP）、激光雷达、惯性导航系统 PP7、2 路毫米波雷达 以及 2 路相机。

本次集成硬件的电气框架拓扑图如下：

![图谱图](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/4735b0c6c902387bc6e33496a15a0097.png)

> 注意：本文档着重介绍硬件传感器的电气集成，硬件安装只做位置介绍展示，您可根据自己的车辆情况进行集成，且本次集成是按传感器集成顺序讲解，不代表实际硬件安装步骤。

## 前提条件

为了在 林肯 MKZ 上完成硬件集成安装，您需要提前了解以下内容：

- 激光雷达、相机、组合导航系统的基本功能及其接口类型，

- 准备好 Apollo 自动驾驶系统支持的传感器，详情参见 [Apollo 硬件开发平台](https://developer.apollo.auto/platform/hardware_cn.html)，

- 电气和通讯的基础知识。

## 硬件集成安装

林肯 MKZ 硬件设备集成安装位置分为两处。一处安装于后备箱固定铝板上，安装硬件设备有Hardware 3.0、Expansion Box 和 蓄电池；一处安装于车顶支架上，安装的硬件设备有激光雷达、相机、毫米波雷达 及 GNSS 天线。

> 注意：固定铝板和车顶支架为百度根据 林肯 MKZ 车辆结构及安装的硬件设备定制的。开发者可根据自己的车辆及安装的设备设计自己的固定铝板和车顶支架。

![后备箱硬件集成](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/infoflow%202021-10-13%2015-07-02_ae3f1d9.png)

![车顶支架](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/01523053f03b73c93a64a5093f8ed6a6.png)

## 电气集成安装

### Hardware 3.0 集成安装

百度为 林肯 MKZ 无人车配备的计算单元为百度自研的 Hardware 3.0 。Hardware 3.0 集成有 CPU 和 GPU 模块。同时，Hardware 3.0 可为外部各传感器提供 12V 直流电源以及各种传感器接口，如激光雷达接口、相机接口等。

以下是 Hardware 3.0 的硬件接口：

- Hardware 3.0 正面：
  
  Hardware 3.0 正面设有开关机按钮、2 个 NVME 数据盘以及 Hardware 3.0 状态指示灯。

![Hardware 3.0](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/523c3d9cf4c48fab5cf351363a5a7414.png)

- Hardware 3.0 背面：
  
  Hardware 3.0 背面设有系统电源接口，用于为 Hardware 3.0 提供电源；传感器电源接口，用于为外部传感器提供 12V 电源；远程上电开关，用于在驾驶室控制 Hardware 3.0 的开关机。

![Hardware 3.0背面.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/HW3.0%E8%83%8C%E9%9D%A2_28a20f1.png)

- Hardware 3.0 左侧：

![Hardware 3.0左侧.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/HW3.0%E5%B7%A6%E4%BE%A7_b0c3367.png)

Hardware 3.0 左侧接口：

1. EXPANSION接口，可通过MiniSAS HD线缆接到外扩的Expansion Box；

2. EXT GPS接口，用于接PP7的GPRMC和PPS信号；

3. LIDAR16/32 M2接口，用于接激光雷达；

4. LIDAR16/32 64接口，用于给外部的传感器提供GPRMC和PPS信号，共可以提供3组信号；

5. 8路1000M以太网接口，用于外接Lidar传感器或其它设备；

6. SFP+接口，用于将NVME数据盘的信息通过该接口跟服务器通信；

7. WIFI天线接口，用于为内部的WIFI模块提供天线；

8. 4G/LTE天线接口，用于为内部的4G模块提供天线。
- Hardware 3.0右测：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_ece7015.png)

Hardware 3.0 右侧接口：

1. CPU1 接口：包含显示器 VGA 接口，2 路 USB2.0 接口以及 1 路音频输出接口；

2. 24 Pin CAN 接口，通过该接口总共可以出 8 路 CAN 接口；

3. 10 个 Fakra camera 接口，用于外接相机；

4. CPU2 接口：包含 2 路 USB2.0 接口，1 路音频接口和显示器 DVI HDMI 接口；

5. 2 个 USB 接口（console 接口），为系统串口用于调试。

#### Hardware 3.0 硬件集成安装

将 Hardware 3.0 安装到 林肯 MKZ 后备箱固定铝板上。

#### Hardware 3.0 电气集成安装

1. 通过电源线将 Hardware 3.0 背部电源接口与蓄电池正负极连接。

> 注意：蓄电池连接正负极，接反烧坏 Hardware 3.0。

![Hardware 3.0电气接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/c138755592c8979e12ccb186a200cbe5.png)

![Hardware 3.0电气接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/e78d4f5e91113184a8d871ea87a0b1b1.png)

![Hardware 3.0电气接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/8ec152a84ca91ba6e0282d0e305b25d0.png)

2. 将传感器电源线接到 Hardware 3.0 背部的传感器电源接口上。

![Hardware 3.0电气接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/abe99dc09161842d9592c280a05d3cfe.png)

![Hardware 3.0电气接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/9200f491840ac1752947435730e4b65f.png)

> 注意：传感器电源线需要按照车规级要求制作，这条电源线包含了所有传感器硬件的电源供应。

### Expansion Box 集成安装

Expansion box 扩展 2 张加速卡，一张用于 camera 压缩，一张用于定位模块点云算法。

#### Expansion Box 硬件集成安装

将 Expansion Box 安装到 林肯 MKZ 后备箱固定铝板上。

#### Expansion Box 电气集成安装

1. 通过 Expansion Box 电源线，将 Expansion Box 连接到蓄电池上。

> 注意：蓄电池正负极不要连错。

![Expansion Box 接线](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/EP%E6%8E%A5%E7%BA%BF1-1_b7a1217.png)

![Expansion Box 接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/214b59fc79f1684edbdde79120dd9dde.png)

2. 通过四条 MiniSAS HD 线缆，将 Hardware 3.0 左侧面的 expansion 接口与 Expansion Box 对应接口相连接。

![Expansion Box接线](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/EP%E6%8E%A5%E7%BA%BF2-1_a70d68e.png)

![Expansion Box接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/3abcd6e30ea73d65ef86cd2cfaee577c.png)

![Expansion Box接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/8b28fa19ba0ddecbd112dce0cf60b8da.png)

### 惯性导航系统 PP7 集成安装

惯性导航系统属于无人车定位组成的一部分；惯性导航系统集成有 GNSS 接收板卡和惯性 IMU 传感器。林肯 MKZ 无人车使用的是 Novatel 的 PP7 设备，如图。

![PP7](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/a2787939dbba777ec1b290b49cafad8a.jpg)

NovAtel 的 PwrPak7(PP7) 是一款可扩展的高精度 GNSS 接受机，内部集成有 GNSS 和 INS 硬件，提供易于部署的 GNSS+INS 系统，PP7 使用高精度接收卡提供可扩展的高精度定位，使用 Epson G320N IMU 提供加速度计和陀螺仪测量。

对于惯性导航单元，主要接口有：

- 电源接口：为 PP7 设备提供电源；

- 授时输出接口：输出精准的时间，用与其他设备进行时间同步；

- 数据接口：对外提供数据输出，可配置选择使用网口或者 USB 接口。

#### PP7 硬件集成安装

1. PP7 安装在林肯 MKZ 后备箱后轴中间位置。

2. 将两个卫星天线安装在车顶支架后侧安装孔上。

![PP7安装](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/efacf425f82a329a2e273f113b59ae02.png)

#### PP7 电气集成安装

1. 通过射频连接线分别将两个天线与 PP7 的 ANT1、ANT2 接口连接，ANT1为主天线。

![PP7接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/a26441d013789a56210d402914210926.png)

![PP7接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/70ebc10f12ab545848add31c8ccd977a.png)

![PP7接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/dcd09a44e0974eab9bce506d862084d2.png)

2. 将传感器电源线上的PP7电源接头接到PP7电源接口上。

![PP7接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/916ca13e6702b96c5b8117b4704d822a.png)

![PP7接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/b64e4086c2d890ac445140aee679a5e5.png)

3. 通过 PP7 PPS 授时线将 Hardware 3.0 的EXT PPS 接口与 PP7 的 DB26 接口连接。

![PP7接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/1d28bf104d73de495f6881fd743fdc11.png)

![PP7接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/657577f8233a51fa79faeb7710d68435.png)

![PP7接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/c8111edceb277cb8f08d9c2081d7f43b.png)

4. 通过网线转 M12 线将 Hardware 3.0 的 lidar 接口与 PP7 的网口 2 连接。

> 注意：这里的线束是定制的，如果没有定制线束，可用网线将 PP7 网口与 Hardware 3.0 的网口 2 相连接使用。

![PP7接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/3778c2d2a51bbc6dcd90df5e641b92df.png)

![PP7接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/a06bb8a496611ec38938780eb06840a6.png)

![PP7接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/c4768d0828b39f9add4b40fbfc76f44c.png)

### 激光雷达集成安装

林肯 MKZ 无人车集成的激光雷达为 40 线激光雷达。

#### 激光雷达硬件集成安装

激光雷达安装在车顶车架上。

![激光雷达](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/bc10f5c4cc52edcf3908f16933042d95.png)

#### 激光雷达电气集成安装

通过激光雷达延长线将激光雷达接到 Hardware 3.0 的 M12 接口上。

![激光雷达接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/a983e83ecb403f0d7f050676badcd3e5.png)

![激光雷达接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/26b2a9b5fa74aa33779a2d4bde935dfd.png)

![激光雷达接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/a18c0e1aee0bba4ad6db27bfec50e309.png)

### 相机集成安装

在 Hardware 3.0 上提供有 10 路 Fakra 相机接口，Fakra 相机接口如下图所示：

![相机](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/19a5a70fa182b0d23ab37dae9a27017f.png)

#### 相机硬件集成安装

对于开发者学习调试场景，林肯 MKZ 无人车只使用了 2 路 Fakra 相机，一路 6mm 相机，一路 12mm 相机，相机朝正前方安装。如果您有多相机应用场景，可以根据需求增加。

![b4](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/b407154fc570f77becd45892f_b407154.png)

#### 相机电气集成安装

通过 Fakra 延长线将相机连接到 Hardware 3.0 Fakra 接口上，Hardware 3.0 Fakra 接口。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_4761fdc.png)

![相机接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/5780a08ec979037d4b29ce6202805200.png)

![相机接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/6f1c1ba7357785de1553d59acdd69613.png)

### 毫米波雷达集成安装

Hardware 3.0 CAN 通过右视图中的 24 pin CAN 接口引出，Hardware 3.0 提供 8 路 CAN，其中 4 路为 Radar（对应的顺序为 0/1/2/4），2 路为车身 CAN（3/7），另外 2 路为预留。车身 CAN 默认使用 CAN0 对应的连接为 CAN3；CAN7软件上对应的为 CAN1。

林肯 MKZ 无人车安装有两路 continental ARS 408-21 毫米波雷达，ARS408-21 是大陆 40X 毫米波雷达传感器系列中最新推出的高端产品，可以适用不同的应用场景。ARS408-21 很好的处理了测量性能与高安全性之间的矛盾，可实时检测目标的距离并根据当前车速判断是否存在碰撞风险；具有自动故障检测功能，可识别传感器问题，并自动输出故障码，轻量化设计，具有很好的鲁棒性。

![radar](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/7b7270b6f25ad76f523d30334a7788b4.png)

![radar](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/841fbfdc9e00ad239e62eddd6f887b62.png)

#### 毫米波雷达硬件集成安装

将两路毫米波雷达分别安装在车辆前方和车顶支架的后方。

> 注意：激光雷达安装时正面朝上安装，前方毫米波雷达需要加装安装架。

#### 毫米波雷达电气集成连接

通过毫米波雷达线束连接毫米波雷达与 Hardware 3.0 CAN 接口定制线束的 can0 与 can1 接口。

> 注意：此线束为定制线束。请根据实际情况进行连接，对于本文档提供的线束为两条，一条是 Hardware 3.0 转 DB9（提供 CAN 总线接口），一条是连接集成毫米波雷达单元的线束。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_6_0/image_8581c54.png)

![radar接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/6762a41f25a59318c38771fc12cb8266.png)

![radar接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/653824716ba1f7e39dbaaaa07006ca40.png)

### 4G 路由器集成安装

Apollo 无人车采用基于 RTK 的 GNSS 定位，需要连接外网接受基站信息，需要连接网络。百度为 林肯 MKZ 配备的是 4G 无线路由器，内插 4G 物联网卡，来提供连接外网的能力。

![4Grouter](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/11bdae920fdc6d7d9bd82206450e6188.png)

#### 4G 路由器硬件集成安装

将 4G 路由器安装与后备箱合适位置，若有天线，请将天线引出安装在车顶支架合适位置。

#### 4G 路由器电气集成安装

1. 将传感器线束上的路由器电源连接到路由器电源位置。

2. 通过网线将 4G 路由器网口连接到 Hardware 3.0 的网口 1 上。

![4Grouter接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/8d50d6a556c1797a4257b03edbcef0a4.png)

![4Grouter接线](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo_Doc_CN_6_0/2760d8ab38feffcd85f8e202f7efcf9c.png)

> 注意：在使用时确保物联网卡已经正确激活并且安装到了 4G 路由器上。
