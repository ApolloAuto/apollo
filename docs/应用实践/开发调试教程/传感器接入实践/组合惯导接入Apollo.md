## 组合惯性导航接入要求

Apollo 自动驾驶开放平台主要为园区、教育、低速场景提供完整的解决方案。Apollo 目前虽然在 SLAM 支持上持续成长，但目前主要的场景仍然以高精地图与高精定位基础。目前 Apollo 已经支持华测、导远、博盛尚、星网宇达、Novatel 等品牌旗下的部分组合惯性导航设备。

目前应用于 L4 组自动驾驶的组合惯性导航成本依然较高，产品迭代较频繁，为了帮助厂商更快建立导航设备在在自动驾驶场景中的应用，本指导手册详细说明了 Apollo 组合惯性导航驱动开发、驱动测试、驱动代码贡献的方法。

## 组合惯性导航接入流程

### 驱动开发与测试

- 根据本文指引，以及参考代码，完成驱动开发；
- 调度、运行代码，确认驱动输内容满足 Apollo 要求。
  
  ### 成为贡献者
- 连接真实设备，运行代码，确认输出信息正确；
- 录制测试视频、驱动输出信息；
- 提供测试过程数据、开发者信息到 Apollo 团队；
- 提交代码到 Github，等待 Apollo 团队审核。
  
  ### 贡献者宣传
- 硬件设备进入 Apollo 生态支持清单；
- 作为贡献者，信息在 Apollo 官网展示。
  
  ## 组合导航设备接入要求
* 【建议】组合惯导能够以 100HZ 稳定输出imu相关信息；
* 【建议】组合惯导支持 RTK 定位增强；
* 【强制】组合惯导支持 PPS 授时输出；
* 【强制】组合惯导需要双天线定向支持；
* 【强制】如果依赖外部输入 RTK 差分信息，必须支持输出 $GPGGA 数据，并且 GPGGA 数据和位姿数据必须通过一个数据线输出。【技术】

## 组合导航设备输出数据要求

* 【强制】GPSWeek： 自 1980/1/6 至当前星期数(格林尼治时间)；
* 【强制】GPSTime：本周日 0:00:00 至当前的秒数；
* 【强制】Latitude：纬度（-90° 至 90°）；
* 【强制】Longitude：经度（-180° 至 180°）；
* 【强制】Altitude：高度，单位（米）；
* 【强制】Heading（Yaw）： 航向角，0 至 359.99，北偏东顺时针为正（如果不是需自行转换）（IMU 原始值和校正值至少输出一个）；
* 【强制】Pitch：俯仰角（-90 至 90)，车头上扬为正 （IMU 原始值和校正值至少输出一个）；
* 【强制】Roll：横滚角（-180 至 180），车身右倾为正（IMU 原始值和校正值至少输出一个）；
* 【强制】Gyro X/Y/Z：陀螺仪 X 、Y、Z 轴，单位 °/s；
* 【强制】Acc X/Y/Z：加速度 X、Y、Z 轴 （IMU 原始值和校正值至少输出一个）；
* 【强制】Ve、Vn、Vu：东向、北向、天向速度，单位 m/s；
* 【强制】SolutionStatus: 系统状态，参考 [Solution Status](https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#SolutionStatus) 定义，至少能区分是否是组合导航；
* 【强制】PositionType：卫星状态，参考 [Position or Velocity Type](https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm#Position_VelocityType)， 至少包含不定向、RTK浮点解、稳定解；
* 【建议】Latitude_std、Longitude_std、Altitude_std：纬度、经度、高度标准差；
* 【建议】Heading_std、Pitch_std、Roll_std： 航向角、俯仰角、横滚角标准差；
* 【建议】Ve_std、Vn_std、Vu_std: 东向、北向、天向速度标准差，单位：m/s；
* 【建议】SatelliteNum：解算卫星数量；
* 【建议】Age： 差分延迟。

## 开发流程说明

### 准备工作

1. 学习 [Apollo 组件开发流程](https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFjwDsD2AD0ygKqIA)；
2. 学习 github pull request 相关资料 [拉取请求文档](https://docs.github.com/zh/pull-requests)；
3. 熟悉`modules/drivers/gnss`下的华测、导远的驱动程序。

### 开发工作

1. Fork Apollo代码库：进入 https://github.com/ApolloAuto ，点击左上角 Fork 按钮；
2. 以 novatel、华测、导远、博盛尚驱动为模板，开发新的组合惯导驱动；
3. 通过 cyber tools 查看驱动输出信息；
4. 提交代码，通过评审，合入代码。
   
   ## 代码研发说明
   
   ### 驱动开发输出目标

驱动代码要求的输出信息和频率要求如下：

| Channel                           | 格式  | 说明        | 是否必须 | 频率要求              |
| --------------------------------- | --- | --------- | ---- | ----------------- |
| /apollo/sensor/gnss/best_pose     |     | RTK定位信息   | 是    | 不低于1HZ            |
| /apollo/sensor/gnss/corrected_imu |     | 校正惯导信息    | 是    | 100HZ（上下浮动不超过30%） |
| /apollo/sensor/gnss/odometry      |     | INS信息     | 是    | 100HZ（上下浮动不超过30%） |
| /apollo/sensor/gnss/ins_stat      |     | 状态信息      | 是    | 不低于1HZ            |
| /apollo/sensor/gnss/heading       |     | Heading信息 | 否    | 不低于1HZ            |
| /apollo/sensor/gnss/imu           |     | 原始IMU信息   | 否    | 100HZ（上下浮动不超过30%） |

### GNSS驱动开发说明

gnss 驱动目录统一放在 apollo 源码的`drivers/gnss`目录下，结构为：

```bash
- conf：# gnss配置，所有厂家使用一份配置
- dag： # dag文件，所有厂家使用一个DAG
- launch:  # launch文件，所有厂家使用一个Launch文件
- proto： # 存放conf格式定义，新增gnss需要改到这个proto配置
- parse：  # 数据解析代码，新增gnss需要增加一个新的parser类型
- stream:  # 数据流读入基础类，支持串口、tcp、udp、can协议，用户通常不需要修改
```

#### gnss 驱动代码框架说明

stream 目录是一个基础库，内置了 can、ntrip、serial、tcp、udp 各种协议支持，每个 stream 需要实现 Connect 和 read 方法，实际运行时会一直调用 read 方法获取数据，写到 parser/parser.h 里面定义的 Parser 类对象的 data_ 中。

parser 中负责各种厂商 gnss 数据解析，新惯导接入需要新增一个 parser 类型，必须继承 Parser 基类，需实现以下函数：

* CreateXX 函数: 创建一个自定义类型的 Parser，然后在 parser.h 中新增该类型的调用；
* GetMessages 函数：这个是新 gnss 接入实现的关键函数，方法传入了一个 message 数组，对于兼容 novatel 协议可以直接解析返回，对于自定义的一条数据输出或者CAN协议输出，需要自己维护 GnssBestPose、Imu、Heading、Ins、InsStat 几个对象，把最新数据值更新到几个对象中，gnss 驱动库已经内置了一些惯导的支持，可以参考其实现。

#### 代码开发说明

新的 gnss 驱动接入通常需求修改：

* 在 proto/config.proto 中 Stream.Format 中新增 gnss 类型；

* 在 parser 路径下实现自己类型的数据解析库：根据您的数据协议参考已有代码实现 
  
  * 如果是分类型数据二进制输出，参考 novatel_parser.cc 实现；
  * 如果是一整条完整数据二进制输出，参考 broadgnss_parser/broadgnss_binary_parser.cc 实现；
  * 如果是一整条文本数据输出，参考 huace_parser/huace_text_parser.cc 实现；
  * 如果是 CAN 协议输出，参考 huace_parser/huace_can_parser.cc 实现；

* 在 parser/parser.h 里面的增加您的 CreateParser 函数；

* 修改 conf/gnss_conf.pb.txt 配置，添加新增 gnss 类型的使用说明，注意需要以注释的形式添加，不要修改默认生效值；
  
  > 注意：尽量不要修改其他代码，尤其是 parser/data_parser.cc，如必须要修改，请联系 Apollo 团队，否则 PR 不会给予合入。

### 运行调试

```bash
# 编译
bash apollo.sh build_opt_gpu drivers/gnss

# 修改 modules/drivers/gnss/conf/gnss_conf.pb.txt 中data配置为自己的惯导配置
vim modules/drivers/gnss/conf/gnss_conf.pb.txt

# 运行gnss驱动
mainboard -d /apollo/modules/drivers/gnss/dag/gnss.dag

# 启动cyber_monitor 观察是否正常输出以及频率是否满足上述要求
cyber_monitor
```

主要观察红框里面的 4 个 channel 频率是否符合要求，并查看里面的位置、时间等字段是否正确。如果您有 apollo 的地图，建议也启动 localization 模块查看车辆位置和地图中位置、朝向是否匹配，并通过加速、减速控制判断加减速度都是和预期一致。

![image \(1\).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image%20%281%29_2cb50b4.png)
