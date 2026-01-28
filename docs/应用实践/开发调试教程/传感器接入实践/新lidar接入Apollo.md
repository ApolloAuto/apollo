## 激光雷达接入要求

Apollo 自动驾驶开放平台主要为园区、教育、低速场景提供完整的解决方案。Apollo 以激光雷达为主，支持市场中主流的机械、半固态激光雷达方案。目前 Apollo 已经支持禾赛、速腾、镭神、Velodyne、法雷奥、LeddarTech 等品牌的部分激光雷达接入。

激光雷达的发展日新月异，设备厂商开发出更多、更好、更具性价比的激光雷达，为了帮助厂商更快建立激光雷达测试环境，帮助开发者使用最新的激光雷达，本指导手册详细说明了Apollo激光雷达驱动开发、驱动测试、驱动代码贡献的方法。

## 激光雷达接入流程

### 驱动开发与测试

- 根据本文指引，以及参考代码，完成激光雷达驱动开发；

- 调度、运行代码，确认驱动输出内容满足Apollo要求；
  
  ### 成为贡献者

- 连接真实设备，运行代码，确认输出信息正确；

- 录制测试视频、激光雷达输出信息；

- 提供测试过程数据、开发者信息到 Apollo 团队；

- 提交代码到 Github，等待 Apollo 团队审核。
  
  ### 贡献者宣传

- 硬件设备进入 Apollo 开发平台官网硬件展示；

- 作为贡献者，信息在 Apollo 官网展示。

## 激光雷达技术要求

* 【强制】lidar 支持网口通过 TCP 或者 UDP 协议输出点云数据；
* 【强制】lidar 能够稳定以 10HZ 频率输出点云；
* 【强制】可以获取每个 lidar 点的 x、y、z、强度值以及时间戳信息；
* 【建议】提供 lidar C++ SDK，通过 SDK 可以获取原始数据包，以及提供原始数据包转成点云格式接口；
* 【建议】lidar SDK 可以按照角度、距离过滤 lidar 点云；
* 【建议】可以获取每个 lidar 点的 ring 信息；
* 【强制】【不支持】Apollo 感知算法暂不支持基于 2D 激光雷达的感知，故暂不支持2D激光雷达的接入。使用 2D 激光雷达，建议由对 Apollo 系统有良好的认知专家介入。

## 开发流程说明

### 准备工作

1. 学习 [Apollo 组件开发流程](https://apollo.baidu.com/community/Apollo-Homepage-Document?doc=BYFxAcGcC4HpYIbgPYBtXIHQCMEEsATAV0wGNkBbWA5UyRFdZWVBEAU0hFjwDsD2AD0ygKqIA)；
2. 学习 github pull request 相关资料 [拉取请求文档](https://docs.github.com/zh/pull-requests)；
3. 准备好 lidar SDK 驱动，在 x86_64、aarch64 架构编译 Linux 动态库；
4. 熟悉`modules/drivers/lidar`下的禾赛、速腾激光雷达驱动程序；
5. 了解 apollo lidar 驱动框架，选择轮询或订阅方式开发驱动。

### 开发工作

1. Fork Apollo 代码库：进入 [https://github.com/ApolloAuto](https://github.com/ApolloAuto), 点击左上角 Fork 按钮；
2. 以禾赛、速腾驱动为模板，开发新的激光雷达驱动；
3. 通过 cyber tools 查看驱动输出信息；
4. 提交代码，通过评审，合入代码。

## lidar开发框架简介

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a2f71a5.png)

激光雷达驱动运行框架需要满足下面的两个目标：

- 读取激光雷达设备数据，并将设备原始数据转换成感知组件需要的点云数据格式。
- 为了保障自动驾驶落盘过程中尽量节省磁盘空间，数据落盘默认只保存原始数据。

点云数据占用的空间远高于原始数据包，故而在驱动框架设计时，提供`ONLINE_LIDAR`和`RAW_PACKET`两种模式，分别是从连接 lidar 的网卡读取二进制数据和从 scan 通道读取二进制数据。在自动驾驶过程中应使用`ONLINE_LIDAR`模式从连接 lidar 的网口读入，并输出原始数据以及点云数据。在数据落盘后，启动`RAW_PACKET`模式可根据落盘的原始数据还原出当时的点云数据。

## Lidar驱动代码开发

### 驱动开发目录

lidar 驱动目录统一放在 Apollo 源码的`drivers/lidar`目录下，新设备需要创建新的目录结构，目录内容如下所示。

```bash
xxlidar： # 以lidar类型名创建目录
    - conf：# conf目录存放lidar驱动配置
    - dag： # dag目录存放启动lidar驱动的dag文件
    - launch:  # luanch目录存放启动lidar驱动的launch文件
    - proto： # 存放conf格式定义以及其他自定义类型的proto配置
    - src：  # 存放lidar驱动的源代码
    - README_cn.md  # lidar驱动的中文说明
    - README_en.md  # lidar驱动的英文说明
```

### Lidar SDK 准备

考虑到厂商代码维护，也考虑到第三方代码库与 Apollo 代码规范/编译工具的兼容，Apollo 建议通过提供 SDK 包引入方式加速驱动开发。

对于 SDK 代码，不限制用 cmake 或者其他编译工具链，但必须支持 x86_64、aarch64 架构编译，支持指定目录安装，安装输出需满足：

* 包含 include 目录，include 目录的（路径）头文件名不能和系统以及第三方库相同，否则会导致其他程序发生引用错误问题，建议 include 下放一层 lidar 名的子目录，比如`include/xxlidar/point_cloud.h`。
* 包含 lib 目录，lib 目录下存放编译产出的 so 库，最好只输出一个 so，注意必须是动态链接库，不能是 .a 的静态库；

以上准备好后，把改好的代码提供给 Apollo 团队，由 Apollo 团队完成编译（会修改一些编译参数，兼容各种 CPU 型号）以及 deb 包制作，确认可发布后，Apollo 团队会把 deb 包发布到 Apollo apt 源。

SDK 引用方式：参考样例的 hesai、rslidar下面的 BUILD 和 cyberfile.xml 配置。

* BUILD：build 文件中加上`-l`参数直接使用，
* cyberfile.xml: 加上 SDK 的依赖，包的名字在代码给到 Apollo 团队后提供。

自己调试阶段时，直接把编译产出安装到`/usr/`或者`/usr/local`目录下即可。

### 配置文件说明

驱动配置需包含 lidar 驱动基础配置以及自定义配置。
用户自定义配置主要是定义一些 lidar 型号、IP、端口、过滤规则等，一份完整配置样例如下：

```bash
# 基本配置
base_config {
    scan_channel: "/apollo/sensor/hesai40/Scan"
    frame_id: "hesai40"
    pointcloud_channel: "/apollo/sensor/hesai40/PointCloud2"
    source_type: ONLINE_LIDAR
}
# 自定义配置
model: HESAI40P
ip: "192.168.20.13"
lidar_recv_port: 2368
gps_recv_port: 10110
start_angle: 0
time_zone: 8
```

基础配置主要是一些共性配置，Apollo 驱动中已经定义好，格式为：

```bash
message LidarComponentBaseConfig {
    required string scan_channel = 1;  // 原始数据输出channel名称
    required string point_cloud_channel = 2;  // 点云数据输出channel名称
    required string frame_id = 3;  // lidar的frame_id, 要求全局唯一，外参转换、感知程序都会用到这个
    required SourceType source_type = 4; // 二进制数据来源；定义为ONLINE_LIDAR时，将从lidar采集二进制数据，并将二进制数据写入scan_channel；定义为RAW_PACKET时，将从scan_channel采集二进制数据。
    optional uint8_t lidar_id = 5; // 记录一个lidar序号，要求全局唯一，感知程序会用，非必须
    optional uint16_t buffer_size = 6 [default = 50]; // 点云分配buffer大小，通常不需要设置
}
```

### Lidar 驱动开发接口说明

Apollo 官方提供了 Lidar 驱动组件基类，新 lidar 驱动需继承此基类，基类的接口如下：

**Lidar驱动组件基类**

```bash
// LidarComponentBase模板类，T是scan数据的类型
template <typename T>
class LidarComponentBase: public cyber::Component<T> {
public:
  // 用户需要实现Init方法，在Init方法中调用InitBase完成初始化配置；
  void Init() = 0; 

  // 用户需要实现ReadScanCallback，当选择RAW_PACKET模式时，组件将读取scan通道的数据。用户需要实现读取scan数据后输入到lidar sdk的过程
  void ReadScanCallback(shared_ptr<T> scan_message);

protected:
   // 基类中实现: 完成Scan/Pcd writer等创建
   bool InitBase(const driver::lidar::BaseConfig& base_config);

   // 基类中实现: 写scan channel数据
   bool WriteScan(const std::shared_ptr<T>& scan_packet);

   // 基类中实现:分配一个点云对象，注意需要使用此方法获取输出点云对象，否则可能会引入性能问题；
   std::shared_ptr<driver::PointCloud> AllocatePointCloud();

   // 基类类中实现: 写pcd channel数据
   bool WritePointCloud(const std::shared_ptr<driver::PointCloud>& pcd);   
}
```

lidar 驱动库的 common 目录提供基类 LidarComponentBase，该基类会完成基础配置的读取，并完成点云和 scan 读写通道的初始化操作，并提供`RAW_PACKET`和`ONLINE_LIDAR`两种数据输入模式。适配者需要关注以下方面的实现：

* 适配者需要实现`ONLINE_LIDAR`和`RAW_PACKET`两种输入模式下驱动的行为。`ONLINE_LIDAR`模式从网卡读取数据并经 SDK 解析后输出点云、scan 信息；而`RAW_PACKET`模式则是从 scan 通道读取雷达数据，需要经 SDK 解析 scan 数据中的二进制数据输出点云数据。两种模式的差别在于`ONLINE_LIDAR`需要输出 scan 通道的数据，而`RAW_PACKET`需要从 scan 通道获取数据，然后根据获得的数据输出点云。

* 驱动初始化：用户实现 Init 函数，用于配置 SDK 的网卡数据监听，并根据上述两种输入模式设置 SDK 的行为；并在此过程中，调用 InitBase 函数完成初始化配置。

* 调用 scan 发布函数：LidarComponentBase 提供了封装和发布 scan 数据的函数 WriteScan，适配者需要在接收到数据包时，传入二进制的数据包。执行后将会完成 scan 数据在配置通道的发布。
  
  > 注意：确保只在`ONLINE_LIDAR`模式下调用该函数。

* 实现 scan 通道数据写入：用户实现 ReadScanCallback 函数，将自定义类型的 scan 数据输入 SDK。

* 调用 pcd 发布相关函数：LidarComponentBase 提供了封装和发布 scan 数据的函数 AllocatePointCloud 和 WritePointCloud。用户需要调用 AllocatePointCloud 分配点云内存空间，在点云信息写入完毕后调用 WritePointCloud 发布。

速腾驱动适配参考伪代码：

```bash
// rslidar_component.h
class RslidarComponent : LidarPacketComponentBase<ScanType> {
public:
  bool Init() override; 
  // 收到apollo scan数据的处理方式
  void ReadScanCallback(const std::shared_ptr<ScanType>& scan_message) override;

     // 适配者自定义的点云回调函数，返回sdk对应类型的点云数据
  void RsPointCloudCallback(raw_pcd_message);
  void RsPacketCallback(raw_scan_message);
}

// ------------------------------------------------------------------------
// rslidar_component.cpp
void RslidarComponent::Init() override {
  // 配置文件读取
  // 速腾sdk初始化...
  // 注册sdk scan帧回调函数、点云帧回调函数
  sdk->regiest(this->RsPacketCallback);
  sdk->regiest(this->RsPointCloudCallback);

  // 调用父类方法，初始化cyber节点
  InitBase(base_config);
}

int RslidarConvertCompoment::ReadScanCallback(shared_ptr<ScanType> scan_message) override {
    // 当收到scan数据时，调用Proc函数将数据输入sdk解析
    sdk->decodePacket(scan_message);
}

void RslidarPacketComponent::packetCallback(const RslidarPacket& rslidar_packet) {
      std::shared_ptr<ScanType> scan_packet;
      // convert RslidarPacket to ScanType ...

    WriteScan(scan_packet);
    // 调用LidarComponentBase父类提供的方法，用于封装和广播scan数据
}

// 适配者自定义的点云回调函数，返回sdk对应类型的点云数据
void RslidarConvertCompoment::pointCloudPutCallback(const RslidarRawMessage& raw_message) {
  shared_ptr<ApolloMsg> pcd_message = AllocatePointCloud(); //调用父类提供方法，获取已分配好内存的空点云

  // 适配者将raw_message写入pcd_message
  for (int i = 0; i < raw_message.size(); ++i) {
    pcd_message.mutable_point(i).set_x(raw_message.point(i).x());
    pcd_message.mutable_point(i).set_y(raw_message.point(i).y());
    ...
  }
  WritePointCloud(pcd_message); // 调用父类方法，向通道发送点云数据
}
```

### 运行调试

```bash
# 编译
bash apollo.sh build_opt_gpu drivers/lidar

# 运行: 修改成自己编写的dag路径
mainboard -d /apollo/modules/drivers/lidar/xxlidar/dag/xxlidar.dag

# 启动cyber_monitor 观察自定义的scan channel是否正常输出，以及pointcloud channel是否有稳定10HZ输出
cyber_monitor

# 启动 cyber_visualizer 可视化工具查看点云：点击Show PointCloud，选择对应channel
cyber_visualizer
```

观察点云输出和实际场景一致：以下是一个 16 线例子：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_0252d80.png)