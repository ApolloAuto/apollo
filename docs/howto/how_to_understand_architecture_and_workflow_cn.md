如何理解架构和工作流程
===========================================

## 了解AplloAuto核心基础知识 

规划引擎（planning engine）通过CAN bus实现自动驾驶车辆动态的控制。 与汇编语言的做法类似，软件从硬件寄存器中读取数据并把数据写入。定位模块，感知模块，规划模块是作为独立的输入源和输出源通过Peer2Peer协同工作。 RPC网络应用支持P2P。
 
ApolloAuto使用ROS1作为底层网络，这意味着ApolloAuto从ROS1借用了Master-Nodes框架。 因为来自ROS1的xmlRPC比较陈旧 \(相较于brpc和[grpc](https://yiakwy.github.io/blog/2017/10/01/gRPC-C-CORE)\),百度开发了自己的RPC protobuf版本。

在百度ApolloAuto中，描述了三个开发阶段

1.离线仿真引擎Dreamview和ApolloAuto核心软件模块
    - 帮助用户作初步了解这些算法如何在车上工作
    - 即使没有汽车或者相关硬件，也能立即开始开发
 
2.核心模块集成：
    - 定位
    - 感知\(支持第三方解决方案，例如适用于L2开发的基于Mobileye ES4芯片的摄像头\)处理来自激光雷达的点云数据，并根据要求返回分割对象信息
    - 规划：车辆的动态全局路径来自route服务，规划模块会计算路径中路径段的微调路径。
    - 路线：用A \ * star算法，通过`Navigator`接口，在本地实现路径段查找。

3.高清地图：与L2级别AV开发的主要区别之一， 是L4 AV机器需要高精地图。 由于机器人\(自主车辆\)需要在其微机内重建3D世界\(请检查OpenCV [SLAM]()章节\)，AV在地图和现实世界中的重定位，参照物坐标扮演了重要角色。

4. 云端仿真和数据中心
    - 作为百度的合作伙伴，您将获得docker凭证以提交新的镜像并重新运行你在云上开发的算法。
    - 创建和管理复杂的场景，模拟真实世界的驾驶情景

## ROS基础订阅和发布机制以及ApolloAuto模块结构

#### ROS基础订阅和发布机制

那么基于ROS1的系统如何相互通信以及ApolloAuto如何利用它？ ROS有[教程](http://wiki.ros.org/ROS/Tutorials)，在分析ApolloAuto模块结构之前，先了解下ROS。


ROS是一款软件，目前仅由Ubuntu系列提供支持， 它有master roscore。

> printenv | grep ROS

默认的ros master uri是“http：// localhost：11311。使用c ++或python通过执行ros :: init创建一个独立的二进制文件，并通过执行ros :: spin \（某种Linux事件循环\）来启动它。 新创建的包的二进制文件称为*** ROS节点***。节点将在Master中注册其名称和IP地址以方便其他节点可以查询到自己。各节点之间通过直接构建TCP连接实现相互通信。

如果节点想要从其他节点读取数据，我们称之为订阅。 典型的格式是
```
... bla bla bla
ros::NodeHandle h;
ros::Subscriber sub = h.subscribe("topic_name", q_size, cb)
.. bla bla bla
```

如果某个节点想要为订阅者提供可读的数据，我们称其为发布者。 典型的格式是
```
... bla bla bla
ros::NodeHandle h;
ros::Publisher pub = h.advertise<generated_msg_format_cls>("topic_name", q_size)
... bla bla bla
```

这里的cb是一个回调函数，当Linux内核IO准备就绪时执行该回调。 了解了这些内容后，在深入核心模块实现之前，我们先快速分析ApolloAuto模块结构。


#### ApolloAuto模块结构

ApolloAuto模块/common/为每一个模块提供了一些基础的宏来控制ros::spin,  / modules / common / adapter包含有最全的关于如何注册topic信息。 每个模块都将从[point](https://github.com/yiakwy/apollo/blob/master/modules/common/adapters/adapter_manager.cc#L50)被注册。 通过阅读$ {MODULE_NAME} / conf的配置文件，可以获得有关模块订阅和发布topic的基本信息。

每个模块首先触发“Init”接口并注册回调。 如果你想在gdb中逐步调试ApolloAuto，请确保在这些回调中添加了断点。同时也说明，如果您不喜欢百度实现，只需重载这些回调即可。

##数据预处理和扩展卡尔曼滤波

卡尔曼滤波是数学交互方法，在不知道整个实时输入序列的情况下收敛到实际估计。 无论您需要处理哪种数据，都可以
依靠卡尔曼滤波。 扩展卡尔曼滤波器用于矩阵格式的3d刚性运动。 这并不难。 我向您推荐美国F15导演的系列教程[Michel van Biezen](https://www.youtube.com/watch?v=CaCcOwJPytQ).

由于它用于输入数据预处理，您可能会在高清地图，感知，规划等模块中看到它。

##部分模块分析

#### HMI & Dreamviewer

它是topic参数可视化化的工具。

HMI是一个基于Flask的简单python应用程序。
它使用Web socket来查询ROS模块应用程序，而不是使用HTTP。 如果您有异步HTTP downloaders的经验，那么很容易理解，HTTP连接只是一个
socket连接文件描述符，我们已经将HTTP头，方法写入该缓冲区。 一旦hmi flask后端收到命令，它将执行一个子进程执行相应的二进制文件。

相比之下，Dreamweaver的工作方式有点像用React，Webpack和Three Js编写的前端应用程序\(WebGL, see /dreamview/backend/simulation_world, /dreamview/frontend/src/render \)，它订阅来自ROS节点的消息，并一帧一帧的绘制。

#### 感知

最初，该模块专门为激光雷达和雷达数据处理实现。 它由AdapterManager注册为ros节点，作为信息融合系统输出观察到的障碍信息。 在最新版本的代码中，ROS节点的不同硬件输入处理器被特意放在/ perception / barriers / onboard中并在不同的平行位置做了相应的实现，包括*激光雷达，雷达，交通信号灯和GPS *。

1.	激光雷达：
    - 高清地图：获取变换矩阵将世界坐标转换为局部坐标并构建地图多边形
    - ROI过滤器：获取ROI并对输入数据执行卡尔曼滤波
    - 分段：加载基于U-Net的\(很多变种\)Caffe模型并对基于来自HD Maps和ROI过滤结果的数据执行正向计算
    - 对象构建：激光雷达返回点\(x，y，z \)。 因此，您需要将它们分组为“障碍物”\（矢量或集合\）
    - 障碍物追踪：百度正在使用谷歌的HM解算器。 对于大型二分图，由于SGD非常简单，通常部署拉格朗日格式的KM算法 。

2.	雷达：
    - 类似于带有传感器的原始\ _障碍信息的激光雷达。
    - ROI过滤器：获取ROI对象并对输入数据执行卡尔曼滤波
    - 对象跟踪器

3.	概率融合\(Apollo 1.5中新增！\)：
    - 据我所知，ApolloAuto中的融合系统
    - 它通常是最重要的部分之一：收集所有信息，并基于主板上的传感器实现最终信息组合，
      用于跟踪列表和基于规则的认知引擎
    - 主要处理工作是关联，因此这里的HM算法再次用作二分图。
    - 跟踪列表按时间戳维护，每个列表将根据概率规则引擎进行更新
