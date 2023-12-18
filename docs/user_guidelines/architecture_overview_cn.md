# 整体架构

## Apollo 开放平台架构

Apollo 开放平台提供了一个包含车辆认证平台、硬件开发平台、开源软件平台、云端服务平台一体的四层架构。基于此四层架构，开发者可以快速搭建一套属于自己的自动驾驶系统。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/image_e49df47.png)

## 车辆认证平台

车辆认证平台帮助开发者解决 Apollo 自动驾驶系统搭建过程中的线控车辆问题，包括认证线控车辆和开放车辆接口标准两个部分。

### 开放车辆接口标准

开放车辆接口标准提供了 Apollo 线控协议和开发指导手册，开发者可以基于此来开发自己的线控底盘并适配 Apollo 系统。详细 Apollo 线控协议和开放指导手册可以查看 [如何开发适配Apollo系统的底盘](https://developer.apollo.auto/vehicle/certificate_cn.html)。

### 认证线控车辆

认证线控车辆提供了 Apollo 开放平台认证的符合 Apollo 开放车辆接口标准的线控车辆，开发者可以通过购买相应车辆来搭建自动驾驶系统。如您对认证线控车辆感兴趣，可以查看 [Apollo开放车辆认证平台](https://developer.apollo.auto/vehicle/certificate_cn.html)获取更多信息。

## 硬件开发平台

硬件开发平台帮助开发者解决 Apollo 自动驾驶系统搭建过程中的传感器、计算单元等硬件设备问题，其提供了与 Apollo 适配各类硬件设备和驱动。适配的硬件可以分为 Apollo 平台适配与 Apollo 硬件开发平台适配两类。详细信息可以参考 [Apollo硬件开发平台](https://developer.apollo.auto/platform/hardware_cn.html)。

### Apollo 平台适配

Apollo 平台适配硬件是通过了 Apollo 团队软硬件闭环测试，可以为上层软件提供充分支持的硬件。

### Apollo 硬件开发平台适配

Apollo 硬件开发平台适配硬件是指该硬件通过了 Apollo 团队的数据采集测试，但需要开发者自行完成和Apollo 上层软件的闭环测试验证等工作。

## Apollo 开源软件平台

Apollo 开源软件平台提供了自动驾驶车端软件系统框架与技术栈。其包括底层的操作系统，中间层的实时通信框架，以及上层的自动驾驶应用层。

### 底层操作系统

底层操作系统采用 Ubuntu 操作系统，目前支持 Ubuntu 18.04 以上版本。

### 实时通信层

中间实时通信层采用Apollo专为自动驾驶应用设计的 Cyber RT。Apollo Cyber​​ RT 是一个开源、高性能的运行时框架，专为自动驾驶场景而设计。针对自动驾驶的高并发、低延迟、高吞吐量进行了大幅优化。关于Cyber RT 的特点和使用可以参考 [实时通信框架 Cyber RT](<Apollo_Doc_CN_8_0/使用指南/整体架构/实时通信框架 Cyber RT.md>)。

### 自动驾驶应用层

最上层的自动驾驶应用层提供了感知、预测、规划、控制等自动驾驶控制流程核心模块，V2X 协同模块，以及Monitor、Guardian 等功能安全模块。关于以上模块的详细介绍可以参考 [自动驾驶核心模块](Apollo_Doc_CN_8_0/使用指南/整体架构/自动驾驶应用层核心模块.md#自动驾驶核心模块)。同时，在应用层 Apollo 还提供了可视化的人机交互模块 DreamView 人机交互模块，开发者可以通过 DreamView 实现查看当前自动驾驶车辆模块的输出的可视化信息、监控系统软硬件模块状态、调试自动驾驶模块、实时控制车辆模块等能力。

## Apollo云服务平台

Apollo云服务平台提供了自动驾驶研发过程中研发基础设施，提升自动驾驶研发效率。自动驾驶与传统互联网软件研发不同，一是实车测试成本高，二是数据量非常大。而一套能够满足自动驾驶开发流程需求，并提升研发效率的研发基础设施就非常之重要。Apollo云服务平台通过云端的方式解决了数据利用效率的问题，通过与仿真结合降低了实车测试成本，能够极大的提升基于Apollo的自动驾驶研发效率。从研发流程上讲，Apollo车端通过数据采集器生成开放的数据集，并通过云端大规模集群训练生成各种模型和车辆配置，之后通过仿真验证，最后再部署到Apollo车端，无缝连接。这整个过程其实包含了2个迭代循环，一个是模型配置迭代，一个是代码迭代，都通过数据来驱动。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_8_0/image_50def8d.png)

### 数据驱动的模型配置迭代

其主要包括服务于自动驾驶车辆集成、模型训练的 Apollo Fuel 研发云。Apollo Fuel 云提供了车辆标定、传感器标定、控制评测等集成工具服务，同时还提供虚拟车道线制作等工具服务。通过在车辆端的 Fuel Client 智能数据采集器采集数据，驱动云端工具服务生成车辆模型配置，并 OTA 至车端完成迭代闭环。

### 数据驱动的代码迭代

其主要包括服务于开发调试和回归测试的 Apollo Dreamland 仿真云。Apollo Dreamland 仿真云提供了可下载至开发机本地的仿真调试场景，以及云端大规模集群并发测试能力。其通过持续车端数据采集丰富仿真场景，驱动代码不断改进并验证，最终 OTA 至车端实现代码迭代闭环。

更多关于 Apollo 云服务平台的介绍和使用，可以参考 [Apollo Studio 使用文档](https://studio.apollo.auto/Apollo-Homepage-Document/Apollo_Studio/)。

## 文档意见反馈

如果您在使用文档的过程中，遇到任何问题，请到我们在【开发者社区】建立的 [反馈意见收集问答页面](https://studio.apollo.auto/community/article/163)，反馈相关的问题。我们会根据反馈意见对文档进行迭代优化。
