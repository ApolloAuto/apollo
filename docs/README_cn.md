# Apollo3.0 for TX2具体内容说明

## Apollo软件配置

Apollo使用的软件环境大部分基于Ubuntu 14.04，因此需要搭建一个基于arm64/Ubuntu 14.04的环境。NVIDIA官方仅为Jetson TX2提供了Ubuntu 16.04环境，因此我们将使用Docker搭建Apollo能够使用的Ubuntu 14.04环境  
> *Docker 是一个开源的应用容器引擎，让开发者可以打包他们的应用以及依赖包到一个可移植的容器中，然后发布到任何流行的 Linux机器上(需要注意的是，该容器并不能完全实现跨平台架构移植)，也可以实现虚拟化。容器是完全使用沙箱机制，相互之间不会有任何接口。*

Docker Image内存在一个特殊的文件系统，除了提供运行时所需的程序、库、资源、配置等文件外，还包含了为运行时准备的一些配置参数，运行在Docker容器中。基于Docker Image制作的Dev镜像为Apollo开发提供了必须的工具和底层依赖库，通过这些库，可以对Apollo进行编译、运行甚至定制化。

Apollo-Kernel是使用的linux的软实时插件的linux内核，对自动驾驶提供了更专业的调度支持；Apollo是无人驾驶主要的工程部分，包含驱动，环境识别和驾驶算法等主要模块，Apollo编译涉及很多依赖以及驱动编译和安装。

- [安装与配置Docker](specs/\[Jetson_TX2_Platform\]Apollo_3.0_Docker_installation_guide_cn.md)
- [构建Docker Dev镜像](howto/\[Jetson_TX2_Platform\]how_to_build_docker_dev_env_cn.md)
- [源码编译Apollo-Platform](howto/\[Jetson_TX2_Platform\]how_to_build_Apollo-Platform_cn.md)
- [编译Apollo-Kernel](howto/\[Jetson_TX2_Platform\]how_to_build_Apollo-Kernel_cn.md) - *Apollo目前仅对x86提供了实时内核等补丁，因此，该节暂不支持TX2环境的实现*
- [源码编译Apollo3.0](specs/\[Jetson_TX2_Platform\]build_apollo_from_source_cn.md)

## Apollo硬件调试

CAN是实现车辆控制的主要总线和通信协议，本节对其的硬件配置进行描述。Jetson TX2有两个CAN控制器，支持在多个CAN节点间通信。为了使用CAN通信，必须确保CAN收发器与每个控制器连接。CAN收发器是控制器局域网（CAN）协议控制器和物理总线之间的接口。

- [CAN总线通信说明](specs/\[Jetson_TX2_Platform\]communication_with_can_bus_cn.md)
- [Camera调试说明](howto/\[Jetson_TX2_Platform\]howto_config_and_debug_camera_cn.md)

## Apollo运行/调试

编译整个Apollo环境后就基本可以进行脱离实车的离线验证。

- [模拟运行](demo_guide/\[Jetson_TX2_Platform\]run_apollo_3_0_with_simulation_data_guide_cn.md)
- [上车调试](howto/\[Jetson_TX2_Platform\]how_to_debug_on_real_car_cn.md)

## 其他
- [Apollo在TX2平台上问题集锦](FAQs/Jetson_TX2_Platform_FAQs_cn.md)
