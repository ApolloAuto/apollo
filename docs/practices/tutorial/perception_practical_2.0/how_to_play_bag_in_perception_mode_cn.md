# 如何在感知模式下播包

## 前提条件

播放数据包前，您需要先下载 Dreamview+ 插件，启动并打开 Dreamview+，然后再进行播包操作。您可以通过源码和包管理两种方式进入 docker 环境、安装、启动并打开 Dreamview+ 插件，详情参见 [启动 Dreamview+ 步骤一、步骤二](Apollo_alpha_doc/安装说明/软件包方式/快速上手/播放数据包.md)。

## 方式一：通过 Dreamview+ 播包

1. 选择感知模式 **Perception**。
1. 下载数据包。

   在 **Resource Manager** > **Records** 中先下载需要的感知数据包。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_2eda926.png)

1. 设置播包参数。

   - 在 **Mode** 下拉框中选择 **Perception**。

   - **Operation** 中选择 **Record**。

   - 在 **Environment Resources** 中，选择 **Records**，并选择具体想要播放的数据包 **sensor_rgb**。
   - 在 **Environment Resources** 中选择对应的 **HDMap**，此处数据包对应的地图为 **Sunnyvale Big Loop**。

1. 添加面板。

   在左侧选择 **Add Panel**，并选择需要的面板，如单击添加相机视图 **Camera View**、点云视图 **Point Cloud**、模块延时面板 **Module Delay** 和 车辆可视化面板 **Vehicle Visulization**。

1. 单击底部区域播放按钮。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_80d5943.png)

   可以在 **Vehicle Visualization**、**Camera View** 和 **Point Cloud** 面板中看到数据包播放的画面。

## 方式二：通过命令行播包

1. 在 docker 环境中启动并打开 Dreamview+ 可视化工具。

   您可以通包管理和源码两种方式进入 docker。关于如何启动并打开 Dreamview+，参见 [启动 Dreamview+](Apollo_alpha_doc/安装说明/软件包方式/快速上手/播放数据包.md)。

2. 下载数据包。

   在 Dreamview+ 的 **Resource Manager** > **Records** 中先下载需要的感知数据包。此处以 **demo2_mkz_210_sunnybigloop** 为例。

3. 重新开启一个终端，并进入 Apollo Docker 环境后，输入以下命令播包：

   ```bash
   cyber_record play -f  ~/.apollo/resources/records/demo2_mkz_210_sunnybigloop -l
   ```
