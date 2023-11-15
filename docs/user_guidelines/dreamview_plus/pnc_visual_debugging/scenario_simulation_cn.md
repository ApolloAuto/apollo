# 场景仿真

## 概述

用于运行并监测规控算法效果，可设置规划、控制、路由算法的开启与关闭，从云端同步场景后运行，为场景添加不同行驶轨迹，可在运行过程中可查看模块时延、控制台日志、通过监控模块查看 PnC 算法的数据记录，以及各通道内报文信息。

## 步骤一：安装并启动 Dreamview+

参见 [播放数据包](Apollo_alpha_doc/安装说明/软件包方式/快速上手/播放数据包.md)。

## 步骤二：选择模式

1. 在左侧导航栏选择 **模式设置**。
2. 在 **模式** 中选择 **PnC** 模式。
3. 在 **操作** 中选择 **Scenario_Sim**。

## 步骤三：下载场景集

1. 在左侧导航栏选择 **资源管理**。
2. 单击 **场景**，从 Apollo Studio 云端下载需要的场景集。
3. 单击 **模式设置** > **环境资源** > **场景**，在本地场景列表中选择要运行的场景。

## （可选）步骤四：选择车辆

在 **自动驾驶系统资源** 中选择 **车辆**。

## 步骤五：开启模块

在 **模块** 中，开启 **Planning**、**Prediction** 模块。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_e21b23e.png)

## 步骤六：轨迹绘制

1. 在 **车辆可视化** 面板单击 **路由编辑**。
2. 单击 ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_4d04326.png) 图标添加初始点。
3. 单击 ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_a559070.png) 图标添加轨迹点，绘制完成后单击 **保存编辑**。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_804891a.png)

## 步骤七：运行仿真

单击底栏播放按钮，运行仿真。

## 步骤八：查看数据

1. 在左侧导航栏单击 **添加面板**。
2. 添加 **PnC监控** 面板、**模块延时**、**控制台** 面板，查看模块延时、控制台信息。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_ea6a228.png)

## 步骤九：调试修改代码

根据您自己的需求进行代码调试。
