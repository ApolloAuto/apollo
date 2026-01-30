## 概述

通过在地图上绘制一系列点，表示起点和终点之间的路径，从而方便地将数据输入到规划算法中，并可视化路径规划的输出结果。

## 步骤一：安装并启动 Dreamview+

参见 [包管理安装方式](../安装指南/包管理安装方式.md) 文档。

## 步骤二：选择仿真模式

在 **操作** 中选择 **Sim_Control** 或者 **Scenario_Sim** 。此处以 **Sim_Control** 为例。

## 步骤三：选择地图和车辆

1. 在 **环境资源** 中选择地图。此处以 **San Mateo** 为例。

2. 在 **自动驾驶系统资源** 中选择车辆。此处以 **Mkz Example** 为例。

> 注意：车辆为可选操作。

## 步骤四：开启轨迹绘制

1. 在 **车辆可视化** 面板中单击 **路由编辑** 开启路由编辑功能。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_f5da9de.png)

2. 单击左上角初始位置图标，在地图上单击设定初始位置。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_904b988.png)

   > 注意：长按鼠标右键可以拖拽移动地图位置。

3. 单击左上角轨迹点图标，在地图上单击添加途径轨迹点。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_17fbcca.png)

   > 注意：长按拖拽可以修改初始位置和轨迹点的方向。

4. 单击 **保存编辑** ，完成轨迹绘制。

## 步骤五：开启模块，查看规划效果

开启 Planning 和 Prediction 模块，并打开底部操作栏的播放按钮，查看规划效果。

![5d25fa56f2b6054864a44150c234e487.jpg](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/5d25fa56f2b6054864a44150c234e487_5d25fa5.jpg)
