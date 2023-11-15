# 测距

## 概述

测量两点之间的距离或多个点组成的折线长度总和，可以用于确定一段路程的长度、计算物体之间的距离、确定两个地点之间的距离等。

## 步骤一：安装并启动 Dreamview+

参见 [播放数据包](Apollo_alpha_doc/安装说明/软件包方式/快速上手/播放数据包.md)。

## 步骤二：选择仿真场景

在 **操作** 中选择 在 **操作** 中选择 **Sim_Control** 或者 **Scenario_Sim**。此处以 **Sim_Control** 为例。

## 步骤三：选择地图与车辆

1. 在 **环境资源** 中选择地图。此处以 **San Mateo** 为例。

2. 在 **自动驾驶系统资源** 中选择车辆。此处以 **Mkz Example** 为例。

   > 注意：车辆为可选操作。

3. 在 **车辆可视化** 面板中，单击 **路由编辑**，进入路径编辑页面。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_b3bef25.png)

   > 注意：您也可以不打开路由编辑，直接点击 **车辆可视化** 面板右下角的测距按钮。

4. 在 **车辆可视化** 面板中，单击测距按钮![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_95d8c03.png)。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_4ee20d6.png)

5. 在编辑界面点一系列点，最后一个点双击或单击右键结束。选完点，每个点（除第一个点外）旁显示与上一个点之间的距离及夹角。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_352b5bb.png)

   最后一个点旁![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_alpha_doc/image_556beae.png)图标，可以清空此次绘制的坐标点序列。
