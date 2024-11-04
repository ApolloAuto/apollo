## 概述

激光雷达作为自动驾驶平台的主要传感器之一，在感知、定位方面发挥着重要作用。由于激光雷达安装位置不尽相同，在使用前需要求得激光雷达坐标系与无人车坐标系之间的转换关系。在 Apollo 中，求得这种转换关系的方法叫标定，标定有外参标定和内参标定两种。在采购激光雷达时，厂家会提供激光雷达准确的内参，本功能主要实现激光雷达的外参标定方法。

## 模块启动依赖

标定工具依赖定位 Localization 以及 gnss、lidar、camera 驱动，标定前请您确保相关依赖正确安装配置。

## 操作步骤

### 步骤一：选择模式

在 Dreamview+ 页面，选择 **Lidar Calibration** 模式。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_9933735.png)


### 步骤二：环境监测

当前采集车辆环境检查，如上图，定位状态、GPS 状态、Lidar 状态显示为绿色时，表示通过环境检测。单击 下一步。

### 步骤三：初始值确认
1. 确认以下各 Lidar 初始值：
2. 确认无误后，单击 **确认** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_5ffb5d0.png)

   > 注意：初始的外参角度误差不超过正负 20 度，位移误差不超过正负 0.5m。


### 步骤四：动态采集

1. 单击 **开始采集** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_1d37778.png)

2. 在室外场地以 10km/h 绕圈跑满进度条。（时速建议按照自身车型时速采集即可，采集期间进度条 100% 后即可完成，无需人工干预）。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_61c0ba2.png)

   > 说明：采集时需要定位信号好，自然环境中有墙壁或者其他平面物体作为环境特征，采集数据时最好不要有动态移动的物体)。


### 步骤五：标定结果更新

采集完成后，会自动生成文件。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_018da0b.png)

待文件生成后，单击 **完成标定** 。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_db91623.png)


单击 **确认** ，新的标定参数会覆盖车端已经存在的参数。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_ce7663e.png)


