## 概述

动力学标定是通过采集车辆底盘油门踏板量、刹车踏板量、车辆速度、加速度作为车辆纵向动力模型的输入，通过机器学习的方式，生成相应车辆的踏板标定表。生成的标定表数据量大、精度高，提高车辆的控制精度。

## 前提条件

- 完成鉴权工具的安装。
- 开通标定服务且鉴权成功。


## 模块启动依赖

标定工具依赖定位 Localization、GPS 以及 Caubus 模块，标定前请您确保相关模块启动正常。因为动力学标定依赖tensorflow，请执行以下命令安装tensorflow环境。
```bash
pip3 show tensorflow >/dev/null || pip3 install tensorflow==2.11.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
```
## 操作步骤

### 步骤一：参数确认

1. 在 **Mode** 中，选择 **Dynamics Calibration**。
2. 单击 **Setup All**，开启 **Canbus**，**GPS**，**Localization** 模块。

4. 确认配置文件参数，并单击 **下一步**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_bb6b520.png)

    >说明：
    >- 配置文件确认检查，在标定配置文件中进行车辆speedmax、brakemax的确认；车辆参数中要保证brake_deadzone数值要小于breakmin,throttle同理。
    >- 参数配置文件路径为`/apollo/modules/dreamview_plus_plugins/vehicle_calibration/conf/vehicle_calibration_conf.pb.txt`，您可以根据需要修改配置文件中的参数。
    
    ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_4a0881a.png)
    
    速度：
    - speed_min：最小速度
    - speed_max：最大速度
    
    油门：
    - throttle_min：最小油门
    - throttle_max：最大油门
    
    刹车：
    - brake_min：最小刹车
    - brake_max：最大刹车
    
    采集百分比：
    - min_collection_percentage_to_calib：允许计算结果的最小进度。

### 步骤二：环境监测

当前采集车辆环境中，Canbus 状态， GPS 状态和 Localization 状态显示绿色标志时，单击 **下一步**。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_2119922.png)

### 步骤三：数据采集

1. 单击 **开始采集**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_0ea9354.png)

   >说明：选择平直宽阔的道路，手动驾驶或遥控车辆尽量保持直线行驶，如有转弯，将会延长采集时间。
  
   在驾驶过程中，按标定条件进行速度、油门、刹车的操作（大油门大刹车，大油门小刹车，小油门小刹车，小油门大刹车等），预计需要 20～40 分钟。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_f94c00d.png)
   
   >说明：浅色部分为采集的初始图标，相同的数据采集后颜色会加深，采集的数值越多，颜色越深；采满帧数后，进度条会显示100%。
   
2. 待数据采集完成，单击 **结束采集**。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_62b09c5.png)
   
   
   >说明：采集进度达到80%（此数值可通过配置文件修改），自动完成采集。

   

### 步骤四：生成标定表

等待标定表生成完毕。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_eca3a8f.png)

### 步骤五：标定表写入配置文件


1. 单击 **参数替换**，并单击 **确认**，将标定表文件更新到配置文件中。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_59b54d7.png)

   >注意：
   >- 需要保持格式正确，否则会导致 Control 模块不能启动。
   >- 在得到刹车标定表三维图后，需要检查标定表合理性。
   >- 对于油门标定表，标定表应该尽量为一个平面。
   >- 如果自动驾驶速度上限远小于车辆最高车速，在自动驾驶全速域内都近似具有相同油门开度对应相同加速度的特性。如果自动驾驶速度上限接近车辆速度上限，则可能在接近速度上限时加速度较小。
   >- 对于刹车标定表，应该确保任何速度下，更大的刹车力都对应更大的减速度绝对值。如果出现较大刹车力反而对应较小的减速度绝对值，则刹车可能有热衰等物理问题。

   显示参数替换成功。
   
   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Studio/image_3ca3f85.png)

 
   
    
