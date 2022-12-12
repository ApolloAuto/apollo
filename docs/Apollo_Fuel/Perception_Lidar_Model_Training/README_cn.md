# 开放感知激光雷达模型的训练服务

## 概述

Open Perception Lidar Model Training Service是一项基于云的服务，利用你的数据，用pointpillars算法来训练感知激光雷达模型，以更好地探测环境中的障碍物。


## 前提条件

- [Apollo](https://github.com/ApolloAuto/apollo) 6.0或更高版本。

- 根据[文档](https://github.com/ApolloAuto/apollo/blob/master/docs/Apollo_Fuel/apply_bos_account_cn.md)注册的百度云BOS服务。

- 在[Apollo Dreamland](http://bce.apollo.auto/user-manual/fuel-service)上的Fuel服务账户。


## 主要步骤

- 数据收集

- 任务提交

- 模型训练结果


## 数据收集

### 数据记录

在不同的场景中收集激光雷达和摄像头的传感器数据，尽可能地覆盖你的自主驾驶环境，请确保场景中有不同类型的障碍物，如行人和车辆。然后使用kitti数据格式对传感器数据进行标注。

### 数据格式

- **我们使用[Kitti数据格式](http://www.cvlibs.net/datasets/kitti/eval_object.php?obj_benchmark=3d)作为训练数据格式**。

```
    INPUT_DATA_PATH:
        training:
            calib
            image_2
            label_2
            velodyne
        testing:
            calib
            image_2
            velodyne
        train.txt
        val.txt
        trainval.txt
        test.txt 
```

- 支持的障碍物检测类别：
  
```
    bus, Car, construction_vehicle, Truck, barrier, Cyclist, motorcycle, Pedestrian, traffic_cone
```

    在标记你的数据时，`类型`必须是上述类别之一（请注意大写字母）。

## 任务提交

### 上传数据到BOS

任务提交的文件夹结构要求：

1. 输入数据路径：上传你的[数据](##数据格式)到INPUT_DATA_PATH目录。

2. 输出数据路径：如果模型训练成功，一个onnx文件将被保存到OUTPUT_DATA_PATH目录中。

### 在Dreamland上提交任务

进入[Apollo Dreamland](http://bce.apollo.auto/)，用**百度**账号登录，选择`Apollo Fuel --> Jobs`，`New Job`，`Perception Lidar Model Training`，并输入[上传数据到BOS](###上传数据到BOS)部分的正确BOS路径。

## 模型训练结果

- 一旦工作完成，你应该收到一封包括结果和`模型路径`的电子邮件。

![](images/perception_email.png)
