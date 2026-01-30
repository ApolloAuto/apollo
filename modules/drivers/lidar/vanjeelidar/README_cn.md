# **Vanjee LiDAR Driver**

## 1 工程简介

 **vanjee** 为万集科技在阿波罗平台上的雷达驱动集成包。 目前支持*vanjee_720_16"、"vanjee_720_32*型号的雷达。 

## 2 运行

**所有驱动均需要在Apollo Docker环境下运行**

```sh
cyber_launch start /apollo/modules/drivers/lidar/vanjeelidar/launch/vanjeelidar.launch
```

或

```sh
mainboard -d /apollo/modules/drivers/lidar/vanjeelidar/dag/vanjeelidar.dag
```

默认话题名：

- 原始点云 -- /apollo/sensor/vanjeelidar/up/PointCloud2"
- Scan--/apollo/sensor/vanjeelidar/up/Scan

## 3 参数介绍

[参数介绍](doc/parameter_intro.md)
