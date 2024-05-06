# **Robosense LiDAR Driver**

## 1 工程简介

 **robosense** 为速腾聚创在阿波罗平台上的雷达驱动集成包。 目前支持*RS16，RS32，RSBP，RS128, RS80, RSHELIOS, RSM1*等型号的雷达。 

## 2 运行

**所有驱动均需要在Apollo Docker环境下运行**

```sh
cyber_launch start /apollo/modules/drivers/lidar/rslidar/launch/rslidar.launch
```

或

```sh
mainboard -d /apollo/modules/drivers/lidar/rslidar/dag/rslidar.dag
```

默认话题名：

- 原始点云 -- /apollo/sensor/rslidar/up/PointCloud2"
- Scan--/apollo/sensor/rslidar/up/Scan

## 3 参数介绍

[参数介绍](doc/parameter_intro.md)
