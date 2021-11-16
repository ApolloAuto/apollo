# **Robosense LiDAR Driver**



## 1 工程简介

 **robosense** 为速腾聚创在阿波罗平台上的雷达驱动集成包。 目前支持*RS16，RS32，RSBP，RS128*四种型号的雷达。 



## 2 运行

**所有驱动均需要在Apollo Docker环境下运行**

#### 2.1 RS16

```sh
cyber_launch start /apollo/modules/drivers/lidar/robosense/launch/rs16.launch
```

或

```sh
mainboard -d /apollo/modules/drivers/lidar/robosenseag/rs16.dag
```

默认话题名：

- 原始点云 -- /apollo/sensor/rs16/PointCloud2

- Scan--/apollo/sensor/rs16/Scan
- 运动补偿后点云 -- /apollo/sensor/rs16/compensator/PointCloud2

#### 2.2 RS32

```sh
cyber_launch start /apollo/modules/drivers/lidar/robosense/launch/rs32.launch
```

或

```sh
mainboard -d /apollo/modules/drivers/lidar/robosenseag/rs32.dag

```

默认话题名：

- 原始点云 -- /apollo/sensor/rs32/PointCloud2

- Scan -- /apollo/sensor/rs32/Scan
- 运动补偿后点云 -- /apollo/sensor/rs32/compensator/PointCloud2

#### 2.3 RS128

```sh
cyber_launch start /apollo/modules/drivers/lidar/robosense/launch/rs128.launch
```

或

```sh
mainboard -d /apollo/modules/drivers/lidar/robosenseag/rs128.dag
```

默认话题名：

- 原始点云 -- /apollo/sensor/rs128/PointCloud2

- Scan--/apollo/sensor/rs128/Scan
- 运动补偿后点云 -- /apollo/sensor/rs128/compensator/PointCloud2

#### 2.4 RSBP

```sh
cyber_launch start /apollo/modules/drivers/lidar/robosense/launch/rsbp.launch
```

或

```sh
mainboard -d /apollo/modules/drivers/lidar/robosenseag/rsbp.dag
```

默认话题名：

- 原始点云 -- /apollo/sensor/rsbp/PointCloud2

- Scan--/apollo/sensor/rsbp/Scan
- 运动补偿后点云 -- /apollo/sensor/rsbp/compensator/PointCloud2



## 3 参数介绍

[参数介绍](doc/parameter_intro.md)

