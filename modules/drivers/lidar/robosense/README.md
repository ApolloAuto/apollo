# **Robosense LiDAR Driver**



## 1 Introduction

 **robosense**  is the lidar driver kit under Apollo platform. Now support *RS16，RS32，RSBP，RS128* . 



## 2 Run

**All the drivers need to be excuted under Apollo docker environment.**



#### 2.1 RS16

```sh
cyber_launch start /apollo/modules/drivers/lidar/robosense/launch/rs16.launch
```

or

```sh
mainboard -d /apollo/modules/drivers/lidar/robosenseag/rs16.dag
```

Default Channel Name：

- Original point cloud -- /apollo/sensor/rs16/PointCloud2

- Scan--/apollo/sensor/rs16/Scan
- Compensation point cloud -- /apollo/sensor/rs16/compensator/PointCloud2

#### 2.2 RS32

```sh
cyber_launch start /apollo/modules/drivers/lidar/robosense/launch/rs32.launch
```

or

```sh
mainboard -d /apollo/modules/drivers/lidar/robosenseag/rs32.dag

```

Default Channel Name：

- Original point cloud -- /apollo/sensor/rs32/PointCloud2

- Scan--/apollo/sensor/rs32/Scan
- Compensation point cloud -- /apollo/sensor/rs32/compensator/PointCloud2

#### 2.3 RS128

```sh
cyber_launch start /apollo/modules/drivers/lidar/robosense/launch/rs128.launch
```

or

```sh
mainboard -d /apollo/modules/drivers/lidar/robosenseag/rs128.dag
```

Default Channel Name：

- Original point cloud -- /apollo/sensor/rs128/PointCloud2

- Scan--/apollo/sensor/rs128/Scan
- Compensation point cloud -- /apollo/sensor/rs128/compensator/PointCloud2

#### 2.4 RSBP

```sh
cyber_launch start /apollo/modules/drivers/lidar/robosense/launch/rsbp.launch
```

or

```sh
mainboard -d /apollo/modules/drivers/lidar/robosenseag/rsbp.dag
```

Default Channel Name：

- Original point cloud -- /apollo/sensor/rsbp/PointCloud2

- Scan--/apollo/sensor/rsbp/Scan
- Compensation point cloud -- /apollo/sensor/rsbp/compensator/PointCloud2

## 3 Parameters Intro

[Intro to parameters](doc/parameter_intro.md)

