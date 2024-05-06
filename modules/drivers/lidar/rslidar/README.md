# **Robosense LiDAR Driver**

## 1 Introduction

 **robosense**  is the lidar driver kit under Apollo platform. Now support *RS16，RS32，RSBP，RS128, RS80, RSHELIOS, RSM1* . 


## 2 Run

**All the drivers need to be excuted in Apollo docker environment.**

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

## 3 Parameters Intro

[Intro to parameters](doc/parameter_intro.md)
