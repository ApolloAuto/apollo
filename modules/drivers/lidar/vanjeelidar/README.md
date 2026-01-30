# **Vanjee LiDAR Driver**

## 1 Introduction

 **vanjee**  is the lidar driver kit under Apollo platform. Now support *vanjee_720_16","vanjee_720_32*.


## 2 Run

**All the drivers need to be excuted in Apollo docker environment.**

```sh
cyber_launch start /apollo/modules/drivers/lidar/vanjeelidar/launch/vanjeelidar.launch
```

or

```sh
mainboard -d /apollo/modules/drivers/lidar/vanjeelidar/dag/vanjeelidar.dag
```

default topic name：

- raw point cloud -- /apollo/sensor/vanjeelidar/up/PointCloud2"
- Scan--/apollo/sensor/vanjeelidar/up/Scan

## 3 Parameters Intro

[Intro to parameters](doc/parameter_intro.md)
