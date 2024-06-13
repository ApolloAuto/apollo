# **Vanjee LiDAR Driver**

## 1 Introduction

 **vanjee**  is the lidar driver kit under Apollo platform. Now support *WLR-720* . 


## 2 Run

**All the drivers need to be excuted in Apollo docker environment.**

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

