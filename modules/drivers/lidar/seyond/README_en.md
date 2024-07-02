# **Seyond Lidar Driver**

## 1 Introduction

 **seyond** Seyond-Apollo Lidar Driver. support *Falcon/Robin* lidar.

## 2 Run

**All the drivers need to be excuted in Apollo docker environment.**

```sh
cyber_launch start /apollo/modules/drivers/lidar/seyond/launch/seyond.launch
```

or

```sh
mainboard -d /apollo/modules/drivers/lidar/seyond/dag/seyond.dag
```

Default Topic：

- PointCloud -- /apollo/sensor/seyond/PointCloud2"
- Scan -- /apollo/sensor/seyond/Scan

## 3 Parameters Intro
| Parameter          | Default Value | description   |
| :--------:         | :---------:   | :---------:   |
| source_type        | ONLINE_LIDAR | source type: ONLINE_LIDAR/RAW_PACKET   |
| scan_channel       | -            | ScanPackets channel name   |
| pointcloud_channel | -            | PointCloud channel name   |
| frame_id           | seyond       | frame_id   |
| direct_mode        | false        | skip the scan channel and publish pointcloud   |
| aggregate_num      | 50           | aggregate packets num for Scan channel   |
| device_ip          | 172.168.1.10 | lidar ip   |
| port               | 8010         | tcp port   |
| udp_port           | 8010         | udp port, if < 0, use tcp for transmission   |
| reflectance_mode   | true         | false:intensiy mode true:reflectance_mode mode   |
| multiple_return    | 1            | lidar detection echo mode   |
| coordinate_mode    | 3            | convert the xyz direction of a point cloud, 0: lidar-default, 3:WGS-84   |
| max_range          | 2000.0       | point cloud display maximum distance (unit:m)   |
| min_range          | 0.4          | point cloud display minimun distance (unit:m)   |
| log_level          | "info"       | limit log from lidar, can choose from (info warn error)    |
