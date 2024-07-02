# **Seyond Lidar Driver**

## 1 工程简介

 **seyond** 图达通-Apollo平台-激光雷达驱动。 支持*Falcon/Robin*系列雷达。 

## 2 运行

**所有驱动均需要在Apollo Docker环境下运行**

```sh
cyber_launch start /apollo/modules/drivers/lidar/seyond/launch/seyond.launch
```

或

```sh
mainboard -d /apollo/modules/drivers/lidar/seyond/dag/seyond.dag
```

默认话题名：

- 点云 -- /apollo/sensor/seyond/PointCloud2"
- Scan -- /apollo/sensor/seyond/Scan

## 3 参数介绍
| 参数                | 默认值        | 描述  |
| :--------:         | :---------:  | :---------:   |
| source_type        | ONLINE_LIDAR | 源类型: ONLINE_LIDAR/RAW_PACKET   |
| scan_channel       | -            | Scan channle名称   |
| pointcloud_channel | -            | 点云 channel名称   |
| frame_id           | seyond       | frame_id   |
| direct_mode        | false        | 跳过Scan通道，直接发布点云模式   |
| aggregate_num      | 50           | Scan通道发布聚合包数量   |
| device_ip          | 172.168.1.10 | 雷达 ip   |
| port               | 8010         | tcp 端口   |
| udp_port           | 8010         | udp端口，若<0, 使用tcp传输   |
| reflectance_mode   | true         | false:强度模式 true:反射模式   |
| multiple_return    | 1            | 回波模式   |
| coordinate_mode    | 3            | 坐标转换模式, 0: 雷达默认配置, 3:WGS-84   |
| max_range          | 2000.0       | 最大距离限制 (单位:m)   |
| min_range          | 0.4          | 最小距离限制 (单位:m)   |
| log_level          | "info"       | 日志等级限制 (info warn error)    |
