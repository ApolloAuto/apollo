# **Lslidar LiDAR Driver**



## 1、 工程简介

 **lslidar** 为镭神在Apollo 8.0平台的雷达驱动集成包。 目前支持*C16，C32，CH16，CH32，CH64，CH64w，CH120，CH128,CH128X1*等型号的雷达。 



##  2、 驱动运行示例

#### 2.1 C16

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidar16.launch
或
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidar16.dag

默认话题名：

- 原始点云 -- /apollo/sensor/lslidar16/PointCloud2
- Scan--/apollo/sensor/lslidar16/Scan
- 运动补偿后点云 -- /apollo/sensor/lslidar16/compensator/PointCloud2


####  2.2 C32

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidar32.launch
或
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidar32.dag

默认话题名：

- 原始点云 -- /apollo/sensor/lslidar32/PointCloud2
- Scan--/apollo/sensor/lslidar32/Scan
- 运动补偿后点云 -- /apollo/sensor/lslidar32/compensator/PointCloud2


#### 2.3 N401(删除)

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidar401.launch
或
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidar401.dag

默认话题名：

- 原始点云 -- /apollo/sensor/lslidar401/PointCloud2
- Scan--/apollo/sensor/lslidar401/Scan

#### 2.4 CH16

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH16.launch
或
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH16.dag

默认话题名：

- 原始点云 -- /apollo/sensor/lslidarCH16/PointCloud2
- Scan--/apollo/sensor/lslidarCH16/Scan
- 运动补偿后点云 -- /apollo/sensor/lslidarCH16/compensator/PointCloud2

#### 2.5 CH32

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH32.launch
或
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH32.dag

默认话题名：

- 原始点云 -- /apollo/sensor/lslidarCH32/PointCloud2
- Scan--/apollo/sensor/lslidarCH32/Scan
- 运动补偿后点云 -- /apollo/sensor/lslidarCH32/compensator/PointCloud2

#### 2.6 CH64

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH64.launch
或
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH64.dag

默认话题名：

- 原始点云 -- /apollo/sensor/lslidarCH64/PointCloud2
- Scan--/apollo/sensor/lslidarCH64/Scan
- 运动补偿后点云 -- /apollo/sensor/lslidarCH64/compensator/PointCloud2

#### 2.7 CH64w

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH64w.launch
或
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH64w.dag

默认话题名：

- 原始点云 -- /apollo/sensor/lslidarCH64w/PointCloud2
- Scan--/apollo/sensor/lslidarCH64w/Scan
- 运动补偿后点云 -- /apollo/sensor/lslidarCH64w/compensator/PointCloud2

#### 2.8 CH120

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH120.launch
或
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH120.dag

默认话题名：

- 原始点云 -- /apollo/sensor/lslidarCH120/PointCloud2
- Scan--/apollo/sensor/lslidarCH120/Scan
- 运动补偿后点云 -- /apollo/sensor/lslidarCH120/compensator/PointCloud2

#### 2.9 CH128

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH128.launch
或
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH128.dag

默认话题名：

- 原始点云 -- /apollo/sensor/lslidarCH128/PointCloud2
- Scan--/apollo/sensor/lslidarCH128/Scan
- 运动补偿后点云 -- /apollo/sensor/lslidarCH128/compensator/PointCloud2

#### 2.10 CH128X1

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH128X1.launch
或
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH128X1.dag

默认话题名：

- 原始点云 -- /apollo/sensor/lslidarCH128X1/PointCloud2
- Scan--/apollo/sensor/lslidarCH128X1/Scan
- 运动补偿后点云 -- /apollo/sensor/lslidarCH128X1/compensator/PointCloud2



### 常见问题

-  打印“[lslidar]lslidar poll() timeout, port: 2368‘’ 表示计算平台与雷达网络不通。

  可用wireshark查看是否有雷达数据；以及雷达ip和端口号参数设置是否正确。
