# **Lslidar LiDAR Driver**



## 1. Overview

**lslidar** is a lidar driver integration package from LenShen Intelligent System Co. Ltd. for the Apollo 8.0 platform. Currently the *C16, C32, CH16, CH32, CH64, CH64w, CH120, CH128,CH128X1* lidar models are supported. 



##  2. Example of driver running

#### 2.1 C16

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidar16.launch
or
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidar16.dag

Default topic name:

- Raw point cloud -- /apollo/sensor/lslidar16/PointCloud2
- Scan--/apollo/sensor/lslidar16/Scan
- Point cloud after motion compensation -- /apollo/sensor/lslidar16/compensator/PointCloud2


####  2.2 C32

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidar32.launch
or
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidar32.dag

Default topic name:

- Raw point cloud -- /apollo/sensor/lslidar32/PointCloud2
- Scan--/apollo/sensor/lslidar32/Scan
- Point cloud after motion compensation -- /apollo/sensor/lslidar32/compensator/PointCloud2


#### 2.3 N401(delete)

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidar401.launch
or
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidar401.dag

Default topic name:

- Raw point cloud -- /apollo/sensor/lslidar401/PointCloud2
- Scan--/apollo/sensor/lslidar401/Scan

#### 2.4 CH16

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH16.launch
or
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH16.dag

Default topic name:

- Raw point cloud -- /apollo/sensor/lslidarCH16/PointCloud2
- Scan--/apollo/sensor/lslidarCH16/Scan
- Point cloud after motion compensation -- /apollo/sensor/lslidarCH16/compensator/PointCloud2

#### 2.5 CH32

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH32.launch
or
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH32.dag

Default topic name:

- Raw point cloud -- /apollo/sensor/lslidarCH32/PointCloud2
- Scan--/apollo/sensor/lslidarCH32/Scan
- Point cloud after motion compensation -- /apollo/sensor/lslidarCH32/compensator/PointCloud2

#### 2.6 CH64

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH64.launch
or
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH64.dag

Default topic name:

- Raw point cloud -- /apollo/sensor/lslidarCH64/PointCloud2
- Scan--/apollo/sensor/lslidarCH64/Scan
- Point cloud after motion compensation -- /apollo/sensor/lslidarCH64/compensator/PointCloud2

#### 2.7 CH64w

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH64w.launch
or
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH64w.dag

Default topic name:

- Raw point cloud -- /apollo/sensor/lslidarCH64w/PointCloud2
- Scan--/apollo/sensor/lslidarCH64w/Scan
- Point cloud after motion compensation -- /apollo/sensor/lslidarCH64w/compensator/PointCloud2

#### 2.8 CH120

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH120.launch
or
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH120.dag

Default topic name:

- Raw point cloud -- /apollo/sensor/lslidarCH120/PointCloud2
- Scan--/apollo/sensor/lslidarCH120/Scan
- Point cloud after motion compensation -- /apollo/sensor/lslidarCH120/compensator/PointCloud2

#### 2.9 CH128

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH128.launch
or
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH128.dag

Default topic name:

- Raw point cloud -- /apollo/sensor/lslidarCH128/PointCloud2
- Scan--/apollo/sensor/lslidarCH128/Scan
- Point cloud after motion compensation -- /apollo/sensor/lslidarCH128/compensator/PointCloud2

#### 2.10 CH128X1

cyber_launch start /apollo/modules/drivers/lidar/lslidar/launch/lslidarCH128X1.launch
or
mainboard -d /apollo/modules/drivers/lidar/lslidar/dag/lslidarCH128X1.dag

Default topic name:

- Raw point cloud -- /apollo/sensor/lslidarCH128X1/PointCloud2
- Scan--/apollo/sensor/lslidarCH128X1/Scan
- Point cloud after motion compensation -- /apollo/sensor/lslidarCH128X1/compensator/PointCloud2



### Frequently asked questions

-  The printout "[lslidar]lslidar poll() timeout, port: 2368" indicates that the computing platform is not connected to the lidar network.

  You can use Wireshark to see if lidar data is available; and if the lidar IP and port number parameters are set correctly.
  
  
