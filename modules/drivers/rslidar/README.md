
## RS-LiDAR
rslidar driver runs as nodelet, including:
1. data packet processing --> /sensor_rslidar_driver
2. point cloud generation --> /sensor_rslidar_convert

### Topics
* /apollo/sensor/rslidar/rslidarScan --> rslidar_msgs/rslidarScan
* /apollo/sensor/rslidar/PointCloud2 --> sensor_msgs/PointCloud2

### Coordination
* rslidar

### Build RS-LiDAR

```bash
# in dev docker
cd /apollo
bash apollo.sh build_rslidar
```
The output will overwrite the rslidar driver in `/home/tmp/ros/`.


### Configure RS-LiDAR Driver
**rslidar model**
```xml
<!-- RS16/RS32 -->
<arg name="model" default="RS16" />
```
**intrinsic calibration parameters path**
```xml
<arg name="curves_path" default="$(find rslidar_pointcloud)/data/rs_lidar_16/curves.csv"/>
<arg name="angle_path" default="$(find rslidar_pointcloud)/data/rs_lidar_16/angle.csv"/>
<arg name="channel_path" default="$(find rslidar_pointcloud)/data/rs_lidar_16/ChannelNum.csv"/>
```

### Start RS-LiDAR Driver
**Please change the parameters in the launch file for cars when you start**
```bash
roslaunch rslidar start_rslidar.launch
# or
bash /apollo/scripts/start_rslidar.sh # this file has no effect in reality
```

### FAQ
1. 'RSLIDAR port 6699 poll() timeout'
	The network between the host and rslidar is having problem. Please use `sudo tcpdump -i eth0 udp port 6699` to check if rslidar packets are received.
