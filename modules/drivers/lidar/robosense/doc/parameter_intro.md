# Parameter Introduction



This document will give you a introduction to each parameter in the configuration file.  



## 1 driver parameter

For convenient, we take rs16.pb.txt as an example.

```yaml
model: "RS16"	
frame_id: "rslidar_16"
ip: "192.168.1.200"
msop_port: 6699
difop_port: 7788
echo_mode: 1
start_angle: 0
end_angle: 360
min_distance: 0
max_distance: 200
cut_angle: 0
pointcloud_channel: "/apollo/sensor/rs16/PointCloud2"
scan_channel: "/apollo/sensor/rs16/Scan"
use_lidar_clock: false
```

*model*

This is the lidar model, since this configuration file is for RS16, we set the model="RS16".



*frame_id*

This is the frame_id value in the lidar messages.



*ip*

This is the lidar ip address. The default value is "192.168.1.200".



*msop_port*

This is the msop port number of lidar. The default value is 6699.



*difop_port*

This is the difop port number of lidar. The default value is 7788.



*echo_mode*

This is the echo_mode of lidar. 



*start_angle*

The start angle of point cloud.



*end_angle*

The end angle of point cloud.



*min_distance*

The minimum distance of point cloud.



*max_distance*

The maximum distance of point cloud.



*cut_angle*

The angle to split frames.



*pointcloud_channel*

The channel name of point cloud.



*scan_channel*

The channel name of scan message.



*use_lidar_clock*

True: The timestamp of lidar messages is the lidar clock.

False: The timestamp of lidar messages is the PC system clock.