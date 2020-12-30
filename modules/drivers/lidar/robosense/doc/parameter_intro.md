# Parameter Introduction



This document will give you a introduction to  parameters in the configuration file.  



## Driver parameter

For convenience, we take rs16.pb.txt as an example.

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
split_frame_mode: 1
num_pkts_split: 1
cut_angle: 0
angle_path: "/apollo/modules/drivers/robosense/angle.csv"
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



*start_angle*

The start angle of point cloud.



*end_angle*

The end angle of point cloud.



*min_distance*

The minimum distance of point cloud.



*max_distance*

The maximum distance of point cloud.



*split_frame_mode*

The mode to split the LiDAR frames. Default value is 1.

- 1 -- Spliting frames depends on the cut_angle

- 2 -- Spliting frames depends on a fixed number of packets

- 3 -- Spliting frames depends on num_pkts_split



*num_pkts_split*

The number of packets in one frame. Only be used when split_frame_mode = 3



*cut_angle*

 The angle(degree) to split frames. Only be used when split_frame_mode = 1. The default value is 0.



*angle_path*

The path of the angle calibration file. For latest version of LiDAR, this one can be ignored.



*pointcloud_channel*

The channel name of point cloud.



*scan_channel*

The channel name of scan message.



*use_lidar_clock*

True: The timestamp of lidar messages is the lidar clock.

False: The timestamp of lidar messages is the PC system clock.