# Parameter Introduction



This document will give you a introduction to each parameter in the configuration file.  

For convenient, we take RS16 model as an example.

```yaml
model: "RS16"	
ip: "192.168.1.200"
msop_port: 6699
difop_port: 7788
echo_mode: 1
start_angle: 0
end_angle: 360
min_distance: 0
max_distance: 200
cut_angle: 0
split_frame_node: 1
num_pkts_split: 0
use_lidar_clock: false
```

**model**

This is the lidar model, since this configuration file is for RS16, we set the model="RS16".



**frame_id**

This is the frame_id value in the lidar messages.



**ip**

This is the lidar ip address. The default value is "192.168.1.200".



**msop_port**

This is the msop port number of lidar. The default value is 6699.



**difop_port**

This is the difop port number of lidar. The default value is 7788.



**echo_mode**

This is the echo_mode of lidar. 



**start_angle**

The start angle of point cloud.



**end_angle**

The end angle of point cloud.



**min_distance**

The minimum distance of point cloud.



**max_distance**

The maximum distance of point cloud.



**cut_angle**

The angle to split frames.

**split_frame_node**

The mode to split the LiDAR frames. Default value is ```1```

  - 1 -- Spliting frames depending on the cut_angle
  - 2 -- Spliting frames depending on a fixed number of packets
  - 3 -- Spliting frames depending on num_pkts_split

**pointcloud_channel**

The channel name of point cloud.

**scan_channel**

The channel name of scan message.



**use_lidar_clock**

  - True: The timestamp of lidar messages is the lidar clock.

  - False: The timestamp of lidar messages is the PC system clock.