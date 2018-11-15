
# lslidar_c16
## preparation
### IP address
The default IP address for lslidar_c16 is 192.168.1.200, port number is 2368.
The default IP address for PC client is 192.168.1.102, port number is 2368.
These parameters can be configured by using lslidar_c16 configuration software tool.

## ROS
lslidar_c16 driver runs as node, including:
1. data packet processing --> /lslidar_driver_node
2. point cloud generation --> /lslidar_decoder_node
3. compensation --> /lslidar_compensator_node

lslidar_c16 driver runs as nodelet, including:
1. data packet processing --> /lslidar_driver_nodelet
2. point cloud generation --> /lslidar_decoder_nodelet
3. compensation --> /lslidar_compensator_nodelet

Compensation relies on `tf` to query the coordination transform, so gnss_driver is required to run the lslidar nodelets.

### decoder nodelet
roslaunch lslidar_decoder lslidar_c16_apollo.launch

lslidar_decoder runs as nodelet without motion compensation. The lslidar_driver already includes in this launch file, so you don't need to run lslidar_driver alone
Default nodelet manager name is: lslidar_nodelet_manager
Output
frame_id: lslidar
rostopic: /apollo/sensor/lslidarC16/PointCloud2
rosmsg: sensor_msgs/PointCloud2

### compensator nodelet
roslaunch lslidar_compensator lslidar_compensator_nodelet.launch

lslidar_compensator runs as nodelet with motion compensation. The lslidar_driver and lslidar_decoder already includes in this launch file, so you don't need to run they alone

* But it need a GNSS driver to publish a TF between world_frame_id ("world") and chile_frame_id ("lslidar") *

Default nodelet manager name is: lslidar_nodelet_manager
Output
1. PointCloud from lidar without motion compensator
	rostopic: /apollo/sensor/lslidar/PointCloud2
	rosmsg: sensor_msgs/PointCloud2

2. PointCloud obtained from lidar with motion compensator
	rostopic: /apollo/sensor/lslidar/compensator/PointCloud2
	rosmsg: sensor_msgs/PointCloud2


### Topics
* /apollo/sensor/lslidar/LslidarPacket --> lslidar_msgs/LslidarPacket
* /apollo/sensor/lslidar/PointCloud2 --> sensor_msgs/PointCloud2
* /apollo/sensor/lslidar/compensator/PointCloud2 --> sensor_msgs/PointCloud2
 
### TF frame
* world
* lslidar

### Build Velodyne

```bash
# in dev docker
cd /apollo
bash apollo.sh build_lslidar
```
The output will overwrite the lslidar driver in `/home/tmp/ros/`.
 


### Start lslidar_c16 Driver
**Please change the parameters in the launch file when you start**
```bash
# with GNSS compensator
roslaunch lslidar_compensator lslidar_compensator_nodelet.launch
# or
# without GNSS compensator
roslaunch lslidar_decoder lslidar_c16_apollo.launch
```


### FAQ
1. 'lslidar port 2368 poll() timeout'
	The network between the host and lslidar_c16 is having problem. Please use `sudo tcpdump -i eth0 udp port 2368` to check if lidar_c16 packets with 1206 length data are received.
2. 'cannot find transform ...'
	`Compensaton` relies on `tf`, please double check if `gnss_driver` has been started, and also use `rostopic echo /tf` to check if there are any message in the tf.

