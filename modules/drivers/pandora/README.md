
## Pandora
pandora driver, including:
1. data packet processing --> /apollo/sensor/pandora/hesai40/PointCloud2
2. compensation --> /apollo/sensor/pandora/hesai40/compensator/PointCloud2
 
Compensation relies on `tf` to query the coordination transform, so gnss_driver is required to run the pandora driver.
 
### Topics
* /apollo/sensor/pandora/hesai40/PointCloud2 --> sensor_msgs/PointCloud2
* /apollo/sensor/pandora/hesai40/compensator/PointCloud2 --> sensor_msgs/PointCloud2
 
### Coordination
* world
* novatel
* hesai40
 
### Build Pandora 

```bash 
# in dev docker
cd /apollo
bash apollo.sh build_pandora
```
The output will overwrite the pandora driver in `/home/tmp/ros/`.
 
### Configure Pandora Driver
First, specify the parameters of the car, modify them in launch file.

> Notice: Only for hesai40

**Intrinsic parameter file**
Intrinsic parameter file resides in lidar device, pandora driver will try to read from device :

**Other configs**
```xml
<arg name="pandora_ip" default="192.168.20.51"/>
<arg name="lidar_recv_port"  default="2368"/>
<arg name="gps_recv_port"  default="10110"/>
<arg name="start_angle"  default="135"/>
<arg name="pandora_camera_port"  default="9870"/>
<arg name="enable_camera"  default="true"/>
<arg name="timezone"  default="8"/>
<arg name="frame_id"  default="hesai40"/>
```

### Start Pandora Driver
**Please change the parameters in the launch file for cars when you start**
```bash
roslaunch pandora_driver pandora_driver.launch
roslaunch pandora_pointcloud compensator_node.launch
# or
bash /apollo/scripts/pandora.sh # this file has no effect in reality
```
### FAQ
1. 'no message of /apollo/sensor/pandora/hesai40/PointCloud2 topic'
	The network between the host and pandora is having problem. Please use `sudo tcpdump -i eth0 udp port 2368` to check if pandora packets are received.
2. 'cannot find transform ...'
	`Compensaton` relies on `tf`, please double check if `gnss_driver` has been started, and also use `rostopic echo /tf` to check if there are any message in the tf.
 
