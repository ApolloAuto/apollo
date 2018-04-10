
## Pandora
pandora driver, including:
1. data packet processing --> /apollo/sensor/pandora/hesai40/PointCloud2
3. compensation --> /apollo/sensor/pandora/hesai40/compensator/PointCloud2
 
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
First, specify the parameters of the car.

> Notice: Only for hesai40

Intrinsic parameter file resides in lidar device, pandora driver will try to read from device :

**extrinsic parameter file**

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
 
