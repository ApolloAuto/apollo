## Velodyne
velodyne driver runs as component, including:
1. data packet processing --> /driver
2. point cloud generation --> /convert
3. velodyne 16 fusion --> /fusion
4. compensation --> /compensator

Compensation relies on `tf` to query the coordination transform, so gnss_driver is required to run the velodyne components.

### Output Channels
1. data packet
  channel: /apollo/sensor/lidar128/Scan
  type: apollo::drivers::velodyne::VelodyneScan
  proto: [modules/drivers/velodyne/proto/velodyne.proto](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/velodyne/proto/velodyne.proto)
2. point cloud
  channel: /apollo/sensor/lidar128/PointCloud2
  type: apollo::drivers::PointCloud
  proto: [modules/drivers/proto/pointcloud.proto]https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/pointcloud.proto
3. compensation point cloud
  channel: /apollo/sensor/lidar128/compensator/PointCloud2
  type: apollo::drivers::PointCloud
  proto: [modules/drivers/proto/pointcloud.proto]https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/pointcloud.proto

### Coordination
* world
* novatel
* velodyne128

### Start
**Please change the parameters in the launch file for cars when you start**
```bash
#in docker
cyber_launch start velodyne.launch
```

### FAQ
1. "basetime is zero"
  The position pack is not valid, please check gps signal.
2. 'velodyne port 2368 poll() timeout'
  The network between the host and velodyne is having problem. Please use `sudo tcpdump -i eth0 udp port 2368` to check if velodyne packets are received.
3. 'cannot find transform ...'
  `Compensaton` relies on `tf`, please double check if `gnss_driver` has been started, and also use cyber_monitor check `/tf` channel output.
