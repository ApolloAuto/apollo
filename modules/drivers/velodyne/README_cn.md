## velodyne
velodyne驱动是以component的形式实现的，包含了:

1. 数据读取打包 --> /driver
2. 生成点云 --> /convert
3. 点云融合 --> /fusion
4. 运动补偿 --> /compensator

4个处理组件，其中`运动补偿`需要依赖`tf`来进行坐标转换查询，因此需要和`gnss_driver`一起运行才能正常工作，点云融合主要将多个激光雷达数据融合成一张点云

### Output channels

1. 数据包
  channel: /apollo/sensor/lidar128/Scan
  type: apollo::drivers::velodyne::VelodyneScan
  proto: [modules/drivers/velodyne/proto/velodyne.proto](https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/velodyne/proto/velodyne.proto)
2. 原始点云
  channel: /apollo/sensor/lidar128/PointCloud2
  type: apollo::drivers::PointCloud
  proto: [modules/drivers/proto/pointcloud.proto]https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/pointcloud.proto
3. 补偿点云
  channel: /apollo/sensor/lidar128/compensator/PointCloud2
  type: apollo::drivers::PointCloud
  proto: [modules/drivers/proto/pointcloud.proto]https://github.com/ApolloAuto/apollo/blob/master/modules/drivers/proto/pointcloud.proto

### 坐标系
* world
* novatel
* velodyne128

### 启动velodyne驱动
**请先修改并确认launch文件中的参数与实际车辆相对应**
```bash
#in docker
cd /apollo && cyber_launch start modules/drivers/velodyne/launch/velodyne.launch
```

### 常见问题
1. "basetime is zero"
  position packet 不可用，检查gps接线或者将车开到有信号的地方
2. "velodyne port 2368 poll() timeout"
  主机与velodyne网络不通，可用命令`sudo tcpdump -i eth0 udp port 2368`查看是否能收到velodyne数据包
3. "cannot find transform ..."
  `运动补偿`依赖`tf`，请检查`gnss_driver`是否已启动，同时可通过cyber_monitor工具查看 `/tf` channel 是否有消息输出
