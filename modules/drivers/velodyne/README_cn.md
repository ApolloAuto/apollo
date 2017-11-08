
## velodyne
velodyne驱动是以nodelet的形式实现的，包含了:

1. 数据读取打包 --> /sensor_velodyne64_driver
2. 生成点云 --> /sensor_velodyne64_convert
3. 运动补偿 --> /sensor_velodyne64_compensator
 
三个处理节点，其中`运动补偿`需要依赖`tf`来进行坐标转换查询，因此需要和`gnss_driver`一起运行才能正常工作
 
### Topics

* /apollo/sensor/velodyne64/VelodyneScanUnified --> velodyne_msgs/VelodyneScanUnified
* /apollo/sensor/velodyne64/PointCloud2 --> sensor_msgs/PointCloud2
* /apollo/sensor/velodyne64/compensator/PointCloud2 --> sensor_msgs/PointCloud2
 
### 坐标系

* world
* novatel
* velodyne64
 
### 编译velodyne

```bash 
# in dev docker
cd /apolloe
bash apollo.sh buildvelodyne
```
产出会覆盖`/apollo/bazel-apollo/external/ros/`中原有的velodyne驱动相关文件
 
### 配置velodyne驱动

首先要指定每个车的对应参数

**velodyne型号与工作模式**
```xml
<!-- 64E_S3D_STRONGEST|64E_S3D_LATEST |64E_S3D_DUAL -->
<arg name="model" default="64E_S3D_STRONGEST"/>
```
 
**在线解析内参**
```xml
<arg name="calibration_online" default="true" />
```
> 注意: 该参数只对64线雷达生效

当该参数设置为`true`的时候，不需要提供内参文件；设置为`false`时，必须提供内参文件
**内参文件**
```xml
<arg name="velodyne64_calibration_file" default="$(find velodyne_pointcloud)/params/64E_S3_calibration_example.yaml"/>
```
 
**外参文件**
```xml
<arg name="extrinsics_velodyne64" default="$(find velodyne_pointcloud)/params/velodyne64_novatel_extrinsics_example.yaml"/>
```

### 启动velodyne驱动
**请先修改并确认launch文件中的参数与实际车辆相对应**
```bash
roslaunch velodyne start_velodyne.launch
# or
bash /apollo/scripts/velodyne.sh
```
 
### 导出pcd
**请先修改并确认launch文件中的参数与实际车辆相对应**
默认导出的目录：/apollo/data/pcd
```bash
roslaunch velodyne export_pcd.launch
```
 
### velodyne数据检查
```bash
roslaunch velodyne velodyne_check.launch
```
在`/apollo/data/log/`目录生成`velodyne_hz.yyyy-mm-dd-HH-MM-SS.log`和`velodyne_hz.yyyy-mm-dd-HH-MM-SS.log.err`两种方式命名的文件。`.log`文件记录了运动补偿后点云的每一帧之间的时间间隔和频率，`.log.err`记录了频率低于9Hz的点云的时间戳和对应时间间隔

### 常见问题
1. velodyne port 2368 poll() timeout
	主机与velodyne网络不通，可用命令`sudo tcpdump -i eth0 udp port 2368`查看是否能收到velodyne数据包
2. cannot find transform ...
	`运动补偿`依赖`tf`，请检查`gnss_driver`是否已启动，同时可通过`rostopic echo /tf`查看是否有消息输出
 
