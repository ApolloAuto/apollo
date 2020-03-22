# 封闭园区自动驾驶搭建--激光雷达感知适配

## 前提条件

 1. 正确完成了[循迹自动驾驶演示](Waypoint_Following--Operation_And_Questions_cn.md)

 2. 正确完成了[激光雷达感知设备集成](Lidar_Based_Auto_Driving--Sensor_Integration_cn.md)
 
 3. 正确完成了[激光雷达感设备知标定](Lidar_Based_Auto_Driving--Sensor_Calibration_cn.md)

## 配置文件的修改

|序号 | 待修改文件 | 修改内容 | 
|---|---|---|
|  1 | `modules/common/data/global_flagfile.txt` |  添加`--half_vehicle_width=0.43` |
|  2 | `modules/perception/production/launch/dev_kit_perception.launch` |重命名为`perception.launch` 并替换原`perception.launch`文件  |
|  3 | `modules/perception/production/dag/dev_kit_dag_streaming_perception.dag` | 重命名为`dag_streaming_perception.dag` 并替换原`dag_streaming_perception.dag`文件|

## 感知开环验证及测试

把车辆开到户外，手动控制车辆，看感知是否有数据。

#### 1. 进入docker环境，用gpu编译项目，编译项目，启动Dreamview 

    cd apollo
    bash docker/scripts/dev_start.sh
    bash docker/scripts/dev_into.sh
    bash apollo.sh build_gpu
    bash scripts/bootstrap.sh

#### 2. 在浏览器中打开(http://localhost:8888), 选择`dev_kit`并选择相应高精地图，在Module Controller标签页启动GPS、Localization、Radar、Transform模块。

![lidar_adaptation_dreamview1](images/lidar_adaptation_dreamview1.jpeg)

#### 3. 用以下命令启动激光雷达

    budaoshi@in_dev_docker:/apollo$ cyber_launch start modules/drivers/velodyne/launch/velodyne16.launch

#### 4. 在docker中输入`cyber_monitor`命令并检查以下channel（使用`上下方向键`选择channel，使用`右方向键`查看channel详细信息）：
	
|channel_name | 检查项目 | 
|---|---|
| `/apollo/localization/pose`| 确保能正常输出数据 | 
|`/apollo/sensor/gnss/best_pose` | 确保能正常输出数据、`sol_type:` 选项显示为`NARROW_INT`   |
|`/apollo/sensor/lidar16/PointCloud2` | 确保能正常输出数据|
|`/apollo/sensor/lidar16/Scan`| 确保能正常输出数据|
| `/apollo/sensor/lidar16/compensator/PointCloud2`  | 确保能正常输出数据 |
|`/apollo/sensor/radar/front`|确保能正常输出数据|
|`/tf`|确保能正常输出数据|
|`/tf_static`|确保能正常输出数据|

#### 5.  使用如下命令启动perception模块，使用`cyber_monitor`查看`/apollo/perception/obstacles`是否正常输出，并在dreamview上查看障碍物信息：

    budaoshi@in_dev_docker:/apollo$ cyber_launch start modules/perception/production/launch/perception.launch

查看车前方10米处运动的人或者自行车（自行车上要有人），在DreamView上查看障碍物颜色以及位置速度信息（自行车青蓝色，行人黄色，车辆绿色），如下图所示：

![lidar_adaptation_dreamview2](images/lidar_adaptation_dreamview2.png)

`/apollo/perception/obstacles`的数据如下图所示：

![lidar_adaptation_obstacles1](images/lidar_adaptation_obstacles1.png)

![lidar_adaptation_obstacles2](images/lidar_adaptation_obstacles2.png)

如果在dreamview上能看到障碍物并且`/apollo/perception/obstacles`有障碍物信息，则开环测试通过。