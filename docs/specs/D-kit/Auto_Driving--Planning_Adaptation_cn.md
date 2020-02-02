# 封闭园区自动驾驶搭建--规划适配

## 前提条件

  - 完成了[激光雷达感知设备适配](Lidar_Based_Auto_Driving--Perception_Adaptation_cn.md)

## 配置文件的修改

|修改文件名称 | 修改内容 | 对应的gflag参数 | 作用 | 
|---|---|---|---|
|`planning.conf` | 修改`defaut_cruise_speed`数值| 比如1.5 | 默认巡航速度|
|`planning.conf` | 修改`planning_upper_speed_limit`数值| 比如1.5 |车planning最大速度 |
|`planning.conf` | 修改`planning_lower_speed_limit`数值| 比如0.5 |车planning最小速度 |
|`planning.conf` |添加`speed_upper_bound`数值| 比如1.5 | 车最大速度|
|`planning.conf` |添加`max_stop_distance_obstacle`数值| 比如10 | 障碍物最大停止距离|
|`planning.conf` |修改`min_stop_distance_obstacle`数值| 比如5 | 障碍物最小停止距离|
|`planning.conf` |添加`destination_check_distance`数值| 比如1.0 | 认为车已经到达目的地时，车与目的地距离|
|`planning.conf` |添加`lon_collision_buffer`数值| 比如0.3 | 车与障碍物的默认碰撞距离|
|`planning.conf` |添加`noenable_scenario_park_and_go`配置项|  | 使起步停车场景失效|
|`planning_config.pb.txt` |修改`total_time`数值| 比如15.0 | planning规划多长时间的路线|
|`planning_config.pb.txt` |修改`max_acceleration`数值| 比如1.0 | 车辆最大加速度|
|`planning_config.pb.txt` |修改`lowest_speed`数值| 比如0.5 | planning时车的最低速度|
|`planning_config.pb.txt` |修改`max_speed_forward`数值| 比如1.5 | 车前进的最大速度|
|`planning_config.pb.txt` |修改`max_acceleration_forward`数值| 比如1.0 | 车前进的最大加速度|

注意：这些配置重新启动planning模块即可生效。为了安全起见，车planning速度一定要设置得比较小，建议按照上面的给出的值来设置相应的配置参数。

## 规划启动流程及开环测试

把车辆开到户外，手动控制车辆，在感知有障碍物信息显示的情况下添加routing点并查看是否有planning轨迹线信息。

1、进入can卡目录启动can卡，用以下命令启动

    cd ~/SocketCan/
    bash start.sh

2、进入docker环境，用gpu编译项目，启动Dreamview 

    cd /apollo
    bash docker/scripts/dev_start.sh
    bash docker/scripts/dev_into.sh
    bash apollo.sh build_gpu
    bash scripts/bootstrap.sh

3、在浏览器中打开(http://localhost:8888), 选择`dev_kit`并选择相应虚拟车道线，在Module Controller标签页启动GPS、Localization、Radar、Transform、Canbus模块。

![planning_adaptation_dreamview](images/planning_adaptation_dreamview.jpeg)

4、用以下命令启动激光雷达 

    budaoshi@in_dev_docker:/apollo$ cyber_launch start modules/drivers/velodyne/launch/velodyne16.launch

5、在docker中输入`cyber_monitor`命令并检查以下channel（使用`上下方向键`选择channel，使用`右方向键`查看channel详细信息）：

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

6、 在dreamview上启动perception模块，使用`cyber_monitor`查看`/apollo/perception/obstacles`是否正常输出，查看车前方10米处运动的人或者自行车（自行车上要有人），在DreamView上查看障碍物颜色以及位置速度信息（自行车青蓝色，行人黄色，车辆绿色），如下图所示：

![planning_adaptation_planning](images/planning_adaptation_planning.png)

`/apollo/perception/obstacles`的数据如下图所示：

![planning_adaptation_obstacle1](images/planning_adaptation_obstacle1.png)

![planning_adaptation_obstacle2](images/planning_adaptation_obstacle2.png)

7、在Module Controller标签页启动Planning、Prediction、Routing模块确保这些模块能够正常启动。

8、 在Routing Editor标签中点击Add Point of Interest按钮添加一个point 然后选择Send Routing Request按钮发送添加的Routing点，从DreamView中查看会出现一个蓝色的线 以及一个红色的stop标志。如下图所示：

![planning_adaptation_routing1](images/planning_adaptation_routing1.png)

9、在车前方存在人或者自行车（车上有人）时，在task标签页查看planning轨迹线，如下图所示：

![planning_adaptation_routing2](images/planning_adaptation_routing2.png)

在docker环境中输入命令`cyber_monitor`并查看planning channel信息： 

![planning_adaptation_planning_channel](images/planning_adaptation_planning_channel.png) 

如果出现上图所示轨迹线和topic信息，表示规划模块适配和开环测试通过，否则继续查看log信息进行调试。
**注**：如果Planning启动不起来的话，可通过摆正车的朝向以及在地图中的位置多试验几次。