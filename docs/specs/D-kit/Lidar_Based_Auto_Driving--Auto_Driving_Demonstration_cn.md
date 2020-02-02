# 封闭园区自动驾驶搭建-基于Lidar的自动驾驶演示
﻿
## 前提条件
 
1. 正确完成了[规划适配](Auto_Driving--Planning_Adaptation_cn.md)。

2. 确保在道路平整、车少人少等相对安全的情况下实验。

3. 确保至少两人操作，一人操作工控机，一人操作遥控器，做好随时接管准备。

## 启动流程

1、进入can卡目录启动can卡，用以下命令启动

    cd ~/SocketCan/
    bash start.sh

2、进入docker环境，用gpu编译项目，启动DreamView 

    cd /apollo
    bash docker/scripts/dev_start.sh
    bash docker/scripts/dev_into.sh
    bash apollo.sh build_gpu
    bash scripts/bootstrap.sh

3、在浏览器中打开(http://localhost:8888), 选择`dev_kit`并选择相应虚拟车道线，在module Controller标签页启动GPS、Localization、Radar、Transform、Canbus模块。

![lidar_demonstration_dreamview1](images/lidar_demonstration_dreamview1.jpeg)

4、用以下命令启动激光雷达

    cyber_launch start modules/drivers/velodyne/launch/lidar16.launch

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

6、在DreamView上启动Perception模块，使用`cyber_monitor`查看`/apollo/perception/obstacles`是否正常输出，查看车前方10米处运动的人或者自行车（自行车上要有人），在DreamView上查看障碍物颜色以及位置速度信息（自行车青蓝色，行人黄色，车辆绿色），如下图所示：

![lidar_demonstration_dreamview2](images/lidar_demonstration_dreamview2.png)

`/apollo/perception/obstacles`的数据如下图所示：

![lidar_demonstration_obstacles1](images/lidar_demonstration_obstacles1.png)

![lidar_demonstration_obstacles2](images/lidar_demonstration_obstacles2.png)

如果在DreamView上能看到障碍物则确保`/apollo/perception/obstacles`有障碍物信息。

7、在Module Controller标签页启动Planning、Prediction、Routing、Control模块确保这些模块能够正常启动。

8、在Routing Editor标签中点击Add Point of Interest按钮添加一个point 然后选择Send Routing Request按钮发送添加的routing点，从DreamView中查看会出现一个蓝色的线 以及一个红色的stop标志。如下图所示：

![lidar_demonstration_routing1](images/lidar_demonstration_routing1.png)
9、在车前方存在人或者自行车（车上有人）时，在task标签页查看planning轨迹线，如下图所示：

![lidar_demonstration_routing2](images/lidar_demonstration_routing2.png)

在docker环境中输入命令`cyber_monitor`并查看planning channel信息： 

![lidar_demonstration_channel](images/lidar_demonstration_channel.png) 

如果出现上图所示轨迹线和topic信息，表示规划模块适配和开环测试通过，否则继续查看log信息进行调试。

10、在附近没有人员和车的情况下，遥控器下放权限并在task标签页中点击start auto使车进入自动驾驶状态，在车自动驾驶这个过程中做好随时用遥控器接管确保安全的准备。

**注意**：如果Planning启动不起来的话，可通过摆正车的朝向以及在地图中的位置多试验几次。