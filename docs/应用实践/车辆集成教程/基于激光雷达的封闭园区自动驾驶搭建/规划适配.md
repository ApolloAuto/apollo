该用户手册旨在帮助用户完成规划模块的配置和开环验证。规划模块需要上游的感知数据，目前有 Lidar 感知和 Camera 感知两种方案，您可以选择其中任意一种感知方案。

## 前提条件

1. 完成 [基于激光雷达的封闭园区自动驾驶搭建--感知适配](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/感知适配.md) 或 [基于摄像头的封闭园区自动驾驶搭建--感知适配](../应用实践/车辆集成教程/基于摄像头的封闭园区自动驾驶搭建/感知适配.md)。

2. 完成 [封闭园区自动驾驶搭建--虚拟车道线制作](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/虚拟车道线制作.md)。

## 步骤一：修改配置文件

对 `/apollo/modules/planning/conf/planning.conf` 和 `/apollo/modules/planning/conf/planning_config.pb.txt` 两个配置文件进行以下修改：

| 文件名称                 | 参数                                                     | 单位            | 说明                                   |
| ------------------------ | -------------------------------------------------------- | --------------- | -------------------------------------- |
| `planning.conf`          | 修改 `default_cruise_speed` 数值。示例值：1.5            | m/s             | 默认巡航速度                           |
| `planning.conf`          | 修改 `planning_upper_speed_limit` 数值。示例值：1.5      | m/s             | 车 planning 最大速度                   |
| `planning.conf`          | 添加 `planning_lower_speed_limit` 数值。示例值：0.5      | m/s             | 车 planning 最小速度                   |
| `planning.conf`          | 添加 `speed_upper_bound` 数值。示例值：1.5               | m/s             | 车最大速度                             |
| `planning.conf`          | 添加 `max_stop_distance_obstacle` 数值。示例值：10       | m               | 障碍物最大停止距离                     |
| `planning.conf`          | 添加 `min_stop_distance_obstacle` 数值。示例值：5        | m               | 障碍物最小停止距离                     |
| `planning.conf`          | 添加 `destination_check_distance` 数值。示例值：1.0      | m               | 认为车已经到达目的地时，车与目的地距离 |
| `planning.conf`          | 添加 `lon_collision_buffer` 数值。示例值：0.3            | m               | 车与障碍物的默认碰撞距离               |
| `planning.conf`          | 添加 `enable_scenario_park_and_go` 配置项。示例值：false | -               | 使起步停车场景失效                     |
| `planning_config.pb.txt` | 修改 `total_time` 数值。示例值：15.0                     | s               | planning 规划多长时间的路线            |
| `planning_config.pb.txt` | 修改 `max_acceleration` 数值。示例值：1.0                | m/s<sup>2</sup> | 车辆最大加速度                         |
| `planning_config.pb.txt` | 修改 `lowest_speed` 数值。示例值：0.5                    | m/s             | planning 时车的最低速度                |
| `planning_config.pb.txt` | 修改 `max_speed_forward` 数值。示例值：1.5               | m/s             | 车前进的最大速度                       |
| `planning_config.pb.txt` | 修改 `max_acceleration_forward` 数值。示例值：1.0        | m/s<sup>2</sup> | 车前进的最大加速度                     |

> 注意：重新启动 planning 模块，配置即可生效。为了安全起见，planning 速度一定要设置得比较小，建议您按照上面的给出的值来设置相应的配置参数。

## 步骤二：启动流程

把车辆开到户外，手动控制车辆，在感知有障碍物信息显示的情况下添加 routing 点并查看是否有 planning 轨迹线信息。

### 1. 拍下车辆尾部的急停开关

本文档目的是进行规划的适配，并完规划模块的开环验证，不需要车辆行驶，为了安全起见，将车开到室外调试场后，请全程确保急停开关处于按下状态。

### 2. 启动感知

启动感知模块，参见 [感知适配](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/感知适配.md '_self')。

### 3. 启动 Planning、Prediction、Control 模块

确保感知模块启动成功后，在 `Modules` 标签页启动 `Planning` 、 `Prediction` 、 `Control` 模块。DreamView+ 上显示启动的模块如下图所示：

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_6d30cb0.png)

### 4. 验证 Planning、Prediction、Control 模块是否启动成功

1. 在车辆可视化面板中，点击路由编辑，选取起始点和途径点。

2. 点击 **保存编辑** 。

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_9b9860a.png)

3. 单击 **发送路由** 按钮。

   从 DreamView+ 中查看会出现一个蓝色的线以及一个红色的 stop 标志。

   在车前方存在人或者自行车（车上有人）时，在 **车辆可视化面板** 中查看 planning 轨迹线，正常情况下 planning 会重新规划轨迹，如下图所示：

   ![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_f7bbf89.png)

在 docker 环境中输入命令 `cyber_monitor` 并查看 planning channel 信息：

![lidar_demonstration_channel](https://bce.bdstatic.com/p3m/Apollo-Homepage-Document/images/Apollo-D-KIT/37aad4eb3d789c3ff843fa437a9157a8.png)

如果出现上图所示轨迹线和 topic 信息，表示规划模块适配和开环测试通过。否则，继续查看 log 信息进行调试。

## NEXT

现在，您已经完成规划适配，根据您适配的是 Lidar 感知方案还是 Camera 感知方案，接下来可以开始 [基于激光雷达的封闭园区自动驾驶搭建--自动驾驶演示](../应用实践/车辆集成教程/基于激光雷达的封闭园区自动驾驶搭建/自动驾驶演示.md) 或 [基于摄像头的封闭园区自动驾驶搭建--自动驾驶演示](../应用实践/车辆集成教程/基于摄像头的封闭园区自动驾驶搭建/自动驾驶演示.md)。
