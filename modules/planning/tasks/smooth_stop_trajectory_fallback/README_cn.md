planning-task-fast-stop-trajectory-fallback
==============

## 简介
planning-task-smooth-stop-trajectory-fallback 插件主要处理在stage场景下默认的fallback_task为：SMOOTH_STOP_TRAJECTORY_FALLBACK、且其task任务运行failed的情况下，同时根据车辆的实时速度、加速度及考虑周围障碍物的碰撞情况，计算出车辆fallback场景轨迹线Speed_data信息，在保证车辆速度fallback的同时，保证了车辆体感同时、避免了与障碍物的碰撞，提升了行驶的安全性。

### GenerateFallbackSpeed
该函数主要作为fallback任务执行时的主函数来计算fallback场景下的速度信息，该函数的输入主要有两个`ego_info`和`FLAGS_speed_fallback_distance`（未使用，设置常量为0），函数的输入中主要通过`injector_`中的`ego_info`来获取车辆上一帧轨迹线终点的信息（速度、加速度信息），`FLAGS_speed_fallback_distance`设置车辆停止距离（未使用）；主要的策略为：当车辆的加速度小于0时，且速度小于0时，设置位移、速度、加速度为0使得车辆立即停下；当车辆加速度小于0（车辆已经开始减速），速度大于0时，分段计算SpeedPoint并检测车辆是否与周围障碍物有碰撞风险；当车辆加速度大于0时（车辆处于加速阶段），根据车辆当前状态是否与周围障碍物有碰撞风险决策采用Fast_stop和Smooth_stop两种策略进行速度的规划；

### GetLowerSTBound
根据车辆的行驶过程中每一帧的参考线`reference_line_info`信息的S-T图表计算障碍物最小的ST边界存储在`lower_bounds`中，通过遍历参考线上的所有`st_boundaries`,计算最小的边界点存储在`lower_bound_points`的点集中，再将所有的点集存储在`lower_bounds`容器中作为检测碰撞的边界条件。

### IsCollisionWithSpeedBoundaries
该函数主要实现的功能时检测当前规划的speed信息是否会与周围的障碍物发生碰撞；函数的输入主要有两个`lower_bounds`和`speed_data`，`lower_bounds`主要是指当前S-T图中与所有障碍物发生碰撞的最小边界的点集，`speed_data`是当前规划的速度、加速度及S向的位移、和时间点信息；该函数的主要原理是判断规划`speed_data`的时间信息，判断此时刻`speed_data`的S位移是否大于`lower_bounds`此时刻的点集的最小S来判断是否会发生碰撞。

## 文件组织结构及说明

```shell
modules/planning/tasks/smooth_stop_trajectory_fallback/
├── util
│   ├── speed_planner.cc                                  // 任务器实现文件
│   └── speed_planner.h                                   // 任务器实现文件
├── BUILD                                                 // 规则构建文件
├── cyberfile.xml                                         // 插件包管理配置文件
├── smooth_stop_trajectory_fallback.cc                    // 任务器实现文件
├── smooth_stop_trajectory_fallback.h                     // 任务器实现文件
├── plugins.xml                                           // 插件配置文件
└── README_cn.md                                          // 说明文档
```


## 模块

### SmoothStopTrajectoryFallback插件

apollo::planning::SmoothStopTrajectoryFallback

#### 场景切入条件
  1. 当前stage场景中的fallback_task为：SMOOTH_STOP_TRAJECTORY_FALLBACK；
  2. 该stage场景下中的task运行发生error；

#### Flags

| 文件路径                                                 | <div style="width: 300pt">说明</div> |
| -------------------------------------------------------- | ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` | planning模块的flag配置文件           |

#### 阶段

| 阶段名                                | 类型                         | 描述                                                                                             |
| ------------------------------------- | ---------------------------- | ------------------------------------------------------------------------------------------------ |
| `轨迹线终点的速度v和加速度a均小于0`   | `apollo::common::SpeedPoint` | 主车轨迹线终点的速度和加速度均小于0，车辆已经完全停止，返回speed_data信息均为0                   |
| `轨迹线终点的加速度小于0且速度v大于0` | `apollo::common::SpeedPoint` | 车辆未完全停靠，分段计算SpeedPoint并检测车辆是否与周围障碍物有碰撞风险                           |
| `轨迹线终点的加速度a大于0时`          | `apollo::common::SpeedPoint` | 根据车辆是否与周围障碍物有碰撞风险决策采用Fast_stop和Smooth_stop两种策略，分段计算车辆speed_data |

#### 使用方式
##### 配置加载 LaneChangePath Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加插件中的scenarios xxxx中增加相应的配置，配置参数中 `name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可， `type` 是task的类名称，即 SmoothStopTrajectoryFallback。
```
fallback_task: {
    name: "SMOOTH_STOP_TRAJECTORY_FALLBACK"
    type: "SmoothStopTrajectoryFallback"
}
```
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```
