planning-task-path-time-heuristic
============

## 简介

`PathTimeHeuristicOptimizer`任务基于动态规划算法，在非凸的ST空间做粗的速度规划。处理后的数据保存在reference_line_info_的st_graph_data_中。其算法流程为：

1. 构建ST栅格图
  ```c++
  GriddedPathTimeGraph st_graph(
      reference_line_info_->st_graph_data(), dp_st_speed_optimizer_config,
      reference_line_info_->path_decision()->obstacles().Items(), init_point_);
  ```
  2. 栅格图搜素
   ```c++
   if (!st_graph.Search(speed_data).ok()) {
    AERROR << "failed to search graph with dynamic programming.";
    return false;
  }
   ```
    - 初始化代价表，每个栅格点的坐标为其在st图中的坐标，每个栅格点的cost都设置为无穷大。
    `InitCostTable()`
    - 初始化限速表,初始化每个s方向索引位置的限速，这个限速来自于参考线上的限速，可能是地图上车道线限速或者其他的什么在固定位置的限速。
    `InitSpeedLimitLookUp()`
    - 递推计算每个栅格点的代价
    `CalculateTotalCost()`
    - 从终点回溯获得完整st曲线
    `RetrieveSpeedProfile()`
  
## 目录结构

```shell

modules/planning/tasks/path_time_heuristic/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── dp_st_cost.cc
├── dp_st_cost.h
├── gridded_path_time_graph.cc
├── gridded_path_time_graph.h
├── gridded_path_time_graph_test.cc
├── path_time_heuristic_optimizer.cc
├── path_time_heuristic_optimizer.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── path_time_heuristic.proto
├── README_CN.md
├── st_graph_point.cc
└── st_graph_point.h

```

## 模块

### PathTimeHeuristicOptimizer插件

apollo::planning::PathTimeHeuristicOptimizer

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/path_time_heuristic/conf/default_conf.pb.txt` | apollo::planning::SpeedHeuristicOptimizerConfig | PathTimeHeuristicOptimizer的默认配置文件 |
| `modules/planning/planning_component/conf/planning_config.pb.txt`                 | `apollo::planning::PlanningConfig`              | planning组件的配置文件               |
|`modules/common/data/vehicle_param.pb.txt`|`apollo::common::VehicleConfig`|车辆底盘配置文件|

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 模块参数说明

模块参数配置定义于modules/planning/tasks/path_time_heuristic/proto/path_time_heuristic.proto
   
  | unit_t | 采样时间间隔     |
  | ------------------------------- | ---------------------- |
  | dense_dimension_s  | 加密采样位移数量    |
  | dense_unit_s  | 加密采样位移间隔    |
  | sparse_unit_s  | 稀疏采样位移间隔    |
  | speed_weight  | 速度权重    |
  | accel_weight  | 加速度权重    |
  | jerk_weight  | 加加速度权重    |
  | obstacle_weight  | 障碍物权重    |
  | default_obstacle_cost | 障碍物碰撞代价      |
  | default_speed_cost |  速度代价     |
  | exceed_speed_penalty | 超速惩罚权重      |
  | low_speed_penalty | 低速惩罚权重      |
  | reference_speed_penalty | 偏移参考速度惩罚      |
  | keep_clear_low_speed_penalty | 在KeepClear区域低速惩罚      |
  | accel_penalty | 加速度惩罚  |
  | decel_penalty | 减速度惩罚      |
  | positive_jerk_coeff | 正加加速度惩罚      |
  | negative_jerk_coeff | 负加加速度惩罚      |
  | max_acceleration | 最大加速度      |
  | max_deceleration | 最大减速度      |
  | safe_distance | 跟前车保持的安全距离      |
  | spatial_potential_penalty | 距离终点惩罚      |
  | enable_multi_thread_in_dp_st_graph | 是否开启多线程计算      |
  | enable_dp_reference_speed | 是否增加偏离巡航速度代价     |

#### 命令行参数/gflags
`modules/planning/planning_base/common/planning_gflags.cc`中定义了用到的命令行参数，planning.conf中定义命令行参数值。
| flag | 描述 |
  |  ---- | ---- |
|  FLAGS_speed_lon_decision_horizon | 纵向决策的最远距离 |

#### 使用方式
##### 配置加载 PathTimeHeuristicOptimizer Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`PathTimeHeuristicOptimizer`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`PathTimeHeuristicOptimizer`。
```
task {
  name: "SPEED_HEURISTIC_OPTIMIZER"
  type: "PathTimeHeuristicOptimizer"
}
```
