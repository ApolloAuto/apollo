planning-task-edge-follow-path
==============

## 简介

`EdgeFollowPath`用于生成贴边自动驾驶路径。该task通过`Frame`获取自车位置，通过`ReferenceLineInfo`获取车道信息、参考线信息以及静态障碍物信息，结合感知与地图路沿信息生成Path，最后将计算路径输出到`ReferenceLineInfo`中。

### EnableEdgeFollow()
- 函数检查是否允许进入贴边路径规划

1. 如果当前参考线不是最右侧车道，则不进行贴边路径规划。
2. 如果考虑`junction`区域，并且车辆将要进入`junction`中，则不进行贴边路径规划。

### UsePerceptionEdge()
- 函数判断是否使用感知路沿进行贴边路径规划

1. 如果感知路沿信息时间戳与当前时间戳差异较大，则不使用感知路沿信息。
2. 如果感知路沿信息中`is_useable`为false，则不使用感知路沿信息。
3. 最后判断全局参数`FLAGS_disable_perception_edge_info`，是否禁用感知路沿信息。
   
### DecidePathBounds()
- 函数通过对周围车道、车辆位置和静态障碍物的分析，来决定车辆行驶的路径边界（`candidate_path_boundaries`）。

遍历已决定的侧通道通行方向（`decided_side_pass_direction_`），对每个方向都尝试找到路径边界。
使用`PathBoundsDeciderUtil::InitPathBoundary`初始化路径边界。根据车道信息和ego的位置更新路径边界。根据静态障碍物在参考线上的投影再次更新路径边界。

其中包括`ProcessEdgeFollowPathBounds()`，对路沿边界进行处理。

### ProcessEdgeFollowPathBounds()
- 处理`path_road_boundary_`，计算路沿右边界
  
1. 如果使用使用感知路沿，则投影将path优化采样点，投影至右侧路沿上，或者每个采样点右侧道路宽度，更新`path_road_boundary_`。
2. 更新`path_boundary`，由于做QP求解边界。

### OptimizePath()
- 函数用于优化路径。它接收两个参数：`candidate_path_boundaries`（路径边界）和`candidate_path_data`（存储候选路径数据）。

函数计算了路径曲率约束（`ddl_bounds`）、曲率变化率（`jerk_bound`）和参考路径（`ref_l`和`weight_ref_l`）。
调用`PathOptimizerUtil::OptimizePath`来优化路径。若优化成功，将计算路径`path_data`对象添加到`candidate_path_data`中。

其中包括对牵引点的计算，`CalculateTowingDis()`，计算横向牵引的`L`值，引入到优化的代价中，实现优化中将路径向右侧牵引。

### CalculateTowingDis()
- 函数用于计算贴边优化中的牵引点，从而实现优化中将路径向右侧牵引，达到贴边效果。

根据`path_road_boundary_`的右边界、主车半车宽、预设贴边距离`FLAGS_edge_follow_buffer`计算横向牵引的`L`值，`towing_l`。


### AssessPath()
该函数用于评估候选路径数据，并选择一个最终路径。

代码会遍历候选路径数据。对于每个路径数据，使用`PathAssessmentDeciderUtil`类的静态函数`IsValidRegularPath`来检查路径是否有效。其中会判断路径是否为空、路径是否远离参考线、路径是否远离道路、路径是否与静态障碍物碰撞、路径终点是否在逆向的临近车道上。`TrimTailingOutLanePoints`剔除道路末端非本车道的路径点。`ComparePathData`根据道路长度、是否借道逆向车道、自车横向位置、更快回到本车道等因素选择合适的路径。

## 目录结构 
```shell
modules/planning/tasks/edge_follow_path/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── edge_follow_path.cc
├── edge_follow_path.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── edge_follow_path.proto
└── README_cn.md

```

## 模块输入输出与配置

### planning-task-edge-follow-path插件

#### 配置文件&配置项
| 文件路径                                                           | 类型/结构                              | <div style="width: 300pt">说明</div> |
| ------------------------------------------------------------------ | -------------------------------------- | ------------------------------------ |
| `modules/planning/tasks/edge_follow_path/conf/default_conf.pb.txt` | apollo::planning::EdgeFollowPathConfig | EdgeFollowPath 的配置文件            |


#### 使用方式

##### 配置加载 EdgeFollowPath Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`EdgeFollowPath`插件的scenarios xxxx中增加相应的配置，配置参数中`name`表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type`是task的类名称，即`EdgeFollowPath`。
```
task {
  name: "EDGE_FOLLOW_PATH"
  type: "EdgeFollowPath"
}
```
##### 配置 EdgeFollowPath 参数
在`modules/planning/tasks/edge_follow_path/conf/default_conf.pb.txt`中，对`EdgeFollowPath`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`EdgeFollowPath`插件的gflag参数进行配置
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```