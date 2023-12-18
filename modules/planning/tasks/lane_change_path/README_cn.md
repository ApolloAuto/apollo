planning-task-lane-change-path
==============

## 简介
`LaneChangePath`决策生成借道路路径。车辆沿RoutingResponse中的路由线路行驶的过程中，从一个车道切换到相邻车道。切换车道的原因是因为RoutingResponse中指定，而不是车辆主动决策换道，这点与人工驾驶中的换道有一定的区别。因为RoutingResponse是车道级别的导航线路，所以路由线路中前后两个通道之间需要换道。该task通过`Frame`获取自车位置，通过`ReferenceLineInfo`获取车道信息、参考线信息以及静态障碍物信息，同时通过injector_获取上一帧换道状态，最后将计算路径输出到`ReferenceLineInfo`中。
![](./docs/images/1.png)

### UpdateLaneChangeStatus
根据车辆状态、参考线数量以及上一帧车辆是否处于换道状态判断是否换道。`UpdateLaneChangeStatus`遍历所有动态障碍物，通过判断目标行驶方向，根据自车与目标之间速度，设置相应的安全距离，通过`HysteresisFilter`判断换道是否安全。

### DecidePathBounds
函数通过对周围车道、车辆位置和静态障碍物的分析，来决定车辆行驶的路径边界（`candidate_path_boundaries`）。

使用`PathBoundsDeciderUtil::InitPathBoundary`初始化路径边界。根据自车道信息更新路径边界，而后根据自车位置对路径边界进行拓展。调用`GetBoundaryFromLaneChangeForbiddenZone`根据ego位置与当前路径边界，对换道点前的路径进行更新。调用`GetBoundaryFromStaticObstacles`根据静态障碍物在参考线上的投影再次更新路径边界。

### OptimizePath
函数用于优化路径。它接收两个参数：`candidate_path_boundaries`（路径边界）和`candidate_path_data`（存储候选路径数据）。

函数计算了路径曲率约束（`ddl_bounds`）、曲率变化率（`jerk_bound`）和参考路径（`ref_l`和`weight_ref_l`）。
调用`PathOptimizerUtil::OptimizePath`来优化路径。若优化成功，将计算路径`PathData`对象添加到`candidate_path_data`中。

### AssessPath
该函数用于评估候选路径数据，并选择一个最终路径。

代码会遍历候选路径数据。对于每个路径数据，使用`PathAssessmentDeciderUtil`类的静态函数`IsValidRegularPath`来检查路径是否有效。其中会判断路径是否为空、路径是否远离参考线、路径是否远离道路、路径是否与静态障碍物碰撞、路径终点是否在逆向的临近车道上。`TrimTailingOutLanePoints`剔除道路末端非本车道的路径点。

## 目录结构 
```shell
modules/planning/tasks/lane_change_path/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── lane_change_path.cc
├── lane_change_path.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── lane_change_path.proto
└── README_cn.md

```

## 模块

### LaneChangePath插件
apollo::planning::LaneChangePath
#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/lane_change_path/conf/default_conf.pb.txt` | apollo::planning::LaneChangePathConfig | LaneChangePath 的配置文件 |

#### 使用方式
##### 配置加载 LaneChangePath Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`LaneChangePath`插件的scenarios xxxx中增加相应的配置，配置参数中 `name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可， `type` 是task的类名称，即 LaneChangePath。
```
task {
  name: "LANE_CHANGE_PATH"
  type: "LaneChangePath"
}
```
##### 配置 LaneChangePath 参数
在`modules/planning/tasks/lane_change_path/conf/default_conf.pb.txt`中，对`LaneChangePath`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`LaneChangePath`插件的gflag参数进行配置。
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```