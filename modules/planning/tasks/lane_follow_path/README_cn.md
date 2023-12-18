planning-task-lane-follow-path
==============

## 简介
`LaneFollowPath`决策生成借道路路径。
如下图所示：
![](./docs/images/1.png)

### GetStartPointSLState
函数获取规划起点的笛卡尔坐标，而后通过参考线，将笛卡尔坐标转换为frenet坐标。

### DecidePathBounds
函数通过对周围车道、车辆位置和静态障碍物的分析，来决定车辆行驶的路径边界（`candidate_path_boundaries`）。该task通过`Frame`获取自车位置，通过`ReferenceLineInfo`获取车道信息、参考线信息以及静态障碍物信息，最后将计算路径输出到`ReferenceLineInfo`中。

使用`PathBoundsDeciderUtil::InitPathBoundary`初始化路径边界。根据自车道信息更新路径边界，而后根据自车位置对路径边界进行拓展。调用`GetBoundaryFromStaticObstacles`根据静态障碍物在参考线上的投影再次更新路径边界。

### OptimizePath
函数用于优化路径。它接收两个参数：`candidate_path_boundaries`（路径边界）和`candidate_path_data`（存储候选路径数据）。

函数计算了路径曲率约束（`ddl_bounds`）、曲率变化率（`jerk_bound`）和参考路径（`ref_l`和`weight_ref_l`）。
调用`PathOptimizerUtil::OptimizePath`来优化路径。若优化成功，将计算路径`path_data`对象添加到`candidate_path_data`中。

### AssessPath
该函数用于评估候选路径数据，并选择一个最终路径。

代码会遍历候选路径数据。对于每个路径数据，使用`PathAssessmentDeciderUtil`类的静态函数`IsValidRegularPath`来检查路径是否有效。其中会判断路径是否为空、路径是否远离参考线、路径是否远离道路、路径是否与静态障碍物碰撞、路径终点是否在逆向的临近车道上。


## 目录结构 
```shell
modules/planning/tasks/lane_follow_path/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── lane_follow_path.cc
├── lane_follow_path.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── lane_follow_path.proto
└── README_cn.md

```

## 模块

### LaneChangePath插件
apollo::planning::LaneFollowPath

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/lane_follow_path/conf/default_conf.pb.txt` | apollo::planning::LaneFollowPathConfig | LaneFollowPath 的配置文件 |

#### 使用方式
##### 配置加载 LaneFollowPath Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`LaneFollowPath`插件的scenarios xxxx中增加相应的配置，配置参数中 `name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可， `type` 是task的类名称，即 LaneFollowPath。
```
task {
  name: "LANE_FOLLOW_PATH"
  type: "LaneFollowPath"
}
```
##### 配置 LaneFollowPath 参数
在`modules/planning/tasks/lane_follow_path/conf/default_conf.pb.txt`中，对`LaneFollowPath`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`LaneFollowPath`插件的gflag参数进行配置
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```