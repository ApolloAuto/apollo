planning-task-lane-borrow-path
==============

## 简介

`LaneBorrowPath`用于决策生成借道路路径。如果道路前方有障碍物长时间停留阻塞道路，车辆无法通过在当前车道内绕过，需要往相邻车道借道，绕过当前障碍物。当车辆经过障碍物之后，车辆会立即回到原车道行驶。该task通过`Frame`获取自车位置，通过`ReferenceLineInfo`获取车道信息、参考线信息以及静态障碍物信息，最后将计算路径输出到`ReferenceLineInfo`中。
![](./docs/images/1.png)

### IsNecessaryToBorrowLane()

函数首先检查当前是否处于借道场景中。如果处于借道场景，并且已经使用自我车道一段时间，那么函数会切换到非借道场景，并清除已决定的侧向通行方向。

如果路径规划状态原来就不是在借道场景，那么函数将打印出阻塞障碍物的ID，并检查是否满足车道借用的ADC要求。这些要求包括是否有单一参考线、是否在侧向通行速度范围内、阻塞障碍物是否远离交叉路口、是否是长周期阻塞障碍物、是否有在目标位置前的阻塞障碍物，以及障碍物是否可移动。

如果所有的条件都为true，函数将检查车道是否可以借用，如果可以，将更新已决定的侧向通行方向。如果左侧和右侧的车道都不能借用，那么将退出借道task。

### DecidePathBounds()
函数通过对周围车道、车辆位置和静态障碍物的分析，来决定车辆行驶的路径边界（`candidate_path_boundaries`）。

遍历已决定的侧通道通行方向（`decided_side_pass_direction_`），对每个方向都尝试找到路径边界。
使用`PathBoundsDeciderUtil::InitPathBoundary`初始化路径边界。根据车道信息和ego的位置更新路径边界。根据静态障碍物在参考线上的投影再次更新路径边界。

### OptimizePath()
函数用于优化路径。它接收两个参数：`candidate_path_boundaries`（路径边界）和`candidate_path_data`（存储候选路径数据）。

函数计算了路径曲率约束（`ddl_bounds`）、曲率变化率（`jerk_bound`）和参考路径（`ref_l`和`weight_ref_l`）。
调用`PathOptimizerUtil::OptimizePath`来优化路径。若优化成功，将计算路径`path_data`对象添加到`candidate_path_data`中。

### AssessPath()
该函数用于评估候选路径数据，并选择一个最终路径。

代码会遍历候选路径数据。对于每个路径数据，使用`PathAssessmentDeciderUtil`类的静态函数`IsValidRegularPath`来检查路径是否有效。其中会判断路径是否为空、路径是否远离参考线、路径是否远离道路、路径是否与静态障碍物碰撞、路径终点是否在逆向的临近车道上。`TrimTailingOutLanePoints`剔除道路末端非本车道的路径点。`ComparePathData`根据道路长度、是否借道逆向车道、自车横向位置、更快回到本车道等因素选择合适的路径。

## 目录结构 
```shell
modules/planning/tasks/lane_borrow_path/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── lane_borrow_path.cc
├── lane_borrow_path.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── lane_borrow_path.proto
└── README_cn.md

```

## 模块输入输出与配置

### planning-task-lane-borrow-path插件

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/lane_borrow_path/conf/default_conf.pb.txt` | apollo::planning::LaneBorrowPathConfig | LaneBorrowPath 的配置文件 |


#### 使用方式

##### 配置加载 LaneBorrowPath Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`LaneBorrowPath`插件的scenarios xxxx中增加相应的配置，配置参数中`name`表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type`是task的类名称，即`LaneBorrowPath`。
```
task {
  name: "LANE_BORROW_PATH"
  type: "LaneBorrowPath"
}
```
##### 配置 LaneBorrowPath 参数
在`modules/planning/tasks/lane_borrow_path/conf/default_conf.pb.txt`中，对`LaneBorrowPath`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`LaneBorrowPath`插件的gflag参数进行配置
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```