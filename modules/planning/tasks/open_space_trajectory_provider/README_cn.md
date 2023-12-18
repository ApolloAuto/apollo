planning-task-open-space-trajectory-provider
==============

## 简介
`OpenSpaceTrajectoryProvider`用于在指定规划边界与目标点位姿的情况下，调用开放空间算法，生成相应轨迹。

### GenerateStopTrajectory
当车辆到达规划终点、开放空间轨迹计算中或`PARK_AND_GO_CHECK`阶段时，调用该函数生成停车轨迹。

### GenerateTrajectoryThread
通过调用函数`OpenSpaceTrajectoryOptimizer::Plan`来生成轨迹：调用`HybridAStar::Plan`函数使用Hybrid A* 与Reeds Shepp曲线计算出路径粗解；调用`OpenSpaceTrajectoryOptimizer::GenerateDecoupledTraj`函数对轨迹进行路径优化与速度优化（详见：`JINYUN Z, RUNXIN H, WANG Y, et al. DL-IAPS and PJSO: A Path/Speed Decoupled Trajectory Optimization and its Application in Autonomous Driving[J]. arXiv: Robotics,arXiv: Robotics, 2020.`）;调用`OpenSpaceTrajectoryOptimizer::GenerateDistanceApproachTraj`函数生成轨迹（详见：`RUNXIN H, JINYUN Z, SHU J, et al. TDR-OBCA: A Reliable Planner for Autonomous Driving in Free-Space Environment[J]. Cornell University - arXiv,Cornell University - arXiv, 2020.`）

### IsVehicleNearDestination
通过比较车辆与终点的角度与距离以及车速来判断车辆是否到达终点。

## 目录结构 
```shell
modules/planning/tasks/open_space_trajectory_provider/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── open_space_trajectory_optimizer.cc
├── open_space_trajectory_optimizer.h
├── open_space_trajectory_provider.cc
├── open_space_trajectory_provider.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── open_space_trajectory_provider.proto
└── README_cn.md
```

## 模块输入输出与配置

### planning-task-open-space-trajectory-provider插件

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/open_space_trajectory_provider/conf/default_conf.pb.txt` | apollo::planning::OpenSpaceTrajectoryProviderConfig | OpenSpaceTrajectoryProvider 的配置文件 |

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 使用方式
##### 配置加载 OpenSpaceTrajectoryProvider Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`OpenSpaceTrajectoryProvider`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`OpenSpaceTrajectoryProvider`。
```
task {
  name: "OPEN_SPACE_TRAJECTORY_PROVIDER"
  type: "OpenSpaceTrajectoryProvider"
}
```
##### 配置 OpenSpaceTrajectoryProvider 参数
在`modules/planning/tasks/open_space_trajectory_provider/conf/default_conf.pb.txt`中，对`OpenSpaceTrajectoryProvider`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`OpenSpaceTrajectoryProvider`插件的gflag参数进行配置。
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```