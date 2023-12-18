planning-task-fallback-path
==============

## 简介
`FallbackPath`用于stage中前置task未生成路径时，生成备用路径,该task只根据车道、车辆位姿生成路径，不考虑道路障碍物信息，障碍物信息在速度规划中处理。该task通过`Frame`获取自车位置，通过`ReferenceLineInfo`获取车道信息以及参考线信息，最后将计算路径输出到`ReferenceLineInfo`中。

### DecidePathBounds
函数通过对自车道、车辆位置分析，来决定车辆行驶的路径边界（`candidate_path_boundaries`）。

使用`PathBoundsDeciderUtil::InitPathBoundary`初始化路径边界。通过函数`GetBoundaryFromSelfLane`根据自车道信息更新路径边界，函数`ExtendBoundaryByADC`根据自车位置以及速度对路径边界进行拓展。

### OptimizePath
函数用于优化路径。它接收两个参数：`candidate_path_boundaries`（路径边界）和`candidate_path_data`（存储候选路径数据）。

函数计算了路径曲率约束（`ddl_bounds`）、曲率变化率（`jerk_bound`）和参考路径（`ref_l`和`weight_ref_l`）。
调用`PathOptimizerUtil::OptimizePath`来优化路径。若优化成功，将计算路径`PathData`对象添加到`candidate_path_data`中。

### AssessPath
该函数用于评估生成路径是否可行。针对`OptimizePath`生成路径，会判断其是否为空、路径是否远离参考线、路径是否远离道路。

## 目录结构 
```shell
modules/planning/tasks/fallback_path/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── fallback_path.cc
├── fallback_path.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── fallback_path.proto
└── README_cn.md
```

## 模块输入输出与配置

### planning-task-fallback-path插件

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/fallback_path/conf/default_conf.pb.txt` | apollo::planning::FallbackPathConfig | FallbackPath 的配置文件 |

#### 使用方式
##### 配置加载 FallbackPath Task 插件
在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`FallbackPath`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`FallbackPath`。
```
task {
  name: "FALLBACK_PATH"
  type: "FallbackPath"
}
```
##### 配置 FallbackPath 参数
在`modules/planning/tasks/fallback_path/conf/default_conf.pb.txt`中，对`FallbackPath`插件的参数进行配置。
在`modules/planning/planning_component/conf/planning.conf`中，对作用在`FallbackPath`插件的gflag参数进行配置。
##### 启动planning
```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```