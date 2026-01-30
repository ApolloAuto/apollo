planning-scenario-large-curvature
============

## 简介

`LargeCurvatureScenario`在大曲率道路上生成通行轨迹，解决主路算法在大曲率道路上的顿挫问题。

## 目录结构

```shell

modules/planning/scenarios/large_curvature/
├── BUILD
├── conf
│   ├── large_curvature
│   │   ├── open_space_fallback_decider_park.pb.txt
│   │   ├── open_space_path_planning.pb.txt
│   │   ├── open_space_replan_decider.pb.txt
│   │   ├── open_space_roi_decider_park.pb.txt
│   │   ├── open_space_rt_optimizer.pb.txt
│   │   ├── open_space_trajectory_optimizer.pb.txt
│   │   └── open_space_trajectory_post_process.pb.txt
│   ├── pipeline.pb.txt
│   └── scenario_conf.pb.txt
├── cyberfile.xml
├── large_curvature_scenario.cc
├── large_curvature_scenario.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── large_curvature_scenario.proto
├── README_cn.md
├── stage_large_curvature.cc
└── stage_large_curvature.h
```

## 模块

### LargeCurvatureScenario插件

apollo::planning::LargeCurvatureScenario

#### 场景切入条件

  1. 参考线自车前3米处曲率大于min_curvature
  2. 自车前方参考线长度小于10米
  
#### 阶段

| 阶段名                | 类型                                    | <div style="width: 300pt">描述</div> |
| --------------------- | --------------------------------------- | ------------------------------------ |
| `StageLargeCurvature` | `apollo::planning::StageLargeCurvature` | 规划出通过大曲率道路的轨迹           |


#### 配置

| 文件路径                                                               | 类型/结构                                        | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------- | ------------------------------------------------ | ------------------------------------ |
| `modules/planning/scenarios/large_curvature/conf/scenario_conf.pb.txt` | `apollo::planning::ScenarioLargeCurvatureConfig` | 场景的配置文件                       |
| `modules/planning/scenarios/large_curvature/conf/pipeline.pb.txt`      | `apollo::planning::ScenarioPipeline`             | 场景的流水线文件                     |

