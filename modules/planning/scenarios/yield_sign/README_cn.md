planning-scenario-yield-sign
============

## 简介

`YieldSignScenario`场景可以在有让行标记的场景减速观望，然后慢速通过。

## 目录结构

```shell

modules/planning/scenarios/yield_sign/
├── BUILD
├── conf
│   ├── pipeline.pb.txt
│   └── scenario_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── yield_sign.proto
├── README_cn.md
├── stage_approach.cc
├── stage_approach.h
├── stage_approach_test.cc
├── stage_creep.cc
├── stage_creep.h
├── stage_creep_test.cc
├── yield_sign_scenario.cc
├── yield_sign_scenario.h
└── yield_sign_scenario_test.cc

```

## 模块

### YieldSignScenario插件

apollo::planning::YieldSignScenario

#### 场景切入条件

  1. 车辆前方参考线上的第一个overlap是停止标记类型的overlap
  2. 距离停止标记overlap的距离小于start_stop_sign_scenario_distance
  
#### 阶段

| 阶段名                | 类型                                       | 描述                           |
| --------------------- | ------------------------------------------ | ------------------------------ |
| `YIELD_SIGN_APPROACH` | `apollo::planning::YieldSignStageApproach` | 停让行标记前停车避让运动障碍物 |
| `YIELD_SIGN_CREEP`    | `apollo::planning::YieldSignStageCreep`    | 跛行通过让行区                 |


#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/planning/scenarios/yield_sign/conf/scenario_conf.pb.txt` |`apollo::planning::ScenarioYieldSignConfig` |场景的配置文件   |
| `modules/planning/scenarios/yield_sign/conf/pipeline.pb.txt`      | `apollo::planning::ScenarioPipeline`|场景的流水线文件 |
| `modules/planning/planning_component/conf/planning_config.pb.txt`                 | `apollo::planning::PlanningConfig`              | planning组件的配置文件               |

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

