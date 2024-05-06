planning-scenario-traffic-light-protected
============

## 简介

`TrafficLightProtectedScenario` 是有路权保护的红绿灯场景，在该场景下可以实现在红绿灯路口前红灯停车，绿灯通过路口。

## 目录结构

```shell

modules/planning/scenarios/traffic_light_protected/
├── BUILD
├── conf
│   ├── pipeline.pb.txt
│   ├── scenario_conf.pb.txt
│   └── traffic_light_protected_intersection_cruise
├── context.h
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── traffic_light_protected.proto
├── README_cn.md
├── stage_approach.cc
├── stage_approach.h
├── stage_approach_test.cc
├── stage_intersection_cruise.cc
├── stage_intersection_cruise.h
├── traffic_light_protected_scenario.cc
├── traffic_light_protected_scenario.h
└── traffic_light_protected_scenario_test.c

```

## 模块

### TrafficLightProtectedScenario插件

apollo::planning::TrafficLightProtectedScenario

#### 场景切入条件
  1. 车辆前方参考线上的第一个overlap是交通灯类型的overlap
  2. 距离停止标记overlap的距离小于start_traffic_light_scenario_distance
  3. 前方overlap的信号灯存在红灯
  
#### 阶段

| 阶段名                                        | 类型                                                             | <div style="width: 300pt">描述</div> |
| --------------------------------------------- | ---------------------------------------------------------------- | ------------------------ |
| `TRAFFIC_LIGHT_PROTECTED_APPROACH`            | `apollo::planning::TrafficLightProtectedStageApproach`           | 在红绿灯停止线前停车阶段 |
| `TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP`   | `apollo::planning::TrafficLightUnprotectedLeftTurnStageCreep`    | 绿灯后跛行观察路口来车   |
| `TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE` | `apollo::planning::TrafficLightProtectedStageIntersectionCruise` | 通过红绿灯路口阶段       |


#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/planning/scenarios/traffic_light_protected/conf/scenario_conf.pb.txt` | `apollo::planning::ScenarioTrafficLightProtectedConfig`|场景的配置文件   |
| `modules/planning/scenarios/traffic_light_protected/conf/pipeline.pb.txt`      |`apollo::planning::ScenarioPipeline` | 场景的流水线文件 |
| `modules/planning/planning_component/conf/planning_config.pb.txt`                 | `apollo::planning::PlanningConfig`              | planning组件的配置文件               |

