planning-scenario-park-and-go
============

## 简介

`ParkAndGoScenario` 用于车辆在远离终点且静止条件下，在非城市车道或匹配不到道路点的位置，通过freespace规划，实现车辆由开放空间驶入道路的功能。

### 场景切入条件
  1. 存在沿车道行驶命令
  2. 有参考线生成且由其他scenario切入
  3. 车辆静止且远离终点
  4. 车辆附近无道路或道路非城市车道

### 阶段.

| 阶段名                                                    | 类型                                                                       | 描述                     |
| --------------------------------------------------------- | -------------------------------------------------------------------------- | ------------------------ |
| `PARK_AND_GO_CHECK`            | `apollo::planning::ParkAndGoStageCheck`           | 检测当前车辆状态是否满足公路行驶要求 |
| `PARK_AND_GO_ADJUST`               | `apollo::planning::ParkAndGoStageAdjust`              |  车辆驶向道路阶段  |
| `PARK_AND_GO_PRE_CRUISE` | `apollo::planning::ParkAndGoStagePreCruise` | 调整轨迹曲率阶段   |
| `PARK_AND_GO_CRUISE` | `apollo::planning::ParkAndGoStageCruise` | 接近参考线阶段   |


`PARK_AND_GO_CHECK`通过调用`CheckADCReadyToCruise`函数判断车辆档位、车速、前方障碍物是否可nudge、与道路参考线朝向差、与参考线纵向距离等条件，决定车辆是否满足公路行驶要求。若满足要求，下一阶段为`PARK_AND_GO_CRUISE`，若不满足要求，下一阶段为`PARK_AND_GO_ADJUST`。

`PARK_AND_GO_ADJUST`为车辆调整自身位姿，驶向道路阶段。该阶段将自车在参考线上投影点沿纵向方向移动一定距离后的位置作为目标点，生成对应轨迹。同时调用`CheckADCReadyToCruise`函数检测是否满足公路行驶要求。若满足公路行驶要求或轨迹行驶完成，判断轨迹曲率是否小于阈值，若小于，下一阶段为`PARK_AND_GO_CRUISE`，否则，下一阶段为`PARK_AND_GO_PRE_CRUISE`。

`PARK_AND_GO_PRE_CRUISE`同`PARK_AND_GO_ADJUST`，用于调整自车位姿，使轨迹曲率小于阈值，若轨迹曲率小于阈值，转到下一阶段`PARK_AND_GO_CRUISE`。

`PARK_AND_GO_CRUISE`公共道路行驶阶段，当自车与参考线横向距离小于阈值，该阶段结束。

## 目录结构

```shell

modules/planning/scenarios/park_and_go/
├── BUILD
├── conf
│   ├── park_and_go_adjust
│   │   ├── open_space_roi_decider.pb.txt
│   │   └── open_space_trajectory_provider.pb.txt
│   ├── park_and_go_check
│   │   └── open_space_roi_decider.pb.txt
│   ├── park_and_go_cruise
│   │   └── path_decider.pb.txt
│   ├── park_and_go_pre_cruse
│   │   ├── open_space_roi_decider.pb.txt
│   │   └── open_space_trajectory_provider.pb.txt
│   ├── pipeline.pb.txt
│   └── scenario_conf.pb.txt
├── context.h
├── cyberfile.xml
├── park_and_go_scenario.cc
├── park_and_go_scenario.h
├── park_and_go_scenario_test.cc
├── plugins.xml
├── proto
│   ├── BUILD
│   └── park_and_go.proto
├── README_cn.md
├── stage_adjust.cc
├── stage_adjust.h
├── stage_adjust_test.cc
├── stage_check.cc
├── stage_check.h
├── stage_check_test.cc
├── stage_cruise.cc
├── stage_cruise.h
├── stage_cruise_test.cc
├── stage_pre_cruise.cc
├── stage_pre_cruise.h
├── stage_pre_cruise_test.cc
├── util.cc
└── util.h

```

## 模块

### ParkAndGoScenario插件

apollo::planning::ParkAndGoScenario

#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| --------------------------------------------------------------------- | ---------------- | ---------------- |
| `modules/planning/scenarios/park_and_go/conf/scenario_conf.pb.txt` | `apollo::planning::ScenarioParkAndGoConfig` |场景的配置文件   |
| `modules/planning/scenarios/park_and_go/conf/pipeline.pb.txt`      | `apollo::planning::ScenarioPipeline`| 场景的流水线文件 |
| `modules/planning/scenarios/park_and_go/conf/park_and_go_adjust/open_space_roi_decider.pb.txt`      | `apollo::planning::OpenSpaceRoiDeciderConfig` |`park_and_go_adjust` stage 中 `open_space_roi_decider` task 配置文件 |
| `modules/planning/scenarios/park_and_go/conf/park_and_go_adjust/open_space_trajectory_provider.pb.txt`      |`apollo::planning::OpenSpaceTrajectoryProviderConfig` | `park_and_go_adjust` stage 中`open_space_trajectory_provider` task 配置文件 |
| `modules/planning/scenarios/park_and_go/conf/park_and_go_check/open_space_roi_decider.pb.txt`      |`apollo::planning::OpenSpaceRoiDeciderConfig` | `park_and_go_check` stage 中`open_space_roi_decider` task 配置文件 |
| `modules/planning/scenarios/park_and_go/conf/park_and_go_cruise/path_decider.pb.txt`      | `apollo::planning::PathDeciderConfig` |`park_and_go_cruise` stage 中`path_decider` task 配置文件 |
| `modules/planning/scenarios/park_and_go/conf/park_and_go_pre_cruse/open_space_roi_decider.pb.txt`      |`apollo::planning::OpenSpaceRoiDeciderConfig` | `park_and_go_pre_cruse` stage 中`open_space_roi_decider` task 配置文件 |
| `modules/planning/scenarios/park_and_go/conf/park_and_go_pre_cruse/open_space_trajectory_provider.pb.txt`      |`apollo::planning::OpenSpaceTrajectoryProviderConfig` | `park_and_go_pre_cruse` stage 中`open_space_trajectory_provider` task 配置文件 |