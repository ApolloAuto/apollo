planning-scenario-valet-parking
============

## 简介

`ValetParkingScenario`可以在停车区域泊入指定的车位。

## 目录结构

```shell

modules/planning/scenarios/valet_parking/
├── BUILD
├── conf
│   ├── pipeline.pb.txt
│   ├── scenario_conf.pb.txt
│   ├── valet_parking_approaching_parking_spot
│   │   └── open_space_pre_stop_decider.pb.txt
│   └── valet_parking_parking
│       ├── open_space_roi_decider.pb.txt
│       ├── open_space_trajectory_partition.pb.txt
│       └── open_space_trajectory_provider.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── valet_parking.proto
├── README_cn.md
├── stage_approaching_parking_spot.cc
├── stage_approaching_parking_spot.h
├── stage_approaching_parking_spot_test.cc
├── stage_parking.cc
├── stage_parking.h
├── stage_parking_test.cc
├── valet_parking_scenario.cc
├── valet_parking_scenario.h
└── valet_parking_scenario_test.cc

```

## 模块

### ValetParkingScenario插件

apollo::planning::ValetParkingScenario

#### 场景切入条件

  1. planning command里存在泊车命令
  2. 距离泊车点距离parking_spot_range_to_start以内
  
#### 阶段

| 阶段名                                   | 类型                                            | <div style="width: 300pt">描述</div>                      |
| ---------------------------------------- | ----------------------------------------------- | -------------------------- |
| `VALET_PARKING_APPROACHING_PARKING_SPOT` | `apollo::planning::StageApproachingParkingSpot` | 引导车辆沿主路行驶到泊车位 |
| `VALET_PARKING_PARKING`                  | `apollo::planning::StageParking`                | 泊入车位                   |


#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/planning/scenarios/valet_parking/conf/scenario_conf.pb.txt` |`apollo::planning::ScenarioValetParkingConfig` |场景的配置文件   |
| `modules/planning/scenarios/valet_parking/conf/pipeline.pb.txt`      |`apollo::planning::ScenarioPipeline` |场景的流水线文件 |

