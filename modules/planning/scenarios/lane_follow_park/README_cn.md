planning-scenario-lane-follow-park
============

## 简介

`LaneFollowParkScenario`同`LaneFollowScenario`类似，在其基础上增加了遇到障碍物阻塞后倒车脱困的功能。

## 目录结构

```shell

modules/planning/scenarios/lane_follow_park/
├── BUILD
├── cyberfile.xml
├── lane_escape_park_stage.cc
├── lane_escape_park_stage.h
├── lane_follow_park_scenario.cc
├── lane_follow_park_scenario.h
├── lane_follow_park_stage.cc
├── lane_follow_park_stage.h
├── plugins.xml
├── proto
│   ├── BUILD
│   └── lane_follow_park_scenario.proto
└── README_cn.md
```

## 模块

### LaneFollowParkScenario

apollo::planning::LaneFollowParkScenario

#### 场景切入条件

  1. 检查frame规划命令中是否存在车道跟随命令。如果不存在，函数返回false，不能转移。
  2. 检查frame的参考线信息是否为空。如果为空，返回false，不能转移。
  3. 检查other_scenario是否为空。如果为空，返回true，可以转移。

#### LaneEscapeParkStage切入条件

  1. 车辆静止。
  2. 存在规划路径。
  3. 距离终点超过20m。
  4. 前方阈值内存在stop决策障碍物，该障碍物距离路口大于min_distance_block_obs_to_junction，障碍物侧存在足够空间绕行，当前不属于排队场景，该障碍物稳定阻塞ego行驶。

#### LaneEscapeParkStage切出条件

  1. 生成非fallback路径且路径长大于forward_path_length且路径连续稳定生成帧数超success_path_count次。
  2. 车辆静止帧数超stop_check_window次。
  
#### 阶段

| 阶段名                | 类型                                    | <div style="width: 300pt">描述</div>                           |
| --------------------- | --------------------------------------- | -------------------------------------------------------------- |
| `LaneFollowParkStage` | `apollo::planning::LaneFollowParkStage` | 规划通往routing点的行车轨迹                                    |
| `LaneEscapeParkStage` | `apollo::planning::LaneEscapeParkStage` | 前方有障碍物阻塞时执行倒车轨迹，直至生成前进路径或倒车达到阈值 |


#### 配置

| 文件路径                                                                | 类型/结构                                        | <div style="width: 300pt">说明</div> |
| ----------------------------------------------------------------------- | ------------------------------------------------ | ------------------------------------ |
| `modules/planning/scenarios/lane_follow_park/conf/scenario_conf.pb.txt` | `apollo::planning::ScenarioLaneFollowParkConfig` | 场景的配置文件                       |
| `modules/planning/scenarios/lane_follow_park/conf/pipeline.pb.txt`      | `apollo::planning::ScenarioPipeline`             | 场景的流水线文件                     |

