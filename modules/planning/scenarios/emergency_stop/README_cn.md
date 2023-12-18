
planning-scenario-emergency-stop
============

## 简介

`EmergencyStopScenario`: 紧急停车场景，车辆在行驶过程中如果收到PadMessage命令“PadMessage::STOP”，主车计算停车距离，直接停车。


### IsTransferable: 场景切入条件
  1. 接收到PadMessage命令为 `PadMessage::STOP`

### Stages

| 阶段名                                                     | 类型                                                                        | 描述                     |
| ---------------------------------------------------------- | --------------------------------------------------------------------------- | ------------------------ |
| `EMERGENCY_STOP_APPROACH`                | `apollo::planning::EmergencyStopStageApproach`               | 根据最大减速度计算刹车距离，主车减速刹车，减速度达到阈值后，退出该阶段 |
| `EMERGENCY_STOP_STANDBY`                 | `apollo::planning::EmergencyStopStageStandby`              | 主车保持紧急停车状态，`PadMessage`从 `STOP` 切换至其他状态后退出该阶段   |


#### 1. EMERGENCY_STOP_APPROACH
`EmergencyStopStageApproach` : 该阶段用于主车急停前减速，主车速度达到阈值后退出。

**Process**: 该阶段处理的主函数，输入为规划初始点 `planning_init_point`、`Frame`；输出为当前阶段处理状态`StageResult`
- 主车灯光设置：打开危险报警闪光灯`frame->mutable_reference_line_info()->front().SetEmergencyLight()`。
- 构建虚拟障碍物：根据规划计算的停车点，构建id为`EMERGENCY_STOP`的虚拟障碍物。
- ExecuteTaskOnReferenceLine：输入为规划初始点`planning_init_point`、`Frame`信息，按照该stage配置的task列表，依次进行规划。
- 检查主车状态：检查主车是否减速达到目标速度。如果减速达到目标速度，进入`FinishStage`，结束当前Stage，进入`EMERGENCY_STOP_STANDBY`阶段。如果未达到目标速度，则仍处于`EMERGENCY_STOP_APPROACH`阶段，返回状态值`StageStatusType::RUNNING`。

**FinishStage**: 该阶段的退出函数。
- 退出`EMERGENCY_STOP_APPROACH`阶段，进入`EMERGENCY_STOP_STANDBY`阶段。

#### 2. EMERGENCY_STOP_STANDBY
`EmergencyStopStageStandby` : 该阶段用于主车保持紧急停车状态。

**Process**: 该阶段处理的主函数，输入为规划初始点 `planning_init_point`、`Frame`；输出为当前阶段处理状态`StageResult`
- 构建虚拟障碍物：根据规划计算的停车点，构建id为`EMERGENCY_PULL_OVER`的虚拟障碍物。
- ExecuteTaskOnReferenceLine：输入为规划初始点`planning_init_point`、`Frame`信息，按照该stage配置的task列表，依次进行规划。
- 检查PadMessage：`PadMessage`从 `STOP` 切换至其他状态后，进入`FinishStage`，结束当前Stage；否则仍处于`EMERGENCY_STOP_STANDBY`阶段，返回状态值`StageStatusType::RUNNING`。

**FinishStage**: 该阶段的退出函数。
- 退出`EMERGENCY_STOP_STANDBY`阶段， 进入`EmergencyStopScenario`场景。

## 目录结构

```shell
modules/planning/scenarios/emergency_stop/
├── BUILD
├── conf
│   ├── pipeline.pb.txt
│   └── scenario_conf.pb.txt
├── cyberfile.xml
├── emergency_stop_scenario.cc
├── emergency_stop_scenario.h
├── emergency_stop_scenario_test.cc
├── plugins.xml
├── proto
│   ├── BUILD
│   └── emergency_stop.proto
├── README_cn.md
├── stage_approach.cc
├── stage_approach.h
├── stage_approach_test.cc
├── stage_standby.cc
├── stage_standby.h
└── stage_standby_test.cc
```

## 模块

### EmergencyStopScenario插件

apollo::planning::EmergencyStopScenario


#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| --------------------------------------------------------------------- | ---------------- | ---------------- |
| `modules/planning/scenarios/emergency_stop/conf/scenario_conf.pb.txt`| `apollo::planning::ScenarioEmergencyStopConfig` | 场景的配置文件   |
| `modules/planning/scenarios/emergency_stop/conf/pipeline.pb.txt`     | `apollo::planning::ScenarioPipeline` | 场景的流水线文件 |
| `modules/planning/planning_component/conf/planning_config.pb.txt`                 | `apollo::planning::PlanningConfig`              | planning组件的配置文件               |

#### 使用方式
##### 配置加载 EmergencyStopScenario 插件
- 在`modules/planning/planning_component/conf/planning_config.pb.txt`文件中增加`standard_planning_config`的配置，配置使用当前场景插件。
  ``` shell
  # modules/planning/planning_component/conf/planning_config.pb.txt

  standard_planning_config {
    planner_type: ......
    planner_public_road_config {
      ......
      scenario {
        name: "EMERGENCY_STOP"
        type: "EmergencyStopScenario"
      }
      ......
    }
  }
  ```
- 在`modules/planning/scenarios/emergency_stop/conf/pipeline.pb.txt`中编辑该场景下所有`Stage`，以及各`Stage`对应的`Task`列表。
- 同时支持自主开发，对于新增`Stage`插件，需要添加至`modules/planning/scenarios/emergency_stop/plugins.xml`。
    ```shell
    # modules/planning/scenarios/emergency_stop/plugins.xml
    <library path="modules/planning/scenarios/emergency_stop/libemergency_stop_scenario.so">
        <class type="apollo::planning::EmergencyStopScenario" base_class="apollo::planning::Scenario"></class>
        <class type="apollo::planning::EmergencyStopStageApproach" base_class="apollo::planning::Stage"></class>
        <class type="apollo::planning::EmergencyStopStageStandby" base_class="apollo::planning::Stage"></class>
    </library>
    ```

##### 配置 EmergencyStopScenario 参数
- `proto`文件配置：`modules/planning/scenarios/emergency_stop/proto/emergency_stop.proto`
- EmergencyStopScenario参数配置为：`modules/planning/scenarios/emergency_stop/conf/scenario_conf.pb.txt`
