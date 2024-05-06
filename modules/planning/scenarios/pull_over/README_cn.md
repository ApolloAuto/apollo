planning-scenario-pull-over
============

## 简介

`PullOverScenario`: 靠边停车场景，如果参数配置 `enable_pull_over_at_destination` 设置为 `true`, 当车辆到达终点附近时，将自动切入 `PullOverScenario` 并完成靠边停车。

![](./images/pull_over_scenario.png)


### IsTransferable: 场景切入条件
  1. 当前command为`lane_follow_command`
  2. 参考线信息不为空
  3. `FLAGS_enable_pull_over_at_destination` 参数配置允许靠边停车场景
  4. 主车不处于变道状态
  5. 主车距离目标点满足靠边停车距离阈值
  6. 不处于overlap
  7. 最右侧车道允许靠边停车

### Stages

| 阶段名                                        | 类型                                                   | 描述                                                                                                           |
| -------------------------------------------- | ------------------------------------------------------| ---------------------------------------------------------------------------------------------------------------|
| `PULL_OVER_APPROACH`                         | `apollo::planning::PullOverStageApproach`             | 主车靠近靠边停车点  |
| `PULL_OVER_RETRY_APPROACH_PARKING`           | `apollo::planning::PullOverStageRetryApproachParking` | 接近Parking位置点，主车速度、距离误差达到阈值后，进入下一个stage                                                        |
| `PULL_OVER_RETRY_PARKING`                    | `apollo::planning::PullOverStageRetryParking`         | 执行openspace的轨迹规划，主车位置、航向达到阈值后，退出该stage         |

#### 1. PULL_OVER_APPROACH
`PullOverStageApproach` : 该阶段用于主车规划接近靠边停车点，获取靠边停车轨迹，并检查是否完成靠边停车。

**Process**: 该阶段处理的主函数，输入为规划初始点 `planning_init_point`、`Frame`；输出为当前阶段处理状态`StageResult`
- ExecuteTaskOnReferenceLine：输入为规划初始点`planning_init_point`、`Frame`信息，按照该stage配置的task列表，依次进行规划。
- CheckADCPullOver：检查主车的PullOver状态。输入主车状态、参考线信息、场景信息和规划上下文信息，根据主车当前位置和速度，判断与停靠点关系，确定主车`PullOverState`。状态返回值分为: `UNKNOWN`, `PASS_DESTINATION`, `APPROACHING`, `PARK_COMPLETE` 和 `PARK_FAIL`。如果完成靠边停车，即状态为 `PASS_DESTINATION` 或 `PARK_COMPLETE` ，则进入`FinishStage`，结束当前Stage，并且退出当前`PullOverScenario`；如果靠边停车失败，即状态为`PARK_FAIL`，则进入`FinishStage`，结束当前Stage，进入`PULL_OVER_RETRY_APPROACH_PARKING`阶段。
- CheckADCPullOverPathPoint：如果当前仍处于靠边停车阶段，检查关键path_point，根据path_point与停靠点的位置和heading偏差，判断是否path_fail。如果`path_fail==true`, 在未到达停靠点前设置`STOP`的虚拟障碍物。主车到达虚拟障碍物后，进入`FinishStage`，结束当前Stage，进入`PULL_OVER_RETRY_APPROACH_PARKING`阶段。如果`path_fail==false`，则仍处于`PULL_OVER_APPROACH`阶段。


**FinishStage**: 该阶段的退出函数，输入为`bool success`，即该阶段是否靠边停车成功。
- 如果`success==true`，退出`PULL_OVER_APPROACH`阶段，并退出`PullOverScenario`。
- 如果`success==false`，退出`PULL_OVER_APPROACH`阶段，进入`PULL_OVER_RETRY_APPROACH_PARKING`阶段。


#### 2. PULL_OVER_RETRY_APPROACH_PARKING
`PullOverStageRetryApproachParking`：上一阶段直接靠边停车失败，进入该阶段重试接近靠边停车点。

**Process**: 该阶段处理的主函数，输入为规划初始点 `planning_init_point`、`Frame`；输出为当前阶段处理状态`StageResult`
- ExecuteTaskOnReferenceLine：输入为规划初始点`planning_init_point`、`Frame`信息，按照该stage配置的task列表，依次进行规划。
- CheckADCStop：检查主车是否靠近停车点，输入为`Frame`信息。主车满足速度小于阈值，位置距离规划设置的`open_space_pre_stop_fence_s`小于阈值，进入`FinishStage`，结束当前Stage。

**FinishStage**: 该阶段的退出函数。
- 退出`PULL_OVER_RETRY_APPROACH_PARKING`阶段，进入`PULL_OVER_RETRY_PARKING`阶段。

#### 3. PULL_OVER_RETRY_PARKING
`PullOverStageRetryParking`：上一阶段靠近停车点后，进入该阶段实线停车。

**Process**: 该阶段处理的主函数，输入为规划初始点 `planning_init_point`、`Frame`；输出为当前阶段处理状态`StageResult`
- ExecuteTaskOnReferenceLine：输入为规划初始点`planning_init_point`、`Frame`信息，按照该stage配置的task列表，依次进行规划。该阶段主要时调用Openspace的轨迹规划方法进行靠边停车规划。
- CheckADCPullOverOpenSpace：检查主车是否停车，输入为`Frame`信息。主车与目标点的位置、航向偏差小于阈值，进入`FinishStage`，结束当前Stage。

**FinishStage**: 该阶段的退出函数。
- 退出`PULL_OVER_RETRY_PARKING`阶段，退出当前`PullOverScenario`。

## 目录结构

```shell
modules/planning/scenarios/pull_over/
├── BUILD
├── conf
│   ├── pipeline.pb.txt
│   ├── pull_over_approach
│   │   └── pull_over_path.pb.txt
│   ├── pull_over_retry_approach_parking
│   │   └── open_space_pre_stop_decider.pb.txt
│   ├── pull_over_retry_parking
│   │   ├── open_space_roi_decider.pb.txt
│   │   └── open_space_trajectory_partition.pb.txt
│   └── scenario_conf.pb.txt
├── context.h
├── cyberfile.xml
├── images
│   └── pull_over_scenario.png
├── plugins.xml
├── proto
│   ├── BUILD
│   └── pull_over.proto
├── pull_over_scenario.cc
├── pull_over_scenario.h
├── pull_over_scenario_test.cc
├── README_cn.md
├── stage_approach.cc
├── stage_approach.h
├── stage_approach_test.cc
├── stage_retry_approach_parking.cc
├── stage_retry_approach_parking.h
├── stage_retry_approach_parking_test.cc
├── stage_retry_parking.cc
├── stage_retry_parking.h
├── stage_retry_parking_test.cc
├── util.cc
└── util.h
```

## 模块

### PullOverScenario插件

apollo::planning::PullOverScenario

#### 配置文件

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| --------------------------------------------------------------------- | ---------------- | ---------------- |
| `modules/planning/scenarios/pull_over/conf/scenario_conf.pb.txt`                                                 | `apollo::planning::ScenarioPullOverConfig` |场景的配置文件                                     |
| `modules/planning/scenarios/pull_over/conf/pipeline.pb.txt`                                                      | 场景的流水线文件                                   |
| `modules/planning/scenarios/pull_over/conf/pull_over_approach/pull_over_path.pb.txt`                             | `apollo::planning::ScenarioPipeline` |`pull_over_approach` 阶段的配置文件                |
| `modules/planning/scenarios/pull_over/conf/pull_over_retry_approach_parking/open_space_pre_stop_decider.pb.txt`  | `apollo::planning::OpenSpacePreStopDeciderConfig` |`pull_over_retry_approach_parking` 阶段的配置文件  |
| `modules/planning/scenarios/pull_over/conf/pull_over_retry_parking/open_space_roi_decider.pb.txt`                | `apollo::planning::OpenSpaceRoiDeciderConfig` |`pull_over_retry_parking` 阶段的配置文件           |
| `modules/planning/scenarios/pull_over/conf/pull_over_retry_parking/open_space_trajectory_partition.pb.txt`       | `apollo::planning::OpenSpaceTrajectoryPartitionConfig` |`pull_over_retry_parking` 阶段的配置文件           |

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 使用方式
##### 配置加载 PullOverScenario 插件
- 在`modules/planning/planning_component/conf/planning_config.pb.txt`文件中增加`standard_planning_config`的配置，配置使用当前场景插件。
  ``` shell
  # modules/planning/planning_component/conf/planning_config.pb.txt

  standard_planning_config {
    planner_type: ......
    planner_public_road_config {
      ......
      scenario {
        name: "PULL_OVER"
        type: "PullOverScenario"
      }
      ......
    }
  }
  ```
- 在`modules/planning/scenarios/pull_over/conf/pipeline.pb.txt`中编辑该场景下所有`Stage`，以及各`Stage`对应的`Task`列表。
- 同时支持自主开发，对于新增`Stage`插件，需要添加至`modules/planning/scenarios/pull_over/plugins.xml`。

    ```shell
    # modules/planning/scenarios/pull_over/plugins.xml
    <library path="modules/planning/scenarios/pull_over/libpull_over_scenario.so">
        <class type="apollo::planning::PullOverScenario" base_class="apollo::planning::Scenario"></class>
        <class type="apollo::planning::PullOverStageApproach" base_class="apollo::planning::Stage"></class>
        <class type="apollo::planning::PullOverStageRetryApproachParking" base_class="apollo::planning::Stage"></class>
        <class type="apollo::planning::PullOverStageRetryParking" base_class="apollo::planning::Stage"></class>
    </library>
    ```


##### 配置 PullOverScenario 参数
- `proto`文件配置：`modules/planning/scenarios/pull_over/proto/pull_over.proto`
- PullOverScenario参数配置为：`modules/planning/scenarios/pull_over/conf/scenario_conf.pb.txt`
- 各`Stage`参数配置，位于`modules/planning/scenarios/pull_over/conf/<stage_name>/`文件夹下

  ```shell
  # modules/planning/scenarios/pull_over/conf/
  ├── pipeline.pb.txt
  ├── pull_over_approach
  │   └── pull_over_path.pb.txt
  ├── pull_over_retry_approach_parking
  │   └── open_space_pre_stop_decider.pb.txt
  ├── pull_over_retry_parking
  │   ├── open_space_roi_decider.pb.txt
  │   └── open_space_trajectory_partition.pb.txt
  └── scenario_conf.pb.txt
  ```
