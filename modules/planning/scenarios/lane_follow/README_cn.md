planning-scenaios-lane-follow
==============

## 简介
`LANE FOLLOW`

1) lane_follow_scenario
这个函数用于判断other_scenario是否能够转移到当前的LaneFollowScenario中。

首先，检查frame规划命令中是否存在车道跟随命令。如果不存在，函数返回false，不能转移。
然后，检查frame的参考线信息是否为空。如果为空，返回false，不能转移。
最后，检查other_scenario是否为空。如果为空，返回true，可以转移。

2) lane_follow_stage

### process
在reference_line_info中寻找可驾驶的线路

这个方法首先检查frame是否包含任何参考线路信息。如果没有，返回finish的StageResult。

然后，它遍历所有的参考线路。对于每一条参考线路，调用PlanOnReferenceLine方法来进行规划。

如果规划结果没有错误，它会检查这条参考线路是否需要进行车道变更（IsChangeLanePath）。
如果不需要，将把这条参考线路标记为可驾驶，并继续处理下一条参考线路。
如果需要，检查这条参考线路的代价（Cost）是否小于不进行车道变更的代价（kStraightForwardLineCost）。如果是，那么它将把这条参考线路标记为可驾驶的。否则，把这条参考线路标记为不可驾驶

如果plan中发生错误，那么把这条参考线路标记为不可驾驶。

在遍历完所有的参考线路后，如果找到了一条可驾驶的线路，返回RUNNING的StageResult。否则，返回ERROR的StageResult。

### PlanOnReferenceLine
生成每一个参考线对应的代价，用于后续判断。

reference_line_info是否变换车道。如果是，增加一个路径成本kStraightForwardLineCost
遍历任务列表中的每一个任务。对于每一个任务，打印执行任务的时间差。

如果任务执行出现错误（ret.IsTaskError()），那么会打印出错误信息并停止执行。

然后，路径和速度信息组合成一条轨迹。如果失败，返回错误状态并打印出错误信息。

接下来，它检查是否有目标点（目的地）在参考线上。如果有，它会记录下这个目标点的s值（在参考线上的位置）。

然后，对于参考线上的每一个障碍物，如果障碍物是静态的并且有停止决策，它会检查这个障碍物的停止点是否在目标点之前。如果是，那增加一个障碍物成本kReferenceLineStaticObsCost。
如果启用了轨迹检查（通过FLAGS变量enable_trajectory_check），那么会检查轨迹是否有效。如果无效，那么会返回一个错误状态并打印出错误信息。

最后，它会设置参考线的轨迹和可驾驶状态，然后返回一个运行中的状态。


## 目录结构 
```shell
modules/planning/scenarios/lane_follow/
├── BUILD
├── conf
│   └── pipeline.pb.txt
├── cyberfile.xml
├── lane_follow_scenario_test.cc
├── lane_follow_scenario.cc
├── lane_follow_scenario.h
├── lane_follow_stage.cc
├── lane_follow_stage.h
├── plugins.xml
└── README_cn.md

```

## 模块

### LaneFollowScenario插件

apollo::planning::LaneFollowScenario

#### 配置

| 文件路径                                                          |类型/结构                            | 说明          |
| --------------------------------------------------------------- |----------------------------------- |---------------|
| `modules/planning/scenarios/lane_follow/conf/pipeline.pb.txt`   | apollo::planning::ScenarioPipeline |场景的流水线文件 |

#### 使用方式
##### 配置加载 LaneFollowScenario 插件
- 在`modules/planning/planning_component/conf/planning_config.pb.txt`文件中增加`standard_planning_config`的配置，配置使用当前场景插件。
  ``` shell
  # modules/planning/planning_component/conf/planning_config.pb.txt

  standard_planning_config {
    planner_type: ......
    planner_public_road_config {
      ......
      scenario {
        name: "LANE_FOLLOW"
        type: "LanefollowScenario"
      }
      ......
    }
  }
  ```
- 在`modules/planning/scenarios/lane_follow/conf/pipeline.pb.txt`中编辑该场景下所有`Stage`，以及各`Stage`对应的`Task`列表。
- 同时支持自主开发，对于新增`Stage`插件，需要添加至`modules/planning/scenarios/lane_follow/plugins.xml`。
    ```shell
    # modules/planning/scenarios/lane_follow/plugins.xml
    <library path="modules/planning/scenarios/lane_follow/liblane_follow_scenario.so">
        <class type="apollo::planning::LaneFollowScenario" base_class="apollo::planning::Scenario"></class>
        <class type="apollo::planning::LaneFollowStageApproach" base_class="apollo::planning::Stage"></class>
        <class type="apollo::planning::LaneFollowStageStandby" base_class="apollo::planning::Stage"></class>
    </library>
    ```

##### 配置 LaneFollowScenario 参数
- `proto`文件配置：`modules/planning/scenarios/lane_follow/proto/lane_follow.proto`
- LaneFollowScenario参数配置为：`modules/planning/scenarios/lane_follow/conf/scenario_conf.pb.txt`