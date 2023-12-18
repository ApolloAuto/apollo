planning-task-rule-based-stop-decider
============

## 简介

`RuleBasedStopDecider`任务用于产生基于规则的停车策略
### 模块流程

1. 借对向车道超车时，检查是否存在感知盲区，如果有则产生停车决策
  ```c++
    StopOnSidePass(frame, reference_line_info);
  ```
  2. 换道时接近换道终点没有成功换道产生停车决策
   ```c++
    if (config_.enable_lane_change_urgency_checking()) {
    CheckLaneChangeUrgency(frame);
  }
   ```
  3. 路径终点产生停车决策
   ```c++
     AddPathEndStop(frame, reference_line_info);
   ```
  
## 目录结构

```shell

modules/planning/tasks/rule_based_stop_decider/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── rule_based_stop_decider.proto
├── README_cn.md
├── rule_based_stop_decider.cc
└── rule_based_stop_decider.h

```

## 模块

### RuleBasedStopDecider 插件

apollo::planning::RuleBasedStopDecider

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/rule_based_stop_decider/conf/default_conf.pb.txt` | apollo::planning::RuleBasedStopDeciderConfig | RuleBasedStopDecider的默认配置文件 |
| `modules/planning/planning_component/conf/planning_config.pb.txt`                 | `apollo::planning::PlanningConfig`              | planning组件的配置文件               |
|`modules/common/data/vehicle_param.pb.txt`|`apollo::common::VehicleConfig`|车辆底盘配置文件|

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |
 
#### 模块参数说明
  
   算法参数配置定义于modules/planning/tasks/rule_based_stop_decider/proto/rule_based_stop_decider.proto
   
   | max_adc_stop_speed                  | 判断主车停车速度     |
   | ----------------------------------- | -------------------- |
   | max_valid_stop_distance             | 停车距离             |
   | search_beam_length                  | 盲区搜索距离         |
   | search_beam_radius_intensity        | 盲区搜索密度向       |
   | search_range                        | 盲区范围             |
   | is_block_angle_threshold            | 判断盲区阈值         |
   | approach_distance_for_lane_change   | 到换道终点距离       |
   | urgent_distance_for_lane_change     | 换道终点前停车距离   |
   | enable_lane_change_urgency_checking | 是否产生换道停止决策 |
   | short_path_length_threshold         | 路径长度阈值         |

#### 使用方式

##### 配置加载 RuleBasedStopDecider Task 插件

    在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`RuleBasedStopDecider`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`RuleBasedStopDecider`。
    ```
    task {
      name: "RULE_BASED_STOP_DECIDER"
      type: "RuleBasedStopDecider"
    }
      ```
