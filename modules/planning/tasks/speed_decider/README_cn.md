planning-task-speed-decider
============

## 简介

`SpeedDecider`任务用于根据规划的粗速度曲线产生对障碍物的纵向决策
### 模块流程

1. 对于没有st边界或者st边界在可行驶ST区域以外的障碍物产生忽略决策
```c++
    if (boundary.IsEmpty() || boundary.max_s() < 0.0 ||
        boundary.max_t() < 0.0 ||
        boundary.min_t() >= speed_profile.back().t()) {
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }
    if (obstacle->HasLongitudinalDecision()) {
      AppendIgnoreDecision(mutable_obstacle);
      continue;
    }
```
2. 对于不再车道上的虚拟障碍物产生忽略决策
```c++
    if (obstacle->IsVirtual()) {
        const auto& obstacle_box = obstacle->PerceptionBoundingBox();
        if (!reference_line_->IsOnLane(obstacle_box.center())) {
          continue;
        }
      }
```   
3. 对于行人产生停止决策
```c++
    if (config_.is_stop_for_pedestrain() &&
          CheckStopForPedestrian(*mutable_obstacle)) {
        ObjectDecisionType stop_decision;
        if (CreateStopDecision(*mutable_obstacle, &stop_decision,
                              -FLAGS_min_stop_distance_obstacle)) {
          mutable_obstacle->AddLongitudinalDecision("dp_st_graph/pedestrian",
                                                    stop_decision);
        }
        continue;
      }
```   
4. 根据ST曲线和障碍物ST边界的位置对障碍物产生决策：
      * 如果ST曲线在障碍物下方，且障碍物行驶方向和主车相同，产生跟车决策
      * 如果ST曲线 在障碍物下方，且障碍物横穿主车路径，产生让行决策
      * 如果ST曲线 在障碍物下方，且障碍物距离主车过近，产生停止决策
      * 如果ST曲线 在障碍物中间穿过，且障碍物为阻塞障碍物，产生停车决策
## 目录结构

```shell

modules/planning/tasks/speed_decider/
├── BUILD
├── conf
│   └── default_conf.pb.txt
├── cyberfile.xml
├── plugins.xml
├── proto
│   ├── BUILD
│   └── speed_decider.proto
├── README_cn.md
├── speed_decider.cc
└── speed_decider.h

```

## 模块

### SpeedDecider插件

apollo::planning::SpeedDecider

#### 配置文件&配置项
| 文件路径 | 类型/结构 | <div style="width: 300pt">说明</div> |
| ---- | ---- | ---- |
| `modules/planning/tasks/speed_decider/conf/default_conf.pb.txt` | apollo::planning::SpeedDeciderConfig | SpeedDecider的默认配置文件 |

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 模块参数说明
  
算法参数配置定义于modules/planning/tasks/speed_decider/proto/speed_decider.proto
   
   | follow_min_obs_lateral_distance | 跟车的最小横向距离     |
   | ------------------------------- | ---------------------- |
   | max_centric_acceleration_limit  | 最大向心加速度         |
   | follow_min_time_sec             | 跟车时距               |
   | keep_clear_last_point_speed     | KEEP_CLEAR区域最低速度 |

#### 命令行参数/gflags

`modules/planning/planning_base/common/planning_gflags.cc`中定义了用到的命令行参数，planning.conf中定义命令行参数值。
| flag | 描述 |
  |  ---- | ---- |
|  FLAGS_min_stop_distance_obstacle | 障碍物的最短刹停距离 |

 #### 使用方式

  ##### 配置加载 SpeedDecider Task 插件

  在 `modules/planning/scenarios/xxxx/conf/pipeline.pb.txt` 在期望增加`SpeedDecider`插件的scenarios xxxx中增加相应的配置，配置参数中`name` 表示task的名称，这个由用户自定义，表达清楚是哪个task即可，`type` 是task的类名称，即`SpeedDecider`。
  
    ```
    task {
      name: "SPEED_DECIDER"
      type: "SpeedDecider"
    }
      ```
        