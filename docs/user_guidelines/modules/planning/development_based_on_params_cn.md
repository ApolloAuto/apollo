# 通过配置参数开发

## 全局参数

全局参数基于 gflag 定义，全局参数的定义在 `modules/planning/planning_base/common/planning_gflags.cc`文件中。
在 apollo 目录的`modules/planning/planning_base/conf/planning.conf`中可以配置规划模块所有的全局参数。比如配置巡航速度为 10m/s，则可以修改`planning.conf`的参数为：

```bash
--default_cruise_speed=10
```

注意每一行参数中不要有多余的空格，在代码中，可以`FLAGS_XXX`的行驶来获得对应参数的数值，比如代码中想获取巡航车速可以通过`FLAGS_default_cruise_speed`变量来获取，
`planning.conf`中没有定义的`gflag`参数将采用`planning_gflags.cc`中定义的默认值，如果需要修改这些参数，只需要将参数加到`planning.conf`末尾即可。

## 场景插件配置

场景插件配置用来定义您所需要启用的 planning 场景插件，在`modules/planning/planning_base/conf/planning_config.pb.txt`的`planner_public_road_config`中配置，配置的场景插件会在规划模块运行时动态加载到规划模块进程上。如下例子表示仅启动泊车和沿车道行驶场景：

```bash
planner_public_road_config {
    scenario {
      name: "VALET_PARKING"
      type: "ValetParkingScenario"
    }
    scenario {
      name: "LANE_FOLLOW"
      type: "LaneFollowScenario"
    }
 }
```

其中，name 是场景的名字，通常是大写加下划线方式表示，type 是场景的类的名字，均为字符串类型。查看 apollo 包含的所有场景插件可以在`modules/planning/scenarios`目录中查看。

## 场景配置

### 场景流水线配置

每个场景（scenario）都会有个场景流水线配置文件，场景流水线配置了该场景所加载的阶段（stage）插件和每个阶段所需要加载的任务（task）插件。
场景流水线配置文件在场景的conf目录的`pipeline.pb.txt`，比如无保护左转场景的流水线配置文件在`modules/planning/scenarios/traffic_light_unprotected_left_turn/pipeline.pb.txt`:

```bash
stage: {
  name: "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_APPROACH"
  type: "TrafficLightUnprotectedLeftTurnStageApproach"
  enabled: true
  task {
    name: "LANE_FOLLOW_PATH"
    type: "LaneFollowPath"
  }
  task {
    name: "LANE_BORROW_PATH"
    type: "LaneBorrowPath"
  }

}
stage: {
  name: "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP"
  type: "TrafficLightUnprotectedLeftTurnStageCreep"
  enabled: true
  task {
    name: "LANE_FOLLOW_PATH"
    type: "LaneFollowPath"
  }
  ...
}
stage: {
  name: "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE"
  type: "TrafficLightUnprotectedLeftTurnStageIntersectionCruise"
  enabled: true
    task {
    name: "LANE_FOLLOW_PATH"
    type: "LaneFollowPath"
  }

  ...
}
```

如上所示为无保护左转场景配置了 3 个阶段分别为接近路口、缓行观察路口，以及穿过交叉口。每个阶段配置了在这个阶段下所需要执行的任务，可以通过配置任务流水线增减场景的功能，比如不希望在接近路口过程中借道绕行，则可以删除`LANE_BORROW_PATH`任务的配置。

### 场景参数配置

场景参数配置文件在场景目录下的`scenario_conf.pb.txt`，比如无保护左转场景的配置文件为`modules/planning/scenarios/traffic_light_unprotected_left_turn/conf/scenario_conf.pb.txt`。

场景中的配置参数作用范围为场景和场景所包含的阶段使用。比如，无保护左转场景参数：

```bash
start_traffic_light_scenario_distance: 30.0
approach_cruise_speed: 2.78
max_valid_stop_distance: 2.0
creep_timeout_sec: 10.0
max_adc_speed_before_creep: 5.56
creep_stage_config {
    min_boundary_t: 6.0
    ignore_max_st_min_t: 0.1
    ignore_min_st_min_s: 15.0
}
```

通过调整`start_traffic_light_scenario_distance`可以调整进入该场景的距离，调整`approach_cruise_speed`可以调整接近路口阶段的巡航车速。

### 场景任务配置

场景中可以复用任务库中的所有任务，如果该场景下任务参数与默认的任务参数有冲突，则可以单独设置针对于该场景的任务参数。场景任务参数保存在场景的conf配置目录的阶段子目录下。如果完全复用默认的任务参数，则可以不添加场景任务配置文件。

比如，紧急靠边停车场景`emergency_pull_over`，在靠边停车阶段`EMERGENCY_PULL_OVER_APPROACH`，需要用到`pull_over_path`规划靠边停车的路径，但是其参数和默认靠边停车任务参数不同，需要添加`pull_over_path`的场景任务配置参数`pull_over_path.pb.txt`，将其放到场景阶段的配置目录下：`modules/planning/scenarios/emergency_pull_over/conf/emergency_pull_over_approach/pull_over_path.pb.txt`。

### 任务默认配置参数

场景任务配置参数只会在对应场景内生效，任务默认参数的修改可以对所有场景生效。每一个任务的配置参数保存在任务`conf`目录的`default_conf.pb.txt`。比如，规划靠边停车任务的默认配置参数在：
`modules/planning/tasks/pull_over_path/conf/default_conf.pb.txt`。

## 交通规则插件配置

通过交通规则插件配置可以配置 planning 启用的交通规则插件，交通规则插件的所有业务逻辑对于所有场景均会生效，可以通过增加删除插件配置是否启用对应功能。其配置文件位于 `modules/planning/planning_base/conf/traffic_rule_config.pb.txt`。

```bash
rule {
  name: "BACKSIDE_VEHICLE"
  type: "BacksideVehicle"
}
rule {
  name: "CROSSWALK"
  type: "Crosswalk"
}
...
```

其中，name 为交通规则插件名字，type 为交通规则插件类的名字，所有的交通规则插件可以在 `modules/planning/traffic_rules`目录中查看。

对于每一个交通规则，还有其自己的交通规则参数，位于交通规则插件的`conf`目录内。比如人行道规则参数位于`modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt`。

```bash
stop_distance: 1.0
max_stop_deceleration: 4.0
min_pass_s_distance: 1.0
expand_s_distance: 2.0
stop_strict_l_distance: 4.0
stop_loose_l_distance: 5.0
start_watch_timer_distance:10
stop_timeout: 10.0
```

如果希望修改在人道前停车的距离，可以修改`stop_distance`参数。
