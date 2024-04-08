# module-planning

## 模块概述

`planning` 模块包含planning模块的整体架构和流程。planning模块根据上游模块输入的感知周围环境信息，地图定位导航信息，以及全局路径信息，为自动驾驶车辆规划出一条运动轨迹（包含坐标，速度，加速度，jerk加加速度，时间等信息），然后将这些信息传递给控制模块。

## 输入输出

### 输入

Planning模块需要获取外部环境信息，车辆自身信息进行轨迹规划，以下是planning的外部输入信息：

| Channel 名                         | 类型                                         | <div style="width: 300pt">描述</div>                                                                                                                    |
| ---------------------------------- | -------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/apollo/prediction`               | `apollo::prediction::PredictionObstacles`    | 障碍物预测信息，可通过 `modules/planning/planning_component/dag/planning.dag` 启动文件修改channel名                                                     |
| `/apollo/perception/traffic_light` | `apollo::perception::TrafficLight`           | perception模块输出的交通灯感知信息，包含交通灯亮起的颜色，id等信息                                                                                      |
| `/apollo/localization/pose`        | `apollo::localization::LocalizationEstimate` | 定位信息，可通过 `modules/planning/planning_component/dag/planning.dag` 配置文件修改channel名                                                           |
| `/apollo/canbus/chassis`           | `apollo::canbus::Chassis`                    | canbus模块输出的车辆底盘信息，包含底盘速度，油门，刹车，档位，灯光等状态， `modules/planning/planning_component/dag/planning.dag` 配置文件修改channel名 |

此外，planning模块还需要外部输入的导航命令信息，用户首先向external_command发送导航命令请求，external_command再将这些命令进行处理后转发给planning模块。下面介绍用户可以发送的几种导航命令：

| Channel 名                               | 类型                                            | <div style="width: 300pt">描述</div>                                           |
| ---------------------------------------- | ----------------------------------------------- | ------------------------------------------------------------------------------ |
| `/apollo/external_command/lane_follow`   | `apollo::external_command::LaneFollowCommand`   | 基于高精地图导航的命令，给定终点的位置或朝向，从当前车辆位置导航到目标终点位置 |
| `/apollo/external_command/valet_parking` | `apollo::external_command::ValetParkingCommand` | 从当前位置导航泊车到停车位上                                                   |
| `/apollo/external_command/action`        | `apollo::external_command::ActionCommand`       | HMI发送的流程操作命令                                                          |

### 输出

| Channel 名                             | 类型                                          | <div style="width: 300pt">描述</div>             |
| -------------------------------------- | --------------------------------------------- | ------------------------------------------------ |
| `/apollo/planning`                     | `apollo::planning::ADCTrajectory`             | 输出规划轨迹，包含轨迹点，速度和时间等信息       |
| `/apollo/planning/command_status`      | `apollo::external_command::CommandStatus`     | 导航命令的执行状态                               |
| `/apollo/external_command/lane_follow` | `apollo::external_command::LaneFollowCommand` | 在道路被阻塞，换道失败超时时，发送重新路由的申请 |

## 参数

### 配置

| 文件路径                                                                          | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| --------------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/planning/planning_component/conf/planning_config.pb.txt`                 | `apollo::planning::PlanningConfig`              | planning组件的配置文件               |
| `modules/planning/planning_component/conf/public_road_planner_config.pb.txt`      | `apollo::planning::PlannerOpenSpaceConfig`      | PublicRoadPlanner的配置文件          |
| `modules/planning/planning_component/conf/traffic_rule_config.pb.txt`             | `apollo::planning::TrafficRulesPipeline`        | 支持的traffic rules列表的配置文件    |
| `modules/planning/planning_component/conf/discrete_points_smoother_config.pb.txt` | `apollo::planning::ReferenceLineSmootherConfig` | 参考线使用离散点平滑时的配置文件     |
| `modules/planning/planning_component/conf/qp_spline_smoother_config.pb.txt`       | `apollo::planning::ReferenceLineSmootherConfig` | 参考线使用五次多项式平滑时的配置文件 |
| `modules/planning/planning_component/conf/spiral_smoother_config.pb.txt`          | `apollo::planning::ReferenceLineSmootherConfig` | 参考线使用五次螺旋线平滑时的配置文件 |
| `modules/planning/planning_component/conf/planner_open_space_config.pb.txt`       | `apollo::planning::PlannerOpenSpaceConfig`      | 开放空间规划算法的配置文件           |

### Flags

| 文件路径                                                 | <div style="width: 300pt">说明</div> |
| -------------------------------------------------------- | ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` | planning模块的flag配置文件           |

## 包列表

| 文件路径                                                                                                                               | <div style="width: 300pt">说明</div>                                                                                     |
| -------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| [planning](modules/planning/planning_component/README_cn.md)                                                                           | planning组件包，负责启动planning模块，串联planning运行流程                                                               |
| [planning-interface-base](modules/planning/planning_interface_base/README_cn.md)                                                       | 包含planning插件父类接口                                                                                                 |
| [planning-base](modules/planning/planning_base/README_cn.md)                                                                           | 包含planning模块基础数据结构和算法库                                                                                     |
| planning-open-space | 包含开放空间规划算法 |
| [planning-planner-lattice](modules/planning/planners/lattice/README_cn.md)                                                             | LatticePlanner插件包，适用于高速公路等简单场景                                                                           |
| [planning-planner-navi](modules/planning/planners/navi/README_cn.md)                                                                   | Navi Planner插件包，适用于高速公路等简单场景                                                                             |
| [planning-planner-rtk](modules/planning/planners/rtk/README_cn.md)                                                                     | Rtk Panner插件包，根据事先录制好的轨迹进行循迹行驶                                                                       |
| [planning-planner-public-road](modules/planning/planners/public_road/README_cn.md)                                                     | PublicRoad Planner插件包，适用于城市道路的场景                                                                           |
| [planning-lane-follow-map](modules/planning/pnc_map/lane_follow_map/README_cn.md)                                                      | LaneFollowMap插件包，基于高精地图生成参考线                                                                              |
| [planning-scenario-bare-intersection-unprotected](modules/planning/scenarios/bare_intersection_unprotected/README_cn.md)               | 无保护路口场景插件包，处理没有交通灯的交叉路口场景                                                                       |
| [planning-scenario-emergency-pull-over](modules/planning/scenarios/emergency_pull_over/README_cn.md)                                   | 紧急靠边停车场景插件包，接收紧急停靠的命令并就近靠边停车                                                                 |
| [planning-scenario-emergency-stop](modules/planning/scenarios/emergency_stop/README_cn.md)                                             | 紧急停车场景插件包，接收紧急停车的命令并在当前车道停车                                                                   |
| [planning-scenario-lane-follow](modules/planning/scenarios/lane_follow/README_cn.md)                                                   | 沿车道线行驶场景插件包，是默认的行驶场景                                                                                 |
| [planning-scenario-park-and-go](modules/planning/scenarios/park_and_go/README_cn.md)                                                   | 停车后重新行驶场景插件包，在车辆停靠在路边非车道线范围内，重新行驶时时会先进入此场景，将车辆行驶到车道线上               |
| [planning-scenario-pull-over](modules/planning/scenarios/pull_over/README_cn.md)                                                       | 靠边停车场景插件包，在车辆接近终点并且终点靠边停车使能后进入此场景                                                       |
| [planning-scenario-stop-sign-unprotected](modules/planning/scenarios/stop_sign_unprotected/README_cn.md)                               | 停车标志场景插件包，在行驶过程中遇到停止标志时进入此场景                                                                 |
| [planning-scenario-traffic-light-protected](modules/planning/scenarios/traffic_light_protected/README_cn.md)                           | 有保护交通灯场景插件包，在路口有保护的交通标志时进入此场景                                                               |
| [planning-scenario-traffic-light-unprotected-left-turn](modules/planning/scenarios/traffic_light_unprotected_left_turn/README_cn.md)   | 无保护交通灯左转场景插件包，在路口无保护的交通灯左转时进入此场景                                                         |
| [planning-scenario-traffic-light-unprotected-right-turn](modules/planning/scenarios/traffic_light_unprotected_right_turn/README_cn.md) | 无保护交通灯右转场景插件包，在路口无保护的交通灯右转时进入此场景                                                         |
| [planning-scenario-valet-parking](modules/planning/scenarios/valet_parking/README_cn.md)                                               | 在停车位停车场景插件包，在导航命令中指定了停车位并且车辆接近停车位时进入此场景                                           |
| [planning-scenario-yield-sign](modules/planning/scenarios/yield_sign/README_cn.md)                                                     | 让行标志场景插件包，在行驶时遇到让行标志时进入此场景                                                                     |
| [planning-task-fallback-path](modules/planning/tasks/fallback_path/README_cn.md)                                                       | 备用路径任务插件包，在沿道路行驶时无法规划出合理的路径时，此任务产生备用路径，沿当前车道线行驶并在障碍物前停车           |
| [planning-task-lane-borrow-path](modules/planning/tasks/lane_borrow_path/README_cn.md)                                                 | 借道路径任务插件包，在沿道路行驶时当前车道被占用时生成借道路径                                                           |
| [planning-task-lane-change-path](modules/planning/tasks/lane_change_path/README_cn.md)                                                 | 换到路径任务插件包，在沿道路行驶时根据路由线路需要换道时生成换道路径                                                     |
| [planning-task-lane-follow-path](modules/planning/tasks/lane_follow_path/README_cn.md)                                                 | 沿当前车道任务插件包，在沿道路行驶时正常沿当前车道行驶时生成路径                                                         |
| [planning-task-open-space-fallback-decider](modules/planning/tasks/open_space_fallback_decider/README_cn.md)                           | 开放空间行驶备用轨迹任务插件包，检测开放空间下轨迹是否与障碍物发生碰撞并做出相应策略                                     |
| [planning-task-open-space-pre-stop-decider](modules/planning/tasks/open_space_pre_stop_decider/README_cn.md)                           | 开放空间行驶预先停车任务插件包，执行泊车与靠边停车任务时，生成在公共道路上的停车点，车辆停在停车点后，会转入开放空间算法 |
| [planning-task-open-space-roi-decider](modules/planning/tasks/open_space_roi_decider/README_cn.md)                                     | 开放空间行驶可行区域生成任务插件包，在开放空间算法中生成可行驶边界                                                       |
| [planning-task-open-space-trajectory-partition](modules/planning/tasks/open_space_trajectory_partition/README_cn.md)                   | 开放空间行驶轨迹分割任务插件包，在开放空间将轨迹按前进后退分割成多段，并调整速度规划                                     |
| [planning-task-open-space-trajectory-provider](modules/planning/tasks/open_space_trajectory_provider/README_cn.md)                     | 开放空间轨迹生成任务插件包，在指定规划边界与目标点位姿的情况下，调用开放空间算法，生成相应轨迹                           |
| [planning-task-path-decider](modules/planning/tasks/path_decider/README_cn.md)                                                         | 路径决策任务插件包，对行驶路径进行决策                                                                                   |
| [planning-task-path-reference-decider](modules/planning/tasks/path_reference_decider/README_cn.md)                                     | 路径合理性决策任务插件包，对推理生成的路径的合理性进行决策，用于后续求解                                                 |
| [planning-task-path-time-heuristic](modules/planning/tasks/path_time_heuristic/README_cn.md)                                           | 速度规划初解任务插件包，基于动态规划算法，在非凸的ST空间做粗的速度规划                                                   |
| [planning-task-piecewise-jerk-speed](modules/planning/tasks/piecewise_jerk_speed/README_cn.md)                                         | 线性速度规划优化解任务插件包，基于二次规划在凸ST空间进行速度规划                                                         |
| [planning-task-piecewise-jerk-speed-nonlinear](modules/planning/tasks/piecewise_jerk_speed_nonlinear/README_cn.md)                     | 非线性速度规划优化解任务插件包，基于非线性规划在凸ST空间进行速度规划                                                     |
| [planning-task-pull-over-path](modules/planning/tasks/pull_over_path/README_cn.md)                                                     | 靠边停车路径生成任务插件包，用于生成靠边停车路径                                                                         |
| [planning-task-reuse-path](modules/planning/tasks/reuse_path/README_cn.md)                                                             | 复用路径任务插件包，在没有碰撞的情况下，复用上一帧的路径                                                                 |
| [planning-task-rss-decider](modules/planning/tasks/rss_decider/README_cn.md)                                                           | RSS决策器任务插件包，用于判断planning给出的规划是否安全                                                                  |
| [planning-task-rule-based-stop-decider](modules/planning/tasks/rule_based_stop_decider/README_cn.md)                                   | 停车策略任务插件包，用于产生基于规则的停车策略                                                                           |
| [planning-task-speed-bounds-decider](modules/planning/tasks/speed_bounds_decider/README_cn.md)                                         | 速度边界生成任务插件包，用于产生速度规划的ST可行驶区间                                                                   |
| [planning-task-speed-decider](modules/planning/tasks/speed_decider/README_cn.md)                                                       | 速度规划决策任务插件包，用于根据规划的粗速度曲线产生对障碍物的纵向决策                                                   |
| [planning-task-st-bounds-decider](modules/planning/tasks/st_bounds_decider/README_cn.md)                                               | st边界决策器任务插件包，生成st边界                                                                                       |
| [planning-traffic-rules-backside-vehicle](modules/planning/traffic_rules/backside_vehicle/README_cn.md)                                | 后向车辆规则插件包，生成对后向车辆的速度决策                                                                             |
| [planning-traffic-rules-crosswalk](modules/planning/traffic_rules/crosswalk/README_cn.md)                                              | 人行道规则插件包，对前方人行道生成是否停车的决策                                                                         |
| [planning-traffic-rules-destination](modules/planning/traffic_rules/destination/README_cn.md)                                          | 终点规则插件包，在终点附近生成是否停车的决策                                                                             |
| [planning-traffic-rules-keepclear](modules/planning/traffic_rules/keepclear/README_cn.md)                                              | 禁行区域规则插件包，在禁行区域生成速度决策                                                                               |
| [planning-traffic-rules-reference-line-end](modules/planning/traffic_rules/reference_line_end/README_cn.md)                            | 参考线终点规则插件包，在参考线终点附近生成是否停车的决策                                                                 |
| [planning-traffic-rules-rerouting](modules/planning/traffic_rules/rerouting/README_cn.md)                                              | 重新规划规则插件包，在规划失败的情况下决策是否要重新规划                                                                 |
| [planning-traffic-rules-stop-sign](modules/planning/traffic_rules/stop_sign/README_cn.md)                                              | 停止标志插件包，在停止标志附近决策是否要停车                                                                             |
| [planning-traffic-rules-traffic-light](modules/planning/traffic_rules/traffic_light/README_cn.md)                                      | 交通灯规则插件包，在交通灯附近决策是否要停车                                                                             |
| [planning-traffic-rules-yield-sign](modules/planning/traffic_rules/yield_sign/README_cn.md)                                            | 让行规则插件包，在让行标志附近生成速度决策                                                                               |
