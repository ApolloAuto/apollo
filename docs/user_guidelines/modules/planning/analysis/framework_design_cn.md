# 框架设计

## 框架设计

![image (7).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%287%29_c99a812.png)

Planning 模块从 apollo 3.5 开始使用了双层状态机的场景机制，相比于 apollo 3.0 之前的版本，每个驾驶用例都被当作不同的驾驶场景，现在在特定场景中报告的问题可以在不影响其他场景的工作的情况下得到修复，其中问题修复不会影响其他驾驶用例。

双层状态机 Top Layer 是 Scenario 状态机，BottomLayer 是 Stage 状态机。

在 Top Layer 中进行场景切换，ScenarioManager 根据周围的环境和地图信息，决定需要切换到哪个场景，如 LaneFollow 沿道路行驶的场景，PullOver 靠边停车的场景。

在 Bottom Layer 中进行 Stage 的切换，如果上个 Stage 已经执行完成，继续执行下一个 Stage。如果所有的 Stage 都执行完毕，认为当前 Scenario 也执行完毕。

![image (8).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%288%29_ff1f212.png)

## 输入输出

### 输入

Planning 模块需要获取外部环境信息，车辆自身信息进行轨迹规划，以下是 planning 的外部输入信息：

<table><thead><tr><th>信息分类</th><th>信息</th><th>说明</th><th>数据类型</th><th>topic name</th></tr></thead><tbody><tr><td rowspan="2">外部环境信息</td><td>障碍物预测信息</td><td>perception 模块输出的障碍物信息，经过 prediction 处理后增添速度，加速度，和预测轨迹等信息。</td><td>prediction::PredictionObstacles</td><td>/apollo/prediction</td></tr><tr><td>交通灯感知信息</td><td>perception 模块输出的交通灯感知信息，包含交通灯亮起的颜色，id 等信息。</td><td>perception::TrafficLight</td><td>/apollo/perception/traffic_light</td></tr><tr><td rowspan="2">车辆信息</td><td>车辆定位</td><td>localization 输出的车辆位姿信息。</td><td>localization::LocalizationEstimate</td><td>/apollo/localization/pose</td></tr><tr><td>底盘状态</td><td>canbus 模块输出的车辆底盘信息，包含底盘速度，油门，刹车，档位，灯光等状态。</td><td>canbus::Chassis</td><td>/apollo/canbus/chassis</td></tr></tbody></table>

此外，planning 模块还需要外部输入的导航命令信息，用户首先向 external_command 发送导航命令请求，external_command 再将这些命令进行处理后转发给 planning 模块。下面介绍用户可以发送的几种导航命令：

<table><thead><tr><th>信息分类</th><th>信息</th><th>说明</th><th>数据类型</th><th>topic name</th></tr></thead><tbody><tr><td rowspan="3"><br>外部命令</td><td>沿车道线行驶命令</td><td>基于高精地图导航的命令，给定终点的位置或朝向，从当前车辆位置导航到目标终点位置。</td><td>external_command::LaneFollowCommand</td><td>/apollo/external_command/lane_follow</td></tr><tr><td>泊车命令</td><td>从当前位置导航泊车到停车位上。</td><td>external_command::ValetParkingCommand</td><td>/apollo/external_command/valet_parking</td></tr><tr><td>流程操作命令</td><td>HMI发送的流程操作命令，包括紧急靠边停车（PULL_OVER），紧急停车（STOP），继续行驶（CRUISE）等命令，目前只对PULL_OVER和STOP命令响应。</td><td>planning::ActionCommand</td><td>/apollo/external_command/action</td></tr></tbody></table>

### 输出

<table><thead><tr><th>信息</th><th>说明</th><th>数据类型</th><th>topic name</th></tr></thead><tbody><tr><td>规划轨迹</td><td>输出规划轨迹，包含轨迹点，速度和时间等信息。</td><td>planning::ADCTrajectory</td><td>/apollo/planning</td></tr><tr><td>导航状态</td><td>导航命令的执行状态。</td><td>external_command::CommandStatus</td><td>/apollo/planning/command_status</td></tr><tr><td>重新路由的请求</td><td>在道路被阻塞，换道失败超时时，发送重新路由的申请。</td><td>external_command::LaneFollowCommand</td><td>/apollo/external_command/lane_follow</td></tr></tbody></table>

## 设计概览

### Planning 模块入口

#### 模块注册

Planning 模块的入口为`planning_component.h`和`planning_component.cc`两个文件，实现的功能如下：

modules/planning/planning_base/planning_component.h：

```bash
// 订阅和发布消息
  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::LaneFollowCommand,
                            apollo::external_command::CommandStatus>>
      rerouting_client_;
  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;
  std::shared_ptr<cyber::Reader<storytelling::Stories>> story_telling_reader_;
  std::shared_ptr<cyber::Reader<PlanningCommand>> planning_command_reader_;

  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;
  std::shared_ptr<cyber::Writer<PlanningLearningData>>
      planning_learning_data_writer_;
  std::shared_ptr<cyber::Writer<external_command::CommandStatus>>
      command_status_writer_;

// 在Cyber中注册模块
CYBER_REGISTER_COMPONENT(PlanningComponent)
```

#### 模块初始化

除了注册模块，订阅和发布消息之外，planning 模块实现了 2 个主要函数`init`和`proc`。
Init 中实现了模块的初始化：

modules/planning/planning_base/planning_component.cc：

```bash
if (FLAGS_use_navigation_mode) {
    planning_base_ = std::make_unique<NaviPlanning>(injector_);
} else {
    planning_base_ = std::make_unique<OnLanePlanning>(injector_);
}
```

上面实现了 2 种 Planning 的注册，planning 模块根据配置选择不同的 Planning 实现方式，`FLAGS_use_navigation_mode`在 Planning 模块的`conf`目录中，因为默认为 false，Planning 默认情况下的实现是`OnLanePlanning`。下面介绍下这 2 种 Planning 的区别。

- NaviPlanning - 主要用于高速公路的导航规划，
- OnLanePlanning - 主要的应用场景是开放道路的自动驾驶。

![image (12).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2812%29_5477fb4.png)

可以看到`NaviPlanning`和`OnLanePlanning`都继承自同一个基类，并且在 PlanningComponent 中通过配置选择一个具体的实现进行注册。

Init 接下来实现了具体的消息发布和消息订阅，我们只看具体的一个例子：

modules/planning/planning_base/planning_component.cc：

```bash
// 订阅planning输入导航命令的消息
planning_command_reader_ = node_->CreateReader<PlanningCommand>(
      config_.topic_config().planning_command_topic(),
      [this](const std::shared_ptr<PlanningCommand>& planning_command) {
        AINFO << "Received planning data: run planning callback."
              << planning_command->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        planning_command_.CopyFrom(*planning_command);
      });
// 订阅红绿灯的消息
traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      config_.topic_config().traffic_light_detection_topic(),
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received traffic light data: run traffic light callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
  });
// 订阅planning流程干预消息
pad_msg_reader_ = node_->CreateReader<PadMessage>(
      config_.topic_config().planning_pad_topic(),
      [this](const std::shared_ptr<PadMessage>& pad_msg) {
        ADEBUG << "Received pad data: run pad callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
  });
// 创建planning输出轨迹发布者
planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());
// 道路被阻塞时重规划的client
rerouting_client_ =
      node_->CreateClient<apollo::external_command::LaneFollowCommand,
                      external_command::CommandStatus>(
      config_.topic_config().routing_request_topic());
// 创建planning导航命令实时执行状态发布者
command_status_writer_ = node_->CreateWriter<external_command::CommandStatus>(
      FLAGS_planning_command_status);
```

至此，Planning 模块的初始化就完成了。

#### 模块运行

Proc 的主要是检查数据，并且执行注册好的 Planning，生成路线并且发布。

modules/planning/planning_base/planning_component.cc：

```bash
bool PlanningComponent::Proc(...) {
  // 1. 检查是否需要重新规划线路。
  CheckRerouting();
﻿
  // 2. 数据放入local_view_中，并且检查输入数据。
  ...

  // 3. 执行注册好的Planning，生成线路。
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
﻿
  // 4. 发布消息
  planning_writer_->Write(std::make_shared<ADCTrajectory>(adc_trajectory_pb));
}
```

整个`PlanningComponent`的分析就完成了，由于默认的 Planning 是开放道路的 OnLanePlanning，我们接下来主要分析这个 Planning。

### OnLanePlanning

每次 Planning 会根据以下 2 个信息作为输入来执行：

- Planning上下文信息，
- Frame 结构体（车辆信息，位置信息等所有规划需要用到的信息，在`/planning/common/frame.h`中）。

modules/planning/planning_base/common/frame.h：

```bash
uint32_t sequence_num_ = 0;
LocalView local_view_;
const hdmap::HDMap *hdmap_ = nullptr;
common::TrajectoryPoint planning_start_point_;
common::VehicleState vehicle_state_;
std::list<ReferenceLineInfo> reference_line_info_;

bool is_near_destination_ = false;

// 车辆最终选择行驶的参考线信息
const ReferenceLineInfo *drive_reference_line_info_ = nullptr;

ThreadSafeIndexedObstacles obstacles_;

std::unordered_map<std::string, const perception::TrafficLight *> traffic_lights_;

// 当前帧中规划并发布的轨迹
ADCTrajectory current_frame_planned_trajectory_;
// 当前帧中规划的路径
DiscretizedPath current_frame_planned_path_;

const ReferenceLineProvider *reference_line_provider_ = nullptr;

OpenSpaceInfo open_space_info_;

std::vector<routing::LaneWaypoint> future_route_waypoints_;

common::monitor::MonitorLogBuffer monitor_logger_buffer_;
```

#### 初始化

OnLanePlanning 的初始化逻辑在 Init 中，主要实现分配具体的 Planner，启动参考线提供器（`reference_line_provider_`），代码分析如下：

modules/planning/planning_base/on_lane_planning.cc：

```bash
Status OnLanePlanning::Init(const PlanningConfig& config) {
  ...

  // 启动参考线提供器，会另启动一个线程，执行一个定时任务，每隔50ms提供一次参考线。
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(
      injector_->vehicle_state(), reference_line_config);
  reference_line_provider_->Start();

  // 为Planning分配具体的Planner。
  planner_ = planner_dispatcher_->DispatchPlanner(config_, injector_);
  ...
}
```

可以看到`DispatchPlanner`在`OnLanePlanning`实例化的时候就指定了。

modules/planning/planning_base/on_lane_planning.h：

```bash
class OnLanePlanning : public PlanningBase {
 public:
  explicit OnLanePlanning(const std::shared_ptr<DependencyInjector>& injector)
      : PlanningBase(injector) {
    planner_dispatcher_ = std::make_unique<OnLanePlannerDispatcher>();
  }
```

在看`OnLanePlannerDispatcher`具体的实现，也是根据配置选择具体的`Planner`，默认为`PUBLIC_ROAD`规划器：

modules/planning/planning_base/planner/on_lane_planner_dispatcher.cc：

```bash
std::unique_ptr<Planner> OnLanePlannerDispatcher::DispatchPlanner(
    const PlanningConfig& planning_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  return planner_factory_.CreateObject(
      planning_config.standard_planning_config().planner_type(0), injector);
}
```

#### 事件触发

OnLanePlanning 的主要逻辑在`RunOnce()`中，在 Apollo 3.5 之前是定时器触发，3.5 改为事件触发，即收到对应的消息之后，就触发执行，这样做的好处是增加了实时性参考。

modules/planning/planning_base/on_lane_planning.cc：

```bash
void OnLanePlanning::RunOnce(const LocalView& local_view,
                             ADCTrajectory* const ptr_trajectory_pb) {

  // 初始化Frame
  status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);
  ...

  // 判断是否符合交通规则
  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
    auto traffic_status =
        traffic_decider_.Execute(frame_.get(), &ref_line_info);
    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
      AWARN << "Reference line " << ref_line_info.Lanes().Id()
            << " traffic decider failed";
    }
  }

  // 执行计划
  status = Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb);

  ...
}


Status OnLanePlanning::Plan(
    const double current_time_stamp,
    const std::vector<TrajectoryPoint>& stitching_trajectory,
    ADCTrajectory* const ptr_trajectory_pb) {

  ...

  // 调用具体的(PUBLIC_ROAD)Planner执行
  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get(),
                               ptr_trajectory_pb);
  ...
}

```

上述就是`OnLanePlanning`的执行过程，先是 Planner 分发器根据配置，选择具体的 planner，然后初始化 Frame，（`PUBLIC_ROAD`）planner 根据输入帧执行`Plan`方法。

### Planner

我们先看下 Planner 目录结构，一共实现了 4 种 Planner：

```bash
.
├── BUILD
├── navi_planner_dispatcher.cc
├── navi_planner_dispatcher.h
├── navi_planner_dispatcher_test.cc
├── on_lane_planner_dispatcher.cc
├── on_lane_planner_dispatcher.h
├── on_lane_planner_dispatcher_test.cc
├── planner_dispatcher.cc
├── planner_dispatcher.h
├── planner.h
├── lattice           // lattice planner
├── navi              // navi planner
├── public_road       // public road planner
└── rtk               // rtk planner
```

可以看到 Planner 目录分别实现了 Planner 发布器和具体的 Planner，关于发布器我们后面会根据流程图来介绍，这里先介绍一下 4 种不同的 Planner。

- rtk- 根据录制的轨迹来规划行车路线，
- public_road- 开放道路的轨迹规划器，
- lattice- 基于网格算法的轨迹规划器，
- navi- 基于实时相对地图的规划器。

#### Planner 注册场景

Planning 模块的整体流程如下：

![image (13).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2813%29_3b05ca5.png)

1. PlanningComponent 在 cyber 中注册，
2. 选择 Planning，
3. 根据不同的 Dispatcher，分发 Planner。

下面我们主要介绍`PublicRoadPlanner`，主要的实现还是在 Init 和 Plan 中。init 中主要是注册规划器支持的场景（scenario）。

modules/planning/planning_base/planner/public_road/public_road_planner.cc：

```bash
Status PublicRoadPlanner::Init(const PlanningConfig& config) {
  config_ = config;
  // 根据配置注册不同的场景
  scenario_manager_.Init(config);
  return Status::OK();
}

```

`PublicRoadPlanner`支持的场景配置如下：

modules/planning/planning_base/conf/planning_config.pb.txt：

```bash

standard_planning_config {
  planner_type: PUBLIC_ROAD
  planner_public_road_config {
    scenario {
      name: "EMERGENCY_PULL_OVER"
      type: "EmergencyPullOverScenario"
    }
    scenario {
      name: "EMERGENCY_STOP"
      type: "EmergencyStopScenario"
    }
    scenario {
      name: "VALET_PARKING"
      type: "ValetParkingScenario"
    }
    scenario {
      name: "BARE_INTERSECTION_UNPROTECTED"
      type: "BareIntersectionUnprotectedScenario"
    }
    scenario {
      name: "STOP_SIGN_UNPROTECTED"
      type: "StopSignUnprotectedScenario"
    }
    scenario {
      name: "YIELD_SIGN"
      type: "YieldSignScenario"
    }
    scenario {
      name: "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN"
      type: "TrafficLightUnprotectedLeftTurnScenario"
    }
    scenario {
      name: "TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN"
      type: "TrafficLightUnprotectedRightTurnScenario"
    }
    scenario {
      name: "TRAFFIC_LIGHT_PROTECTED"
      type: "TrafficLightProtectedScenario"
    }
    scenario {
      name: "PULL_OVER"
      type: "PullOverScenario"
    }
    scenario {
      name: "PARK_AND_GO"
      type: "ParkAndGoScenario"
    }
    scenario {
      name: "LANE_FOLLOW"
      type: "LaneFollowScenario"
    }
  }
}
```

#### 运行场景

运行场景在 Plan 函数中进行：

modules/planning/planning_base/planner/public_road/public_road_planner.cc：

```bash

Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory) {
  scenario_manager_.Update(planning_start_point, frame);
  scenario_ = scenario_manager_.mutable_scenario();
  if (!scenario_) {
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "Unknown Scenario");
  }
  auto result = scenario_->Process(planning_start_point, frame);

  if (FLAGS_enable_record_debug) {
    auto scenario_debug = ptr_computed_trajectory->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_scenario();
    scenario_debug->set_scenario_type(scenario_->Name());
    scenario_debug->set_stage_type(scenario_->GetStage());
    scenario_debug->set_msg(scenario_->GetMsg());
  }

  if (result.GetScenarioStatus() == ScenarioStatusType::STATUS_DONE) {
    // only updates scenario manager when previous scenario's status is
    // STATUS_DONE
    scenario_manager_.Update(planning_start_point, frame);
  } else if (result.GetScenarioStatus() == ScenarioStatusType::STATUS_UNKNOWN) {
    return Status(common::PLANNING_ERROR,
                  result.GetTaskStatus().error_message());
  }
  return Status(common::OK, result.GetTaskStatus().error_message());
}
```

可以看到`Planner`模块把具体的规划转化成一系列的场景，每次执行规划之前先判断更新当前的场景，然后针对具体的场景去执行。

### Scenario

scenario 的父类声明在`modules/planning/planning_base/scenario_base/scenario.h`中，每个具体的 scenario 都继承自父类 Scenario，并且是单独的一个插件包。scenario 子类的插件包在`modules/planning/scenarios`目录下，当前 scenario 支持的场景如下：

modules/planning/scenarios：

```bash
.
├── bare_intersection_unprotected          // 无保护交叉路口场景
├── emergency_pull_over                    // 紧急靠边停车场景
├── emergency_stop                         // 紧急停车场景
├── lane_follow                            // 沿车道线行驶场景
├── park_and_go                            // 靠边停车后沿车道线行驶场景
├── pull_over                              // 终点靠边停车场景
├── stop_sign_unprotected                  // 无保护停止标志场景
├── traffic_light_protected                // 有保护交通灯场景
├── traffic_light_unprotected_left_turn    // 无保护左转场景
├── traffic_light_unprotected_right_turn   // 无保护右转场景
├── valet_parking                          // 泊车场景
├── yield_sign                             // 让行标志场景
```

其中需要知道场景如何转换，以及每种场景如何执行。常用的场景是`lane_follow`。

#### 场景转换

场景转换的实现在`scenario_manager.cc`中，其中实现了场景注册，创建场景和更新场景的功能。

modules/planning/planning_base/scenario_base/scenario_manager.cc：

```bash
bool ScenarioManager::Init(const PlanningConfig& planning_config) {
  if (init_) {
    return true;
  }
  if (!planning_config.has_standard_planning_config()) {
    AERROR << "lost standard_planning_config" << planning_config.DebugString();
    return false;
  }
  if (!planning_config.standard_planning_config()
           .has_planner_public_road_config()) {
    AERROR << "lost planner_public_road_config"
           << planning_config.DebugString();
    return false;
  }
  const auto& public_road_config =
      planning_config.standard_planning_config().planner_public_road_config();
  // 从配置文件中读取支持的场景列表，并创建相应的场景
  for (int i = 0; i < public_road_config.scenario_size(); i++) {
    auto scenario = PluginManager::Instance()->CreateInstance<Scenario>(
        ConfigUtil::GetFullPlanningClassName(
            public_road_config.scenario(i).type()));
    ACHECK(scenario->Init(injector_, public_road_config.scenario(i).name()))
        << "Can not init scenario" << public_road_config.scenario(i).name();
    scenario_list_.push_back(scenario);
    if (public_road_config.scenario(i).name() == "LANE_FOLLOW") {
      default_scenario_type_ = scenario;
    }
  }
  AINFO << "Load scenario list:" << public_road_config.DebugString();
  // 设置当前的场景
  current_scenario_ = default_scenario_type_;
  init_ = true;
  return true;
}

```

在`PublicRoadPlanner::Plan`函数中通过调用`ScenarioManager::Update`函数进行场景的切换：

modules/planning/planning_base/scenario_base/scenario_manager.cc：

```bash
void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             Frame* frame) {
  CHECK_NOTNULL(frame);
  for (auto scenario : scenario_list_) {
    if (current_scenario_.get() == scenario.get() &&
        current_scenario_->GetStatus() ==
            ScenarioStatusType::STATUS_PROCESSING) {
      // The previous scenario has higher priority
      return;
    }
    // 具体的场景子类实现IsTransferable，根据当前环境判断是否可以切换到自身的场景中，
    // 如果可以切换，则不再遍历后面的场景
    if (scenario->IsTransferable(current_scenario_.get(), *frame)) {
      current_scenario_->Exit(frame);
      AINFO << "switch scenario from" << current_scenario_->Name() << " to "
            << scenario->Name();
      current_scenario_ = scenario;
      current_scenario_->Reset();
      current_scenario_->Enter(frame);
      return;
    }
  }
}
```

其中场景的状态切换参考下图：

![image (14).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2814%29_818c1ef.png)

#### 场景运行

场景的执行在`scenario.cc`和对应的场景目录中，实际上每个场景又分为一个或者多个阶段(stage)，每个阶段又由不同的任务(task)组成。执行一个场景，就是顺序执行不同阶段的不同任务。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image_d7dc794.png)

以 TrafficLightProtectedScenario 为例，它的 pipeline 在插件包下的配置文件`modules/planning/scenarios/traffic_light_protected/conf/pipeline.pb.txt`中设定：

modules/planning/scenarios/traffic_light_protected/conf/pipeline.pb.txt：

```bash
stage: {
  name: "TRAFFIC_LIGHT_PROTECTED_APPROACH"
  type: "TrafficLightProtectedStageApproach"
  enabled: true
  task {
    name: "LANE_FOLLOW_PATH"
    type: "LaneFollowPath"
  }
  task {
    name: "LANE_BORROW_PATH"
    type: "LaneBorrowPath"
  }
  task {
    name: "FALLBACK_PATH"
    type: "FallbackPath"
  }
  task {
    name: "PATH_DECIDER"
    type: "PathDecider"
  }
  task {
    name: "RULE_BASED_STOP_DECIDER"
    type: "RuleBasedStopDecider"
  }
  task {
    name: "ST_BOUNDS_DECIDER"
    type: "STBoundsDecider"
  }
  task {
    name: "SPEED_BOUNDS_PRIORI_DECIDER"
    type: "SpeedBoundsDecider"
  }
  task {
    name: "SPEED_HEURISTIC_OPTIMIZER"
    type: "PathTimeHeuristicOptimizer"
  }
  task {
    name: "SPEED_DECIDER"
    type: "SpeedDecider"
  }
  task {
    name: "SPEED_BOUNDS_FINAL_DECIDER"
    type: "SpeedBoundsDecider"
  }
  task {
    name: "PIECEWISE_JERK_SPEED"
    type: "PiecewiseJerkSpeedOptimizer"
  }
}
stage: {
  name: "TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE"
  type: "TrafficLightProtectedStageIntersectionCruise"
  enabled: true
  task {
    name: "LANE_FOLLOW_PATH"
    type: "LaneFollowPath"
  }
  task {
    name: "LANE_BORROW_PATH"
    type: "LaneBorrowPath"
  }
  task {
    name: "FALLBACK_PATH"
    type: "FallbackPath"
  }
  task {
    name: "PATH_DECIDER"
    type: "PathDecider"
  }
  task {
    name: "RULE_BASED_STOP_DECIDER"
    type: "RuleBasedStopDecider"
  }
  task {
    name: "ST_BOUNDS_DECIDER"
    type: "STBoundsDecider"
  }
  task {
    name: "SPEED_BOUNDS_PRIORI_DECIDER"
    type: "SpeedBoundsDecider"
  }
  task {
    name: "SPEED_HEURISTIC_OPTIMIZER"
    type: "PathTimeHeuristicOptimizer"
  }
  task {
    name: "SPEED_DECIDER"
    type: "SpeedDecider"
  }
  task {
    name: "SPEED_BOUNDS_FINAL_DECIDER"
    type: "SpeedBoundsDecider"
  }
  task {
    name: "PIECEWISE_JERK_SPEED"
    type: "PiecewiseJerkSpeedOptimizer"
  }
}
```

由于 Stage 都是顺序执行，如果当前的 Stage 没有执行结束，下一个周期继续执行；如果已经执行结束，直接切换到下一个 Stage 执行：

modules/planning/planning_base/scenario_base/scenario.cc：

```bash
ScenarioResult Scenario::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  if (current_stage_ == nullptr) {
    current_stage_ = CreateStage(
        *stage_pipeline_map_[scenario_pipeline_config_.stage(0).name()]);
    AINFO << "Create stage " << current_stage_->Name();
  }
  if (current_stage_->Name().empty()) {
    scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_DONE);
    return scenario_result_;
  }
  auto ret = current_stage_->Process(planning_init_point, frame);
  scenario_result_.SetStageResult(ret);
  switch (ret.GetStageStatus()) {
    case StageStatusType::ERROR: {
      AERROR << "Stage '" << current_stage_->Name() << "' returns error";
      scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_UNKNOWN);
      break;
    }
    // 下个周期继续执行当前Stage
    case StageStatusType::RUNNING: {
      scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_PROCESSING);
      break;
    }
    // 切换下一个Stage
    case StageStatusType::FINISHED: {
      auto next_stage = current_stage_->NextStage();
      ... ...
    }
    default: {
      AWARN << "Unexpected Stage return value: "
            << static_cast<int>(ret.GetStageStatus());
      scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_UNKNOWN);
    }
  }
  return scenario_result_;
}

```

以`TrafficLightProtectedStageApproach`为例，它的`Process`函数的执行如下：

modules/planning/scenarios/traffic_light_protected/stage_approach.cc：

```bash
StageResult TrafficLightProtectedStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Approach";
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(context_);

  auto context = GetContextAs<TrafficLightProtectedContext>();
  const ScenarioTrafficLightProtectedConfig& scenario_config =
      context->scenario_config;
  // 根据参考线规划
  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "TrafficLightProtectedStageApproach planning error";
  }

  if (context->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  ... ...

  if (traffic_light_all_done) {
    return FinishStage();
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}

```

其中`ExecuteTaskOnReferenceLine`沿参考线规划，执行当前 Stage 中所包含的 Task 任务。

modules/planning/planning_base/scenario_base/stage.cc：

```bash
StageResult Stage::ExecuteTaskOnReferenceLine(
    const common::TrajectoryPoint& planning_start_point, Frame* frame) {
  StageResult stage_result;
  if (frame->reference_line_info().empty()) {
    AERROR << "referenceline is empty in stage" << name_;
    return stage_result.SetStageStatus(StageStatusType::ERROR);
  }
  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
    if (!reference_line_info.IsDrivable()) {
      AERROR << "The generated path is not drivable";
      return stage_result.SetStageStatus(StageStatusType::ERROR);
    }
    // 执行Stage中包含的Task任务
    for (auto task : task_list_) {
      const double start_timestamp = Clock::NowInSeconds();

      const auto ret = task->Execute(frame, &reference_line_info);

      const double end_timestamp = Clock::NowInSeconds();
      const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
      ADEBUG << "after task[" << task->Name()
             << "]: " << reference_line_info.PathSpeedDebugString();
      ADEBUG << task->Name() << " time spend: " << time_diff_ms << " ms.";
      RecordDebugInfo(&reference_line_info, task->Name(), time_diff_ms);

      if (!ret.ok()) {
        stage_result.SetTaskStatus(ret);
        AERROR << "Failed to run tasks[" << task->Name()
               << "], Error message: " << ret.error_message();
        break;
      }
    }
    ... ...
    return stage_result;
  }
  return stage_result;
}

```

上面是用`ExecuteTaskOnReferenceLine`中的`Process`来举例子，不同场景中的`Process`实现可能也不一样，这样设计的好处是，当发现一个场景有问题，需要修改不会影响到其他的场景。同时也可以针对不同场景做优化，比通用的规划更加适合单独的场景。每种场景都有一个专门的目录来进行优化。

### Task

Task 的父类定义在`modules/planning/planning_base/task_base/task.h`中，具体的任务继承自 Task 父类，是一个独立的插件包，具体的任务子类在`modules/planning/tasks`目录下：

modules/planning/tasks：

```bash
.
├── fallback_path                            // 备用路径生成
├── lane_borrow_path                         // 借道路径生成
├── lane_change_path                         // 换道路径生成
├── lane_follow_path                         // 沿当前车道直行路径生成
├── open_space_fallback_decider              // 换道路径生成
├── open_space_pre_stop_decider              // 开始进行openspace泊车前，计算车辆现在停车位附近停止，准备泊车的位置
├── open_space_roi_decider                   // 泊车时可行区域计算的
├── open_space_trajectory_partition          // 泊车轨迹分割，按照前进倒车将轨迹进行分割
├── open_space_trajectory_provider           // 泊车轨迹生成
├── path_decider                             // 对路径上的障碍物做决策，是否要纵向停止，或者横向忽略
├── path_reference_decider                   //
├── path_time_heuristic                      // 速度动态规划
├── piecewise_jerk_speed                     // 基于二次规划的速度规划
├── piecewise_jerk_speed_nonlinear           // 基于非线性规划的速度规划
├── pull_over_path                           // 靠边停车路径生成
├── reuse_path                               // 处理路径是否重用
├── rss_decider                              // 上报RSS信息
├── rule_based_stop_decider                  // 根据规则设置车辆停止墙
├── speed_bounds_decider                     // 生成速度边界
├── speed_decider                            // 根据速度动态规划的speed_profile，设置障碍物标签
├── st_bounds_decider                        // 生成ST图，为速度动态规划提供可搜索空间

```
