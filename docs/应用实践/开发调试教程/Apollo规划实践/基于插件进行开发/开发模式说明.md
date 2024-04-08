您可以通过新增插件的形式对规划模块进行二次开发。Apollo 规划模块采用双状态机的架构来组织决策和运动规划任务。在Apollo 9.0 中对规划模块基于双状态机机制进行了插件化改造，规划模块运行时基于配置文件对插件的动态加载运行，实现了乐高式开发，自由组合场景（Scenario）和功能任务（Task）

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_a4c3288.png)

planning base 为规划模块基座，负责接受和发布和其他模块的通信信息，生成参考线，提供基础数据结构和基础库的功能。双层状态机的第一层为 Scenario 插件，每个Scenario插件通常会包含其独有的Stage插件，每个 Stage 插件会从任务库中编排不同的任务(Task)来执行。

## Scenario插件的二次开发

Scenario 可以根据地理位置来划分，当场景中主车在特定区域规划动作和默认场景（Lane Follow）存在有所不同是，为了避免影响默认场景的运行，可以为其开发一个新的场景，比如前方有红绿灯需要开发红绿灯场景，前方停止牌需要开发停止牌场景；

Scenario 也可根据命令来划分，当接收到紧急靠边停车命令时，进入紧急靠边停车场景。比如接收到泊车命令时，进入泊车场景。

开发一个新的 Scenario 需要继承 Scenario 基类：

```bash
class Scenario {
 public:
  Scenario();

  virtual ~Scenario() = default;

  virtual bool Init(std::shared_ptr<DependencyInjector> injector,
                    const std::string& name);

  /**
   * @brief Get the scenario context.
   */
  virtual ScenarioContext* GetContext() = 0;

  /**
   * Each scenario should define its own transfer condition, i.e., when it
   * should allow to transfer from other scenario to itself.
   */
  virtual bool IsTransferable(const Scenario* other_scenario,
                              const Frame& frame) {
    return false;
  }

  virtual ScenarioResult Process(
      const common::TrajectoryPoint& planning_init_point, Frame* frame);

  virtual bool Exit(Frame* frame) { return true; }

  virtual bool Enter(Frame* frame) { return true; }

  /**
   * Each scenario should define its own stages object's creation
   * scenario will call stage's Stage::Process function following a configured
   * order, The return value of Stage::Process function determines the
   * transition from one stage to another.
   */
  std::shared_ptr<Stage> CreateStage(const StagePipeline& stage_pipeline);

  const ScenarioStatusType& GetStatus() const {
    return scenario_result_.GetScenarioStatus();
  }

  const std::string GetStage() const;

  const std::string& GetMsg() const { return msg_; }

  const std::string& Name() const { return name_; }

  /**
   * @brief Reset the scenario, used before entering the scenario.
   */
  void Reset();

 protected:
  template <typename T>
  bool LoadConfig(T* config);

  ScenarioResult scenario_result_;
  std::shared_ptr<Stage> current_stage_;
  std::unordered_map<std::string, const StagePipeline*> stage_pipeline_map_;
  std::string msg_;  // debug msg
  std::shared_ptr<DependencyInjector> injector_;

  std::string config_path_;
  std::string config_dir_;
  std::string name_;
  ScenarioPipeline scenario_pipeline_config_;
};

template <typename T>
bool Scenario::LoadConfig(T* config) {
  return apollo::cyber::common::GetProtoFromFile(config_path_, config);
}
```

这里需要您实现几个函数。

### 场景初始化

场景的初始化需要继承 Scenario 的 Init() 函数，场景基类的 Init 函数主要是从场景插件中加载场景的流水线，将加载的 Stage 实例保存到 `stage_pipeline_map_`中。如果场景自身还有配置文件，则可以调用`Scenario::LoadConfig<T>()`函数加载场景自身的配置文件，保存到场景实例上下文变量中`context_`。

下面是一个靠边停车 Scenario 初始化案例，首先调用基类的 Init 函数，加载 Scenario 所包含的 Stage，然后调用`Scenario::LoadConfig<ScenarioPullOverConfig>()`加载靠边停车场景的配置文件保存到靠边停车场景上下文变量`context_`中，上下文变量可以在 Scenario 和 Stage 间传递配置和数据。

```bash
bool PullOverScenario::Init(std::shared_ptr<DependencyInjector> injector,
                            const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<ScenarioPullOverConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();
    return false;
  }

  init_ = true;
  return true;
}

struct PullOverContext : public ScenarioContext {
  ScenarioPullOverConfig scenario_config;
};
```

### 场景切换函数

场景切换函数主要是从场景管理器中调用，判断是否需要切入该场景。场景切换函数继承与 Scenario 基类的 IsTransferable()，判断当前帧是否可以切入该场景，
如果返回 true，则会切入该场景，不再进行后续场景的判断。场景切入包括基于地理位置触发的方式，比如停止标记场景：

从参考线获得首个 overlap 是否是停止标记，如果是且主车距离停止标记满足设定的距离，则进入停止标记场景：

```bash
bool StopSignUnprotectedScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    return false;
  }
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& first_encountered_overlaps =
      reference_line_info.FirstEncounteredOverlaps();
  // note: first_encountered_overlaps already sorted
  hdmap::PathOverlap* stop_sign_overlap = nullptr;
  for (const auto& overlap : first_encountered_overlaps) {
    if (overlap.first == ReferenceLineInfo::SIGNAL ||
        overlap.first == ReferenceLineInfo::YIELD_SIGN) {
      return false;
    } else if (overlap.first == ReferenceLineInfo::STOP_SIGN) {
      stop_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      break;
    }
  }
  if (stop_sign_overlap == nullptr) {
    return false;
  }
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_stop_sign =
      stop_sign_overlap->start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_stop_sign[" << adc_distance_to_stop_sign
         << "] stop_sign[" << stop_sign_overlap->object_id
         << "] stop_sign_overlap_start_s[" << stop_sign_overlap->start_s << "]";
  const bool stop_sign_scenario =
      (adc_distance_to_stop_sign > 0.0 &&
       adc_distance_to_stop_sign <=
           context_.scenario_config.start_stop_sign_scenario_distance());

  return stop_sign_scenario;
}
```

还有一种场景切入方式是基于命令触发的，比如说紧急靠边停车场景，其切入条件就是收到 pad_msg 的命令即进入紧急靠边停车场景：

```bash
bool EmergencyPullOverScenario::IsTransferable(
    const Scenario* const other_scenario, const Frame& frame) {
  if (frame.reference_line_info().empty()) {
    return false;
  }
  const auto& pad_msg_driving_action = frame.GetPadMsgDrivingAction();
  if (pad_msg_driving_action == PadMessage::PULL_OVER) {
    return true;
  }
  return false;
}
```

### 场景进入、退出函数

场景的进入函数继承于基类的 Enter() 函数，在首次进入场景前调用做一些预处理的工作，重置场景内变量。比如在停止标记场景的 Enter() 函数，首先寻找参考线的停止标记 id 保存到上下文变量中，然后重置停止标记的全局变量。

```bash
bool StopSignUnprotectedScenario::Enter(Frame* frame) {
  const auto& reference_line_info = frame->reference_line_info().front();
  std::string current_stop_sign_overlap_id;
  const auto& overlaps = reference_line_info.FirstEncounteredOverlaps();
  for (auto overlap : overlaps) {
    if (overlap.first == ReferenceLineInfo::STOP_SIGN) {
      current_stop_sign_overlap_id = overlap.second.object_id;
      break;
    }
  }

  if (current_stop_sign_overlap_id.empty()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->Clear();
    AERROR << "Can not find stop sign overlap in refline";
    return false;
  }

  const std::vector<hdmap::PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();
  auto stop_sign_overlap_itr = std::find_if(
      stop_sign_overlaps.begin(), stop_sign_overlaps.end(),
      [&current_stop_sign_overlap_id](const hdmap::PathOverlap& overlap) {
        return overlap.object_id == current_stop_sign_overlap_id;
      });

  if (stop_sign_overlap_itr != stop_sign_overlaps.end()) {
    injector_->planning_context()
        ->mutable_planning_status()
        ->mutable_stop_sign()
        ->set_current_stop_sign_overlap_id(current_stop_sign_overlap_id);
    ADEBUG << "Update PlanningContext with first_encountered stop sign["
           << current_stop_sign_overlap_id << "] start_s["
           << stop_sign_overlap_itr->start_s << "]";
  } else {
    AERROR << "Can not find stop sign overlap " << current_stop_sign_overlap_id;
    return false;
  }

  hdmap::StopSignInfoConstPtr stop_sign = HDMapUtil::BaseMap().GetStopSignById(
      hdmap::MakeMapId(current_stop_sign_overlap_id));
  if (!stop_sign) {
    AERROR << "Could not find stop sign: " << current_stop_sign_overlap_id;
    return false;
  }
  context_.current_stop_sign_overlap_id = current_stop_sign_overlap_id;
  context_.watch_vehicles.clear();

  GetAssociatedLanes(*stop_sign);
  return true;
}
```

场景的退出函数继承于基类的 Exit() 函数，在场景切出时会被调用，可以用来清除一些全局变量，比如停止标记场景的切出函数

```bash
bool StopSignUnprotectedScenario::Exit(Frame* frame) {
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_stop_sign()
      ->Clear();
  return true;
}
```

### 场景运行函数

场景运行函数继承于基类的 Process() 函数，在每一帧场景运行时都会被调用，基类的 Process 主要用来创建 Stage、运行 Stage 的 Process 函数以及调度不同 Stage 的切换。

在 apollo 中的场景一般都默认采用基类的 Process() 函数。如果您有更多定制需求可以继承 Process 函数重写业务策略。

## 阶段的二次开发

Apollo 中的 Stage(阶段) 是 Scenario 下的第二层状态机，可以根据时间来划分。当场景中存在先后顺序的业务逻辑时，可以将其划分成多个 Stage。比如在红绿灯无保护左转场景中可以划分为三个阶段，第一个阶段是接近行驶到红绿灯停止线前的过程，第二个是红绿灯为绿灯时慢速观望的过程，第三个是对向直行车道通畅快速通过的过程。

```bash
stage: {
  name: "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_APPROACH"
  type: "TrafficLightUnprotectedLeftTurnStageApproach"
  enabled: true
  }
}
stage: {
  name: "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_CREEP"
  type: "TrafficLightUnprotectedLeftTurnStageCreep"
  enabled: true
}
stage: {
  name: "TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN_INTERSECTION_CRUISE"
  type: "TrafficLightUnprotectedLeftTurnStageIntersectionCruise"
  enabled: true
}
```

阶段划分好后，需要对阶段配置任务（Task）。在每一帧的规划中只会运行一个阶段，在一个阶段内会顺序执行阶段内的每一个任务。任务是处理路径规划或速度规划的最小计算单元。您可以复用 Apollo 中已有的任务进行配置，也可以根据《任务的二次开发》开发新的规划任务。任务的配置和任务参数的配置您可以参考《通过配置参数开发》一章了解详细内容。

### 阶段的初始化

在场景运行时会对创建场景包含的阶段实例，并对阶段初始化。如果需要对阶段初始化，您可以继承重写阶段的 Init() 函数，基类 Init() 函数主要用于加载任务的流水线，并对任务初始化。Apollo 中一般采用的是基类的 Init 函数：

```bash
bool Stage::Init(const StagePipeline& config,
                 const std::shared_ptr<DependencyInjector>& injector,
                 const std::string& config_dir, void* context) {
  pipeline_config_ = config;
  next_stage_ = config.name();
  injector_ = injector;
  name_ = config.name();
  context_ = context;
  injector_->planning_context()
      ->mutable_planning_status()
      ->mutable_scenario()
      ->set_stage_type(name_);
  std::string path_name = ConfigUtil::TransformToPathName(name_);
  // Load task plugin
  for (int i = 0; i < pipeline_config_.task_size(); ++i) {
    auto task = pipeline_config_.task(i);
    auto task_type = task.type();
    auto task_ptr = apollo::cyber::plugin_manager::PluginManager::Instance()
                        ->CreateInstance<Task>(
                            ConfigUtil::GetFullPlanningClassName(task_type));
    if (nullptr == task_ptr) {
      AERROR << "Create task " << task.name() << " of " << name_ << " failed!";
      return false;
    }
    std::string task_config_dir = config_dir + "/" + path_name;
    task_ptr->Init(task_config_dir, task.name(), injector);
    task_list_.push_back(task_ptr);
    tasks_[task.name()] = task_ptr;
  }
  return true;
}
```

### 阶段的运行

阶段的运行函数 Process() 主要用于维护当前阶段的状态。stage 包括三种状态 RUNNING、FINISHED 和 ERROR 三种状态。其中 RUNNING 表示当前状态正在运行，Scenario 将继续维持当前阶段运行；FINISHED 表示当前阶段完成，Scenario 将会切入下一个阶段运行；ERROR 表示当前规划存在严重故障。Scenario 会将其上报，主车将会刹停。

如果当前阶段完成，除了要返回当前阶段为 FINISHED，还需要指定下一个阶段的名称`next_stage_`，Scenario 将会根据当前阶段的`next_stage`切入下一个阶段。如果指定的`next_stage_`为空字符串，则 Scenario 会认为全部 Stage 已经运行完成，Scenario 也会返回完成的状态。

在 Stage 中依次调用 task，有两个基类的函数可以复用，一个是 ExecuteTaskOnReferenceLine，主要是用于主路上一系列任务的运行，一个是 ExecuteTaskOnOpenSpace，主要是用于开放空间一系列任务的运行。

```bash
  StageResult ExecuteTaskOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);

  StageResult ExecuteTaskOnOpenSpace(Frame* frame);
```

场景中的上下文数据会通过 void\* 指针的方式保存在 Stage 的`context_`中，如果需要读取场景的上下文数据，可以通过模板函数`GetContextAs()`对上下文变量进行解析：

```bash
  template <typename T>
  T* GetContextAs() const {
    return static_cast<T*>(context_);
  }
```

下面以无保护左转场景的接近红绿灯停止线阶段为例介绍阶段的运行过程。

首先通过 GetContextAs 解析来自于场景上下文数据`TrafficLightUnprotectedLeftTurnContext`，
如果当前阶段没有使能则跳过当前阶段，设置当前阶段的巡航速度为配置车速`approach_cruise_speed`，依次执行每个规划任务`ExecuteTaskOnReferenceLine()`，检查当前车道红绿灯均为绿灯，则进入下一个 Stage。如果找不到红绿灯则退出当前场景：

```bash
StageResult TrafficLightUnprotectedLeftTurnStageApproach::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  auto context = GetContextAs<TrafficLightUnprotectedLeftTurnContext>();
  const ScenarioTrafficLightUnprotectedLeftTurnConfig& scenario_config =
      context->scenario_config;
  if (!pipeline_config_.enabled()) {
    return FinishStage(frame);
  }

  // set cruise_speed to slow down
  frame->mutable_reference_line_info()->front().LimitCruiseSpeed(
      scenario_config.approach_cruise_speed());

  StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (result.HasError()) {
    AERROR << "TrafficLightUnprotectedLeftTurnStageApproach planning error";
  }

  if (context->current_traffic_light_overlap_ids.empty()) {
    return FinishScenario();
  }

  const auto& reference_line_info = frame->reference_line_info().front();

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  PathOverlap* traffic_light = nullptr;
  bool traffic_light_all_done = true;
  for (const auto& traffic_light_overlap_id :
       context->current_traffic_light_overlap_ids) {
    // get overlap along reference line
    PathOverlap* current_traffic_light_overlap =
        reference_line_info.GetOverlapOnReferenceLine(
            traffic_light_overlap_id, ReferenceLineInfo::SIGNAL);
    if (!current_traffic_light_overlap) {
      continue;
    }

    traffic_light = current_traffic_light_overlap;

    // set right_of_way_status
    reference_line_info.SetJunctionRightOfWay(
        current_traffic_light_overlap->start_s, false);

    const double distance_adc_to_stop_line =
        current_traffic_light_overlap->start_s - adc_front_edge_s;
    auto signal_color = frame->GetSignal(traffic_light_overlap_id).color();
    if (distance_adc_to_stop_line < 0) return FinishStage(frame);
    // check on traffic light color and distance to stop line
    if (signal_color != TrafficLight::GREEN ||
        distance_adc_to_stop_line >=
            scenario_config.max_valid_stop_distance()) {
      traffic_light_all_done = false;
      break;
    }
  }

  if (traffic_light == nullptr) {
    return FinishScenario();
  }

  if (traffic_light_all_done) {
    return FinishStage(frame);
  }

  return result.SetStageStatus(StageStatusType::RUNNING);
}
```

## 任务的二次开发

当 Apollo 中的任务 (Task) 无法满足您场景需求时，您需要开发全新的任务插件。Apollo 中存在多种类型的 Task 基类：

- PathGeneration：主要用于在主路上生成路径，比如规划借道路径 LaneBorrowPath、靠边停车路径 PullOverPath，沿车道行驶路径 LaneFollowPath等，

- SpeedOptimizer：主要用于在主路上规划速度曲线，比如基于二次规划的速度规划，基于非线性规划的速度规划，

- TrajectoryOptimizer：主要用于生成轨迹，比如开放空间规划 OpenSpaceTrajectoryProvider。

您也可以继承 Task 基类实现您的任务。

### 任务的初始化

Stage 在首次运行任务前，会调用任务的 Init 函数对任务进行初始化，基类的 Init 函数主要用于获取任务的默认参数路径`default_config_path_`和场景任务参数路径`config_path_`。

```bash
bool Task::Init(const std::string& config_dir, const std::string& name,
                const std::shared_ptr<DependencyInjector>& injector) {
  injector_ = injector;
  name_ = name;
  config_path_ =
      config_dir + "/" + ConfigUtil::TransformToPathName(name) + ".pb.txt";

  // Get the name of this class.
  int status;
  std::string class_name =
      abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
  // Generate the default task config path from PluginManager.
  default_config_path_ =
      apollo::cyber::plugin_manager::PluginManager::Instance()
          ->GetPluginClassHomePath<Task>(class_name) +
      "/conf/" + "default_conf.pb.txt";
  return true;
}
```

然后可以调用模板函数`Task::LoadConfig<T>(&config_)`加载任务参数。比如生成借道路径的任务 LaneBorrowPath。首先调用`Task::Init`获取任务参数路径，然后调用`Task::LoadConfig<LaneBorrowPathConfig>()`，加载为借道任务的参数。

```bash
bool LaneBorrowPath::Init(const std::string& config_dir,
                          const std::string& name,
                          const std::shared_ptr<DependencyInjector>& injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  // Load the config this task.
  return Task::LoadConfig<LaneBorrowPathConfig>(&config_);
}
```

任务的默认参数保存在任务conf目录下的`default_conf.pb.txt`，比如借道路径任务的参数在`modules/planning/tasks/lane_borrow_path/conf/default_conf.pb.txt`。
任务的配置参数定义为 proto 格式，保存在任务的 proto 目录下，比如借道路径任务的 proto 在`modules/planning/tasks/lane_borrow_path/proto`。

### 任务的运行

任务的运行函数主要是继承基类的 Process 函数或 Execute 函数。运行函数的输入通常是 frame 和 reference_line_info，输出也会保存到 frame 和 reference_line_info 中。

比如借道路径的任务，将生成的路径保存到了`reference_line_info->mutable_path_data()`

```bash
apollo::common::Status LaneBorrowPath::Process(
    Frame* frame, ReferenceLineInfo* reference_line_info) {
  if (!config_.is_allow_lane_borrowing() ||
      reference_line_info->path_reusable()) {
    ADEBUG << "path reusable" << reference_line_info->path_reusable()
           << ",skip";
    return Status::OK();
  }
  if (!IsNecessaryToBorrowLane()) {
    ADEBUG << "No need to borrow lane";
    return Status::OK();
  }
  std::vector<PathBoundary> candidate_path_boundaries;
  std::vector<PathData> candidate_path_data;

  GetStartPointSLState();
  if (!DecidePathBounds(&candidate_path_boundaries)) {
    return Status::OK();
  }
  if (!OptimizePath(candidate_path_boundaries, &candidate_path_data)) {
    return Status::OK();
  }
  if (AssessPath(&candidate_path_data,
                 reference_line_info->mutable_path_data())) {
    ADEBUG << "lane borrow path success";
  }

  return Status::OK();
}
```

## 交通规则插件的二次开发

交通规则插件 TrafficRule 主要是在规划模块执行 Scenario 前对交通规则进行处理，当您需要增加新的对于全场景生效的决策逻辑时，您可以开发新的交通规则插件
traffic rule 插件继承自 traffic rule 基类，而后由 planning_base 中的 traffic_decider 对各个插件进行生成并调用。planning 每进行一次规划任务，会通过 traffic_decider 调用各个 traffic rule，从而使 traffic rule 插件生效。

```bash
Status TrafficDecider::Execute(Frame *frame,
                               ReferenceLineInfo *reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  for (const auto &rule : rule_list_) {
    if (!rule) {
      AERROR << "Could not find rule ";
      continue;
    }
    rule->Reset();
    rule->ApplyRule(frame, reference_line_info);
    ADEBUG << "Applied rule " << rule->Getname();
  }

  BuildPlanningTarget(reference_line_info);
  return Status::OK();
}
```

### 交通规则插件的初始化

交通规则插件初始化主要是加配置插件的配置参数，调用`TrafficRule::Init()`函数可以获取插件配置参数文件路径，通过`TrafficRule::LoadConfig()`的模板函数可以从插件配置参数文件保存到`config_`中，比如人行道插件将`modules/planning/traffic_rules/crosswalk/conf/default_conf.pb.txt`里的配置参数加载进来。

```bash
bool Crosswalk::Init(const std::string& name,
                     const std::shared_ptr<DependencyInjector>& injector) {
  if (!TrafficRule::Init(name, injector)) {
    return false;
  }
  // Load the config this task.
  return TrafficRule::LoadConfig<CrosswalkConfig>(&config_);
}
```

### 交通规则插件的运行

在每个周期规划模块运行场景之前，会先运行每个交通规则插件的`ApplyRule()`函数，因此我们需要实现`ApplyRule`函数，并将所有的决策或约束保存在`reference_line_info`中，比如人行道插件会检查人行道周围障碍物状态，并在`reference_line_info`施加在人行道前的停止决策。

```bash
Status Crosswalk::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!FindCrosswalks(reference_line_info)) {
    injector_->planning_context()->mutable_planning_status()->clear_crosswalk();
    return Status::OK();
  }

  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}
```

## NEXT

[限速区域仿真调试 - 新增traffic rule插件](./限速区域仿真调试 - 新增traffic rule插件.md)
