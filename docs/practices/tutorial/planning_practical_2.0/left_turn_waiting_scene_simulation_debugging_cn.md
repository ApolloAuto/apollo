# 左转待转场景仿真调试

## 云实验

该实践内容已经上线 Apollo 云实验室，开快速启动进行实践体验：[Apollo规划之交通灯场景仿真调试](https://apollo.baidu.com/community/course/25)。

## Planning 运行机制回顾

Apollo 采用的是双状态机的框架，场景（Scenario）是规划模块的状态机，由场景管理器来切换调度，阶段（Stage）是场景的状态机，由场景来切换调度，任务（Task）是具体执行的规划运算，由阶段来编排调度。任务可以从任务库中复用我们现有的任务，也可以开发新的任务加入到任务库中，供各个场景阶段复用。

![image (15).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2815%29_3eb0d1e.png)

### 仿真场景需求

<!--
## 场景的参数配置

### 场景插件的使能
场景插件配置文件用来定义您所需要启用的 planning 场景插件，在`modules/planning/planning_base/conf/planning_config.pb.txt`的`planner_public_road_config`中配置，配置的场景插件会在规划模块运行时动态加载到规划模块进程上。如下例子表示仅启动泊车和沿车道行驶场景：

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

其中 name 是场景的名字，通常是大写加下划线方式表示，type 是场景的类的名字，均为字符串类型。查看 apollo 包含的所有场景插件可以在`modules/planning/scenarios`目录中查看。


### 场景流水线配置
每个场景（scenario）都会有个场景流水线配置文件，场景流水线配置了该场景所加载的阶段（stage）插件和每个阶段所需要加载的任务（task）插件。

场景流水线配置文件在场景的`conf`目录的`pipeline.pb.txt`，比如无保护左转场景的流水线配置文件在`modules/planning/scenarios/traffic_light_unprotected_left_turn/conf/pipeline.pb.txt`：

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
  ...
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

如上所示为无保护左转场景配置了 3 个阶段分别为接近路口、缓行观察路口，以及穿过交叉口。每个阶段配置了在这个阶段下所需要执行的任务，可以通过配置任务流水线增减场景的功能，比如不希望在接近路口过程中借道绕行，则可以删除 LANE_BORROW_PATH 任务的配置。

### 场景参数配置
场景参数配置文件在场景目录下的`scenario_conf.pb.txt`，比如无保护左转场景的配置文件为`modules/planning/scenarios/traffic_light_unprotected_left_turn/conf/scenario_conf.pb.txt`。

场景中的配置参数作用范围为场景和场景所包含的阶段使用，比如无保护左转场景参数：

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

场景中可以复用任务库中的所有任务，如果该场景下任务参数与默认的任务参数有冲突，则可以单独设置针对于该场景的任务参数。场景任务参数保存在场景的conf配置目录的阶段子目录下。如果完全复用默认的任务参数，则可以不添加场景任务配置文件。比如紧急靠边停车场景 emergency_pull_over，在靠边停车阶段 EMERGENCY_PULL_OVER_APPROACH，需要用到 pull_over_path 规划靠边停车的路径，但是其参数和默认靠边停车任务参数不同，需要添加 pull_over_path 的场景任务配置参数 pull_over_path.pb.txt，将其放到场景阶段的配置目录下：

```bash
modules/planning/scenarios/emergency_pull_over/conf/emergency_pull_over_approach/pull_over_path.pb.txt
```
#### 任务配置参数

场景任务配置参数只会在对应场景内生效，任务默认参数的修改可以对所有场景生效。每一个任务的配置参数保存在任务`conf`目录的`default_conf.pb.txt`。比如规划靠边停车任务的默认配置参数在：
`modules/planning/tasks/pull_over_path/conf/default_conf.pb.txt`。

## 场景的二次开发
- Scenario 可以根据地理位置来划分，当场景中主车在特定区域规划动作和默认场景（Lane Follow）存在有所不同是，为了避免影响默认场景的运行，可以为其开发一个新的场景，比如前方有红绿灯需要开发红绿灯场景，前方停止牌需要开发停止牌场景；

- Scenario 也可根据命令来划分，当接收到紧急靠边停车命令时，进入紧急靠边停车场景。比如接收到泊车命令时，进入泊车场景。

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
场景的初始化需要继承 Scenario 的`Init()`函数，场景基类的 Init 函数主要是从场景插件中加载场景的流水线，将加载的 Stage 实例保存到`stage_pipeline_map_`中。如果场景自身还有配置文件，则可以调用`Scenario::LoadConfig<T>()`函数加载场景自身的配置文件，保存到场景实例上下文变量中`context_`。

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

场景切换函数主要是从场景管理器中调用，判断是否需要切入该场景。场景切换函数继承与 Scenario 基类的 IsTransferable()，判断当前帧是否可以切入该场景，如果返回 true，则会切入该场景，不再进行后续场景的判断。场景切入包括基于地理位置触发的方式，比如停止标记场景：从参考线获得首个 overlap 是否是停止标记，如果是且主车距离停止标记满足设定的距离，则进入停止标记场景。

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
还有一种场景切入方式是基于命令触发的，比如说紧急靠边停车场景，其切入条件就是收到`pad_msg`的命令即进入紧急靠边停车场景：


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
场景的进入函数继承于基类的`Enter()`函数，在首次进入场景前调用做一些预处理的工作，重置场景内变量。
比如在停止标记场景的`Enter()`函数，首先寻找参考线的停止标记 id 保存到上下文变量中，然后重置停止标记的全局变量。


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
场景的退出函数继承于基类的`Exit()`函数，在场景切出时会被调用，可以用来清除一些全局变量，比如停止标记场景的切出函数。


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

场景运行函数继承于基类的`Process()`函数，在每一帧场景运行时都会被调用，基类的 Process 主要用来创建 Stage、运行 Stage 的 Process 函数以及调度不同 Stage 的切换。

在 apollo 中的场景一般都默认采用基类的`Process()`函数。如果您有更多定制需求可以继承 Process 函数重写业务策略。

### 场景上下文变量

场景上下文变量为场景的成员变量，在多帧间和 scenario 与 stage 间传递数据，比如停止牌场景的上下文变量：


```bash
// stage context
struct StopSignUnprotectedContext : public ScenarioContext {
  ScenarioStopSignUnprotectedConfig scenario_config;
  std::string current_stop_sign_overlap_id;
  double stop_start_time = 0.0;
  double creep_start_time = 0.0;
  // watch_vehicle: <lane_id, perception_obstacle_ids>
  std::unordered_map<std::string, std::vector<std::string>> watch_vehicles;
  std::vector<std::pair<hdmap::LaneInfoConstPtr, hdmap::OverlapInfoConstPtr>>
      associated_lanes;
};
```

## 阶段的二次开发

Apollo 中的 Stage(阶段) 是 Scenario 下的第二层状态机，可以根据时间来划分。当场景中存在先后顺序的业务逻辑时，可以将其划分成多个 Stage。比如在红绿灯无保护左转场景中可以划分为三个阶段：
- 第一个阶段是接近行驶到红绿灯停止线前的过程，
- 第二个是红绿灯为绿灯时慢速观望的过程，
- 第三个是对向直行车道通畅快速通过的过程。


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
阶段划分好后，需要对阶段配置任务（Task）。在每一帧的规划中只会运行一个阶段，在一个阶段内会顺序执行阶段内的每一个任务。任务是处理路径规划或速度规划的最小计算单元。您可以复用 Apollo 中已有的任务进行配置，也可以根据开发新的规划任务。

### 阶段初始化
在场景运行时会对创建场景包含的阶段实例，并对阶段初始化。如果需要对阶段初始化，您可以继承重写阶段的`Init()`函数，基类`Init()`函数主要用于加载任务的流水线，并对任务初始化。Apollo 中一般采用的是基类的 Init 函数。


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

### 阶段运行函数

阶段的运行函数`Process()`主要用于维护当前阶段的状态。stage 包括三种状态 RUNNING、FINISHED 和 ERROR 三种状态。其中：
- RUNNING 表示当前状态正在运行，Scenario 将继续维持当前阶段运行；
- FINISHED 表示当前阶段完成，Scenario 将会切入下一个阶段运行；
- ERROR 表示当前规划存在严重故障。Scenario 会将其上报，主车将会刹停。

如果当前阶段完成，除了要返回当前阶段为 FINISHED，还需要指定下一个阶段的名称`next_stage_`，Scenario 将会根据当前阶段的 next\_stage 切入下一个阶段。如果指定的 `next_stage_` 为空字符串，则 Scenario 会认为全部 Stage 已经运行完成，Scenario 也会返回完成的状态。

在 Stage 中依次调用 task，有两个基类的函数可以复用，一个是`ExecuteTaskOnReferenceLine`，主要是用于主路上一系列任务的运行，一个是`ExecuteTaskOnOpenSpace`，主要是用于开放空间一系列任务的运行。


```bash
  StageResult ExecuteTaskOnReferenceLine(
      const common::TrajectoryPoint& planning_start_point, Frame* frame);

  StageResult ExecuteTaskOnOpenSpace(Frame* frame);
```

场景中的上下文数据会通过 void\* 指针的方式保存在 Stage 的`context_`中，如果需要读取场景的上下文数据，可以通过模板函数`GetContextAs()`对上下文变量进行解析。

```bash
  template <typename T>
  T* GetContextAs() const {
    return static_cast<T*>(context_);
  }

```

## 任务的二次开发

当 Apollo 中的任务 (Task) 无法满足您场景需求时，您需要开发全新的任务插件。Apollo 中存在多种类型的 Task 基类：

- PathGeneration：主要用于在主路上生成路径，比如规划借道路径 LaneBorrowPath、靠边停车路径 PullOverPath，沿车道行驶路径 LaneFollowPath 等，
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

任务的默认参数保存在任务`conf`目录下的`default_conf.pb.txt`，比如借道路径任务的参数在`modules/planning/tasks/lane_borrow_path/conf/default_conf.pb.txt`。任务的配置参数定义为 proto 格式，保存在任务的`proto`目录下，比如借道路径任务的`proto在modules/planning/tasks/lane_borrow_path/proto`。


### 任务的运行


任务的运行函数主要是继承基类的 Process 函数或 Execute 函数。运行函数的输入通常是 `frame` 和`reference_line_info`，输出也会保存到 `frame` 和`reference_line_info`中。比如借道路径的任务，将生成的路径保存到了 `reference_line_info->mutable_path_data()`。

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

-->

这里以`apollo_virtual_map`为地图，开发一个左转待转场景。左转待转场景可以分为两个区域，直行区和左转待转区。

- 主车在直行区：

  如果左转红灯、直行绿灯，则车辆可以进入左转待转区：

  ![image (34).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2834%29_a02ad93.png)

  如果左转绿灯，无论直行灯是红灯还是绿灯都可以通过路口：

  ![image (35).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2835%29_7839cce.png)

  如果左转红灯、直行红灯，则车辆在直行区停止线等待：

  ![image (36).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2836%29_0cdfea7.png)

- 主车在左转区：

  如果左转为红灯，直行为绿灯，则车辆在左转区停止线前等待：

  ![image (37).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2837%29_1668784.png)

  如果左转为绿灯，则车辆可以通行路口：

  ![image (38).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2838%29_41ff878.png)

  > 说明：在待转区如果左转灯由绿变成红，则主车可以不需要在待转区等待，通行路口。
  > 额外要求：每个区域巡航速度不同，在直行区巡航速度为 5m/s，左转区巡航速度 3m/s，通过路口巡航速度 8m/s。

  ![image (39).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2839%29_ab30a31.png)

  在`base_map.txt`的地图文件可以观察下左转待转区的元素：

  左转待转场景，左转灯在左转车道上存在两条停止线，分别在直行区和左转区，左转灯的 subsignal 的类型为`ARROW_LEFT`；而直行灯在左转的车道上只有一条停止线，位于直行区，且直行灯的 subsignal 为 CIRCLE 类型。可以依此来判断是否进入左转待转场景。

- 左转灯元素：

  ```bash
  signal {
  id {
    id: "451089192"
  }
  boundary {
    point {
      x: 750969.52326592465
      y: 2563961.3250804096
      z: 26.59372596
    }
    point {
      x: 750968.32370583259
      y: 2563961.3780084951
      z: 26.59372596
    }
    point {
      x: 750968.32370583259
      y: 2563961.3780084951
      z: 26.99372597
    }
    point {
      x: 750969.52326592465
      y: 2563961.3250804096
      z: 26.99372597
    }
  }
  subsignal {
    id {
      id: "0"
    }
    type: ARROW_LEFT
    location {
      x: 750969.3284603036
      y: 2563961.3339879983
      z: 26.79372597
    }
  }
  subsignal {
    id {
      id: "1"
    }
    type: ARROW_LEFT
    location {
      x: 750968.92860693939
      y: 2563961.3516306877
      z: 26.79372597
    }
  }
  subsignal {
    id {
      id: "2"
    }
    type: ARROW_LEFT
    location {
      x: 750968.5287535761
      y: 2563961.3692733883
      z: 26.79372597
    }
  }
  overlap_id {
    id: "overlap_263"
  }
  overlap_id {
    id: "overlap_306"
  }
  overlap_id {
    id: "overlap_319"
  }
  overlap_id {
    id: "overlap_324"
  }
  overlap_id {
    id: "overlap_908"
  }
  type: MIX_3_HORIZONTAL
  stop_line {
    segment {
      line_segment {
        point {
          x: 750970.81973172375
          y: 2564012.0906806863
        }
        point {
          x: 750968.8422244587
          y: 2564012.1039131097
        }
        point {
          x: 750966.8749406623
          y: 2564012.1184260151
        }
      }
    }
  }
  stop_line {
    segment {
      line_segment {
        point {
          x: 750972.57867658127
          y: 2563990.9899958805
        }
        point {
          x: 750970.86382346111
          y: 2563990.6154060815
        }
        point {
          x: 750969.1592123817
          y: 2563990.2409889824
        }
      }
    }
  }
  }
  ```

- 右转灯元素：

  ```bash
  signal {
  id {
    id: "451089193"
  }
  boundary {
    point {
      x: 750963.29795619729
      y: 2563961.7155308384
      z: 26.72153167
    }
    point {
      x: 750962.09768731019
      y: 2563961.8105529686
      z: 26.72153167
    }
    point {
      x: 750962.09768731019
      y: 2563961.8105529686
      z: 27.12153168
    }
    point {
      x: 750963.29795619729
      y: 2563961.7155308384
      z: 27.12153168
    }
  }
  subsignal {
    id {
      id: "0"
    }
    type: CIRCLE
    location {
      x: 750963.09279653709
      y: 2563961.7309123669
      z: 26.92153168
    }
  }
  subsignal {
    id {
      id: "1"
    }
    type: CIRCLE
    location {
      x: 750962.69270068756
      y: 2563961.762955647
      z: 26.92153168
    }
  }
  subsignal {
    id {
      id: "2"
    }
    type: CIRCLE
    location {
      x: 750962.29260484129
      y: 2563961.7949989373
      z: 26.92153168
    }
  }
  overlap_id {
    id: "overlap_253"
  }
  overlap_id {
    id: "overlap_257"
  }
  overlap_id {
    id: "overlap_264"
  }
  overlap_id {
    id: "overlap_909"
  }
  type: MIX_3_HORIZONTAL
  stop_line {
    segment {
      line_segment {
        point {
          x: 750970.81973172375
          y: 2564012.0906806863
        }
        point {
          x: 750968.8422244587
          y: 2564012.1039131097
        }
        point {
          x: 750966.8749406623
          y: 2564012.1184260151
        }
      }
    }
  }
  stop_line {
    segment {
      line_segment {
        point {
          x: 750966.87382129545
          y: 2564012.1848895182
        }
        point {
          x: 750964.86549455044
          y: 2564012.2031436064
        }
        point {
          x: 750962.85718649556
          y: 2564012.2202902385
        }
      }
    }
  }
  }
  ```

阶段的划分：

我们可以根据行驶过程分为：

- 在直行区停车等待的阶段TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_APPROACH，
- 在待转区行停车等待的阶段TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CREEP，
- 通过路口的阶段TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CRUISE。

### 实践步骤

1. 在包管理环境下创建插件模板，通过以下命令可以创建一个左转待转插件的模板。

   ```bash
    buildtool create --template plugin --namespaces planning --dependencies planning:binary:planning --base_class_name apollo::planning::Scenario --class_name TrafficLightLeftTurnWaitingZone     modules/planning/scenarios/traffic_light_left_turn_waiting_zone
   ```

2. 在包管理模版基础上实现左转待转的 Scneario 子类。

   - a. 在`IsTransferable()`函数中添加场景切入条件：当发现当前参考线上有两个 id 一样的信号灯 overlap，而且是左转信号灯，且其中一个 overlap 在最近的信号灯组内，则切入该场景。
   - b. 在`Enter()`函数设置进入场景前的参数初始化，将直行信号灯，左转信号灯 id 保存到`context_`变量中。
   - c. 在`Exit()`函数设置场景切出时对成员变量和全局变量的重置。

   文件目录：

   ```bash
   planning/scenarios/traffic_light_left_turn_waiting_zone/traffic_light_left_turn_waiting_zone.cc
   ```

   ```bash
   #include <memory>
    #include "modules/planning/scenarios/traffic_light_left_turn_waiting_zone/traffic_light_left_turn_waiting_zone.h"

    namespace apollo {
    namespace planning {

    using apollo::hdmap::HDMapUtil;

    bool TrafficLightLeftTurnWaitingZone::Init(std::shared_ptr<DependencyInjector> injector, const std::string& name) {
        if (!Scenario::Init(injector, name)) {
            AERROR << "failed to init scenario" << Name();
            return false;
        }

        if (!Scenario::LoadConfig<TrafficLightLeftTurnWaitingZoneConfig>(&context_.scenario_config)) {
            AERROR << "fail to get specific config of scenario " << Name();
            return false;
        }
        return true;
    }

    bool TrafficLightLeftTurnWaitingZone::IsTransferable(const Scenario* const other_scenario, const Frame& frame) {
        if (other_scenario == nullptr || frame.reference_line_info().empty()) {
            return false;
        }
        const auto& reference_line_info = frame.reference_line_info().front();
        const auto& first_encountered_overlaps = reference_line_info.FirstEncounteredOverlaps();
        const std::vector<hdmap::PathOverlap> traffic_light_overlaps
                = reference_line_info.reference_line().map_path().signal_overlaps();
        if (first_encountered_overlaps.empty()) {
            return false;
        }
        hdmap::PathOverlap* traffic_sign_overlap = nullptr;
        for (const auto& overlap : first_encountered_overlaps) {
            if (overlap.first == ReferenceLineInfo::STOP_SIGN || overlap.first == ReferenceLineInfo::YIELD_SIGN) {
                return false;
            } else if (overlap.first == ReferenceLineInfo::SIGNAL) {
                traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
                break;
            }
        }
        if (traffic_sign_overlap == nullptr) {
            return false;
        }
        const double start_check_distance = context_.scenario_config.start_traffic_light_scenario_distance();
        const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
        if (traffic_sign_overlap->start_s - adc_front_edge_s > start_check_distance) {
            return false;
        }
        const auto& turn_type = reference_line_info.GetPathTurnType(traffic_sign_overlap->start_s);
        if (turn_type != hdmap::Lane::LEFT_TURN) {
            return false;
        }
        auto* base_map = HDMapUtil::BaseMapPtr();

        static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
        for (const auto& overlap : traffic_light_overlaps) {
            const double dist = overlap.start_s - traffic_sign_overlap->start_s;
            if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
                const auto& signal = base_map->GetSignalById(hdmap::MakeMapId(overlap.object_id))->signal();
                if (signal.subsignal_size() > 0 && signal.subsignal(0).type() == apollo::hdmap::Subsignal::ARROW_LEFT
                    && signal.stop_line_size() > 1) {
                    return true;
                }
            }
        }

        return false;
    }

    bool TrafficLightLeftTurnWaitingZone::Enter(Frame* frame) {
        const auto& reference_line_info = frame->reference_line_info().front();
        const auto& first_encountered_overlaps = reference_line_info.FirstEncounteredOverlaps();
        const std::vector<hdmap::PathOverlap> traffic_light_overlaps
                = reference_line_info.reference_line().map_path().signal_overlaps();
        hdmap::PathOverlap* nearest_traffic_light_overlap = nullptr;
        for (const auto& overlap : first_encountered_overlaps) {
            if (overlap.first == ReferenceLineInfo::SIGNAL) {
                nearest_traffic_light_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
                break;
            }
        }
        std::vector<hdmap::PathOverlap> next_traffic_lights;
        auto* base_map = HDMapUtil::BaseMapPtr();
        std::string left_turn_signal_id;
        static constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m
        for (const auto& overlap : traffic_light_overlaps) {
            const double dist = overlap.start_s - nearest_traffic_light_overlap->start_s;
            if (fabs(dist) <= kTrafficLightGroupingMaxDist) {
                const auto signal = base_map->GetSignalById(hdmap::MakeMapId(overlap.object_id));
                AINFO << signal->id().id() << "," << overlap.object_id;
                if (signal->signal().subsignal(0).type() == apollo::hdmap::Subsignal::ARROW_LEFT) {
                    context_.left_turn_traffic_light_id = overlap.object_id;
                    AINFO << "left_turn_signal_id" << context_.left_turn_traffic_light_id;
                } else {
                    context_.forward_traffic_light_id = overlap.object_id;
                    AINFO << "forward signal id" << context_.forward_traffic_light_id;
                }
            }
        }

        return true;
    }

    bool TrafficLightLeftTurnWaitingZone::Exit(Frame* frame) {
        context_.left_turn_traffic_light_id.clear();
        context_.forward_traffic_light_id.clear();
        auto done_traffic_light = injector_->planning_context()->mutable_planning_status()->mutable_traffic_light();
        done_traffic_light->mutable_done_traffic_light_overlap_id()->Clear();
        return true;
    }

    }  // namespace planning
    }  // namespace apollo
   ```

   添加头文件`modules/planning/scenarios/traffic_light_left_turn_waiting_zone/traffic_light_left_turn_waiting_zone.h`：

   ```bash
   #pragma once

    #include <memory>
    #include "cyber/plugin_manager/plugin_manager.h"
    #include "modules/planning/planning_base/scenario_base/scenario.h"
    #include "modules/planning/scenarios/traffic_light_left_turn_waiting_zone/proto/traffic_light_left_turn_waiting_zone.pb.h"

    namespace apollo {
    namespace planning {

    struct TrafficLightLeftTurnWaitingZoneContext : public ScenarioContext {
        TrafficLightLeftTurnWaitingZoneConfig scenario_config;
        std::string left_turn_traffic_light_id;
        std::string forward_traffic_light_id;
    };

    class TrafficLightLeftTurnWaitingZone : public apollo::planning::Scenario {
    public:
        bool Init(std::shared_ptr<DependencyInjector> injector, const std::string& name);
        TrafficLightLeftTurnWaitingZoneContext* GetContext() override {
            return &context_;
        }
        bool IsTransferable(const Scenario* other_scenario, const Frame& frame);
        bool Exit(Frame* frame);
        bool Enter(Frame* frame);

    private:
        TrafficLightLeftTurnWaitingZoneContext context_;
    };

    CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::TrafficLightLeftTurnWaitingZone, apollo::planning::Scenario)

    }  // namespace planning
    }  // namespace apollo
   ```

3. 开发左转待转的第一个阶段，行驶到直行区停止线的阶段 TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_APPROACH，实现`Stage的Process()`函数：

   - a. 在`reference_line_info`中设置该阶段的限速，
   - b. 如果左转灯是绿灯，且通过了停止线则切到 TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CRUISE 阶段，同时将直行灯加入到`add_done_traffic_light_overlap_id`的全局变量中，
   - c. 如果左转灯是红灯，直行灯是绿灯，则切到 TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CREEP 阶段，同时将左转灯加入到`add_done_traffic_light_overlap_id`的全局变量中，
   - d. 如果已经通过了停止线，则切入 TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CRUISE 阶段。

   文件路径：

   ```bash
   planning/scenarios/traffic_light_left_turn_waiting_zone/stage_approach.cc
   ```

   ```bash
   #include "modules/planning/scenarios/traffic_light_left_turn_waiting_zone/stage_approach.h"
    #include "modules/planning/scenarios/traffic_light_left_turn_waiting_zone/traffic_light_left_turn_waiting_zone.h"
    #include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
    #include "cyber/common/log.h"
    #include "modules/planning/planning_base/common/frame.h"
    #include "modules/planning/planning_base/common/planning_context.h"

    namespace apollo {
    namespace planning {

    using apollo::common::TrajectoryPoint;
    using apollo::hdmap::PathOverlap;
    using apollo::perception::TrafficLight;

    StageResult TrafficLightLeftTurnWaitingZoneStageApproach::Process(
            const TrajectoryPoint& planning_init_point,
            Frame* frame) {
        ADEBUG << "stage: Approach";
        CHECK_NOTNULL(frame);
        CHECK_NOTNULL(context_);

        auto* context = GetContextAs<TrafficLightLeftTurnWaitingZoneContext>();
        const auto& scenario_config = context->scenario_config;
        auto& reference_line_info = frame->mutable_reference_line_info()->front();
        reference_line_info.LimitCruiseSpeed(scenario_config.approach_speed());
        StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
        if (result.HasError()) {
            AERROR << "TrafficLightLeftTurnWaitingZoneStageApproach planning error";
        }
        if (context->left_turn_traffic_light_id.empty() || context->forward_traffic_light_id.empty()) {
            return FinishScenario();
        }
        hdmap::PathOverlap forward_traffic_light_overlap;
        hdmap::PathOverlap left_turn_traffic_light_overlap_first;
        hdmap::PathOverlap left_turn_traffic_light_overlap_second;
        const std::vector<hdmap::PathOverlap> traffic_light_overlaps
                = reference_line_info.reference_line().map_path().signal_overlaps();
        for (const auto& overlap : traffic_light_overlaps) {
            if (overlap.object_id == context->forward_traffic_light_id) {
                forward_traffic_light_overlap = overlap;
            } else if (overlap.object_id == context->left_turn_traffic_light_id) {
                if (left_turn_traffic_light_overlap_second.start_s < overlap.start_s) {
                    std ::swap(left_turn_traffic_light_overlap_first, left_turn_traffic_light_overlap_second);
                    left_turn_traffic_light_overlap_second = overlap;
                } else {
                    left_turn_traffic_light_overlap_first = overlap;
                }
            }
        }
        const double adc_back_edge_s = reference_line_info.AdcSlBoundary().end_s();
        bool is_passed_forward_stop_line = adc_back_edge_s > forward_traffic_light_overlap.end_s;
        auto forward_signal_color = frame->GetSignal(context->forward_traffic_light_id).color();
        auto left_signal_color = frame->GetSignal(context->left_turn_traffic_light_id).color();
        auto done_traffic_light = injector_->planning_context()->mutable_planning_status()->mutable_traffic_light();
        done_traffic_light->mutable_done_traffic_light_overlap_id()->Clear();
        if (left_signal_color == TrafficLight::GREEN) {
            if (is_passed_forward_stop_line) {
                return FinishStage("TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CRUISE");
            }
            done_traffic_light->add_done_traffic_light_overlap_id(forward_traffic_light_overlap.object_id);
        } else if (forward_signal_color == TrafficLight::GREEN) {
            done_traffic_light->add_done_traffic_light_overlap_id(left_turn_traffic_light_overlap_first.object_id);
        }
        if (is_passed_forward_stop_line) {
            return FinishStage("TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CREEP");
        }

        return result.SetStageStatus(StageStatusType::RUNNING);
    }

    StageResult TrafficLightLeftTurnWaitingZoneStageApproach::FinishStage(std::string next_stage) {
        injector_->planning_context()
                ->mutable_planning_status()
                ->mutable_traffic_light()
                ->mutable_done_traffic_light_overlap_id()
                ->Clear();
        next_stage_ = next_stage;
        return StageResult(StageStatusType::FINISHED);
    }

    }  // namespace planning
    }  // namespace apollo
   ```

   添加头文件`modules/planning/scenarios/traffic_light_left_turn_waiting_zone/stage_approach.h`：

   ```bash
    #pragma once

    #include "cyber/plugin_manager/plugin_manager.h"
    #include "modules/planning/planning_base/scenario_base/stage.h"

    namespace apollo {
    namespace planning {

    class TrafficLightLeftTurnWaitingZoneStageApproach : public Stage {
    public:
        StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;

    private:
        StageResult FinishStage(std::string next_stage);
    };

    CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(
            apollo::planning::TrafficLightLeftTurnWaitingZoneStageApproach,
            apollo::planning::Stage)

    }  // namespace planning
    }  // namespace apollo
   ```

4. 开发第二个阶段，待转区行驶的阶段 TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CREEP，实现待转区阶段的`Process()`函数：

   - a. 在`reference_line_info`中设置该阶段的限速
   - b. 如果通过了左转停止线，或者左转灯是绿灯则切入 TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CRUISE 阶段。

   文件路径：

   ```bash
   planning/scenarios/traffic_light_left_turn_waiting_zone/stage_creep.cc
   ```

   ```bash
   #include "modules/planning/scenarios/traffic_light_left_turn_waiting_zone/stage_creep.h"
    #include "modules/planning/scenarios/traffic_light_left_turn_waiting_zone/traffic_light_left_turn_waiting_zone.h"
    #include <string>

    #include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"
    #include "cyber/common/log.h"
    #include "modules/planning/planning_base/common/frame.h"
    #include "modules/planning/planning_base/common/planning_context.h"

    namespace apollo {
    namespace planning {

    using apollo::common::TrajectoryPoint;
    using apollo::hdmap::PathOverlap;
    using apollo::perception::TrafficLight;

    bool TrafficLightLeftTurnWaitingZoneStageCreep::Init(
            const StagePipeline& config,
            const std::shared_ptr<DependencyInjector>& injector,
            const std::string& config_dir,
            void* context) {
        CHECK_NOTNULL(context);
        bool ret = Stage::Init(config, injector, config_dir, context);
        if (!ret) {
            AERROR << Name() << "init failed!";
            return false;
        }
        return ret;
    }

    StageResult TrafficLightLeftTurnWaitingZoneStageCreep::Process(
            const TrajectoryPoint& planning_init_point,
            Frame* frame) {
        CHECK_NOTNULL(frame);
        CHECK_NOTNULL(context_);

        auto* context = GetContextAs<TrafficLightLeftTurnWaitingZoneContext>();
        const auto& scenario_config = context->scenario_config;
        auto& reference_line_info = frame->mutable_reference_line_info()->front();
        reference_line_info.LimitCruiseSpeed(scenario_config.creep_speed());
        StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
        if (result.HasError()) {
            AERROR << "TrafficLightLeftTurnWaitingZoneStageCreep planning error";
        }
        hdmap::PathOverlap forward_traffic_light_overlap;
        hdmap::PathOverlap left_turn_traffic_light_overlap_first;
        hdmap::PathOverlap left_turn_traffic_light_overlap_second;
        const std::vector<hdmap::PathOverlap> traffic_light_overlaps
                = reference_line_info.reference_line().map_path().signal_overlaps();
        for (const auto& overlap : traffic_light_overlaps) {
            if (overlap.object_id == context->forward_traffic_light_id) {
                forward_traffic_light_overlap = overlap;
            } else if (overlap.object_id == context->left_turn_traffic_light_id) {
                if (left_turn_traffic_light_overlap_second.start_s < overlap.start_s) {
                    std ::swap(left_turn_traffic_light_overlap_first, left_turn_traffic_light_overlap_second);
                    left_turn_traffic_light_overlap_second = overlap;
                } else {
                    left_turn_traffic_light_overlap_first = overlap;
                }
            }
        }
        const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
        bool is_passed_left_turn_stop_line = adc_front_edge_s > left_turn_traffic_light_overlap_second.end_s;
        auto left_signal_color = frame->GetSignal(context->left_turn_traffic_light_id).color();
        auto done_traffic_light = injector_->planning_context()->mutable_planning_status()->mutable_traffic_light();
        done_traffic_light->mutable_done_traffic_light_overlap_id()->Clear();
        if (is_passed_left_turn_stop_line || left_signal_color == TrafficLight::GREEN) {
            reference_line_info.LimitCruiseSpeed(scenario_config.cruise_speed());
            return FinishStage();
        }

        return result.SetStageStatus(StageStatusType::RUNNING);
    }

    StageResult TrafficLightLeftTurnWaitingZoneStageCreep::FinishStage() {
        next_stage_ = "TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CRUISE";
        return StageResult(StageStatusType::FINISHED);
    }

    }  // namespace planning
    }  // namespace apollo
   ```

   添加头文件`modules/planning/scenarios/traffic_light_left_turn_waiting_zone/stage_creep.h`：

   ```bash
   #pragma once

    #include <memory>
    #include <string>

    #include "cyber/plugin_manager/plugin_manager.h"
    #include "modules/planning/planning_base/scenario_base/stage.h"

    namespace apollo {
    namespace planning {

    class TrafficLightLeftTurnWaitingZoneStageCreep : public Stage {
    public:
        bool Init(
                const StagePipeline& config,
                const std::shared_ptr<DependencyInjector>& injector,
                const std::string& config_dir,
                void* context) override;

        StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;

    private:
        StageResult FinishStage();
    };

    CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::TrafficLightLeftTurnWaitingZoneStageCreep, Stage)

    }  // namespace planning
    }  // namespace apollo
   ```

5. 开发第三个阶段通过交叉路口 TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CRUISE。

   - a. 该阶段可以继承当前已有的交叉路口阶段基类 BaseStageTrafficLightCruise，
   - b. 设置当前阶段限速，
   - c. 这一阶段将无视红绿灯信息，因此将直行灯和左转灯都加入到`add_done_traffic_light_overlap_id`。

   文件路径：

   ```bash
   planning/scenarios/traffic_light_left_turn_waiting_zone/stage_intersection_cruise.cc
   ```

   ```bash
       #include "modules/planning/scenarios/traffic_light_left_turn_waiting_zone/stage_intersection_cruise.h"
    #include "modules/planning/scenarios/traffic_light_left_turn_waiting_zone/traffic_light_left_turn_waiting_zone.h"
    #include "cyber/common/log.h"

    namespace apollo {
    namespace planning {

    StageResult TrafficLightLeftTurnWaitingZoneStageIntersectionCruise::Process(
            const common::TrajectoryPoint& planning_init_point,
            Frame* frame) {
        const auto* context = GetContextAs<TrafficLightLeftTurnWaitingZoneContext>();
        const auto& scenario_config = context->scenario_config;
        auto& reference_line_info = frame->mutable_reference_line_info()->front();
        reference_line_info.LimitCruiseSpeed(scenario_config.cruise_speed());
        StageResult result = ExecuteTaskOnReferenceLine(planning_init_point, frame);
        auto done_traffic_light = injector_->planning_context()->mutable_planning_status()->mutable_traffic_light();
        done_traffic_light->mutable_done_traffic_light_overlap_id()->Clear();
        done_traffic_light->add_done_traffic_light_overlap_id(context->forward_traffic_light_id);
        done_traffic_light->add_done_traffic_light_overlap_id(context->left_turn_traffic_light_id);
        if (result.HasError()) {
            AERROR << "TrafficLightLeftTurnWaitingZoneStageIntersectionCruise "
                   << "plan error";
        }

        bool stage_done = CheckDone(*frame, injector_->planning_context(), true);
        if (stage_done) {
            return FinishStage();
        }
        return result.SetStageStatus(StageStatusType::RUNNING);
    }

    StageResult TrafficLightLeftTurnWaitingZoneStageIntersectionCruise::FinishStage() {
        return FinishScenario();
    }

    }  // namespace planning
    }  // namespace apollo
   ```

   添加头文件`modules/planning/scenarios/traffic_light_left_turn_waiting_zone/stage_intersection_cruise.h`：

   ```bash
   #pragma once

    #include <string>

    #include "cyber/plugin_manager/plugin_manager.h"
    #include "modules/planning/planning_base/scenario_base/traffic_light_base/base_stage_traffic_light_cruise.h"

    namespace apollo {
    namespace planning {

    class TrafficLightLeftTurnWaitingZoneStageIntersectionCruise : public BaseStageTrafficLightCruise {
    public:
        StageResult Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) override;

    private:
        StageResult FinishStage();
    };

    CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::TrafficLightLeftTurnWaitingZoneStageIntersectionCruise, Stage)

    }  // namespace planning
    }  // namespace apollo
   ```

6. 完成场景的流水线配置`conf/pipeline.pb.txt`，配置需要启动的阶段，和每个阶段用到的任务。

   文件路径：

   ```bash
   planning/scenarios/traffic_light_left_turn_waiting_zone/conf/pipeline.pb.txt
   ```

   ```bash
   stage: {
      name: "TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_APPROACH"
      type: "TrafficLightLeftTurnWaitingZoneStageApproach"
      enabled: true
      task {
        name: "LANE_FOLLOW_PATH"
        type: "LaneFollowPath"
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
      name: "TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CREEP"
      type: "TrafficLightLeftTurnWaitingZoneStageCreep"
      enabled: true
      task {
        name: "LANE_FOLLOW_PATH"
        type: "LaneFollowPath"
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
      name: "TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE_CRUISE"
      type: "TrafficLightLeftTurnWaitingZoneStageIntersectionCruise"
      enabled: true
        task {
        name: "LANE_FOLLOW_PATH"
        type: "LaneFollowPath"
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

7. 添加场景的参数文件。

- 将场景用到的参数配置到一个 proto 文件：

  文件路径：

  ```bash
  planning/scenarios/traffic_light_left_turn_waiting_zone/proto/traffic_light_left_turn_waiting_zone.proto
  ```

  ```bash
  syntax = "proto2";

  package apollo.planning;

  message TrafficLightLeftTurnWaitingZoneConfig {
    optional double start_traffic_light_scenario_distance = 1;  // meter
    optional double approach_speed = 2;
    optional double creep_speed = 3;
    optional double cruise_speed = 4;
  }
  ```

* 修改配置的编译文件 BUILD，将其编译为一个。

  文件路径：

  ```bash
  proto_library(planning/scenarios/traffic_light_left_turn_waiting_zone/proto/BUILD
  ```

  ```bash
  load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
  load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
  load("//tools/proto:proto.bzl", "proto_library")
  load("//tools:cpplint.bzl", "cpplint")

  package(default_visibility = ["//visibility:public"])

  proto_library(
      name = "traffic_light_left_turn_waiting_zone_proto",
      srcs = ["traffic_light_left_turn_waiting_zone.proto"],
  )

  apollo_package()

  cpplint()
  ```

* 将场景的参数配置添加到`planning/scenarios/traffic_light_left_turn_waiting_zone/conf/scenario_conf.pb.txt`中。

  ```bash
  start_traffic_light_scenario_distance: 30.0
  approach_speed:5.0
  creep_speed: 3.0
  cruise_speed:8.0
  ```

8. 将添加的 scenario 和 stage 插件注册到文件中`modules/planning/scenarios/traffic_light_left_turn_waiting_zone/plugin_traffic_light_left_turn_waiting_zone_description.xml`。
   ```bash
   <library path="modules/planning/scenarios/traffic_light_left_turn_waiting_zone/libtraffic_light_left_turn_waiting_zone.so">
    <class type="apollo::planning::TrafficLightLeftTurnWaitingZone" base_class="apollo::planning::Scenario"></class>
    <class type="apollo::planning::TrafficLightLeftTurnWaitingZoneStageApproach" base_class="apollo::planning::Stage"></class>
    <class type="apollo::planning::TrafficLightLeftTurnWaitingZoneStageCreep" base_class="apollo::planning::Stage"></class>
    <class type="apollo::planning::TrafficLightLeftTurnWaitingZoneStageIntersectionCruise" base_class="apollo::planning::Stage"></class>
   </library>
   ```
9. 将左转待转场景加入到 planning 的场景管理流水线中。

   文件路径：

   ```bash
    profiles/current/modules/planning/planning_base/conf/planning_config.pb.txt
   ```

   ```bash
   topic_config {
      chassis_topic: "/apollo/canbus/chassis"
      hmi_status_topic: "/apollo/hmi/status"
      localization_topic: "/apollo/localization/pose"
      planning_pad_topic: "/apollo/planning/pad"
      planning_trajectory_topic: "/apollo/planning"
      prediction_topic: "/apollo/prediction"
      relative_map_topic: "/apollo/relative_map"
      routing_request_topic: "/apollo/external_command/lane_follow"
      routing_response_topic: "/apollo/routing_response"
      planning_command_topic: "/apollo/planning/command"
      story_telling_topic: "/apollo/storytelling"
      traffic_light_detection_topic: "/apollo/perception/traffic_light"
      planning_learning_data_topic: "/apollo/planning/learning_data"
    }
    # NO_LEARNING / E2E / HYBRID / RL_TEST / E2E_TEST / HYBRID_TEST
    learning_mode: NO_LEARNING
    reference_line_config {
      pnc_map_class: "apollo::planning::LaneFollowMap"
    }
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
          name: "TRAFFIC_LIGHT_LEFT_TURN_WAITING_ZONE"
          type: "TrafficLightLeftTurnWaitingZone"
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

10. 开发一个仿真测试的脚本，通过键盘输入来控制信号灯的变换。

    文件路径：

    ```bash
    modules/planning/scenarios/traffic_light_left_turn_waiting_zone/tools/manual_traffic_light.py
    ```

    ```bash
    from cyber.python.cyber_py3 import cyber
    from cyber.python.cyber_py3 import cyber_time
    import modules.common_msgs.perception_msgs.traffic_light_detection_pb2 as traffic_light_detection_pb2
    import threading
    import time
    ```


    def add_left_light(color, traffic_light_pb):
        light = traffic_light_pb.traffic_light.add()
        light.color = color
        light.id = "451089192"
        light.tracking_time = 10.0


    def add_forward_light(color, traffic_light_pb):
        light = traffic_light_pb.traffic_light.add()
        light.color = color
        light.id = "451089193"
        light.tracking_time = 10.0


    seq_num = 0


    def add_header(msg):
        global seq_num
        msg.header.sequence_num = seq_num
        msg.header.timestamp_sec = cyber_time.Time.now().to_sec()
        msg.header.module_name = "manual_traffic_light"
        seq_num = seq_num + 1


    def pub_func(writer):
        while not cyber.is_shutdown():
            global traffic_light_msg
            add_header(traffic_light_msg)
            writer.write(traffic_light_msg)
            time.sleep(0.1)


    traffic_light_msg = None

    if __name__ == '__main__':
        traffic_light_msg = traffic_light_detection_pb2.TrafficLightDetection()
        cyber.init()
        node = cyber.Node("traffic_light_command")
        writer = node.create_writer(
            "/apollo/perception/traffic_light", traffic_light_detection_pb2.TrafficLightDetection)
        thread = threading.Thread(target=pub_func, args=(writer,))
        thread.start()
        while not cyber.is_shutdown():
            m = input(
                "1: all red 2: forward green  3: left green 4:all green\n")
            traffic_light_msg.ClearField('traffic_light')
            print(m)
            if m == '1':
                add_left_light(
                    traffic_light_detection_pb2.TrafficLight.RED, traffic_light_msg)
                add_forward_light(
                    traffic_light_detection_pb2.TrafficLight.RED, traffic_light_msg)
            elif m == "2":
                add_left_light(
                    traffic_light_detection_pb2.TrafficLight.RED, traffic_light_msg)
                add_forward_light(
                    traffic_light_detection_pb2.TrafficLight.GREEN, traffic_light_msg)
            elif m == "3":
                add_left_light(
                    traffic_light_detection_pb2.TrafficLight.GREEN, traffic_light_msg)
                add_forward_light(
                    traffic_light_detection_pb2.TrafficLight.RED, traffic_light_msg)
            elif m == "4":
                add_left_light(
                    traffic_light_detection_pb2.TrafficLight.GREEN, traffic_light_msg)
                add_forward_light(
                    traffic_light_detection_pb2.TrafficLight.GREEN, traffic_light_msg)

        cyber.shutdown()
       ```


11. 修改 BUILD 文件，配置编译依赖。

    - 将包内所有代码编译为一个 apollo 的插件，添加代码中所有对基础库的依赖。

      ```bash
      load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
        load("//tools:apollo.bzl", "cyber_plugin_description")
        load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
        load("//tools/proto:proto.bzl", "proto_library")
        load("//tools:cpplint.bzl", "cpplint")

        package(default_visibility = ["//visibility:public"])

        filegroup(
            name = "left_turn_waiting_zone_files",
            srcs = glob([
                "conf/**",
            ]),
        )

        apollo_plugin(
            name = "libtraffic_light_left_turn_waiting_zone.so",
            srcs = [
                "stage_approach.cc",
                "stage_creep.cc",
                "stage_intersection_cruise.cc",
                "traffic_light_left_turn_waiting_zone.cc",
            ],
            hdrs = [
                "stage_approach.h",
                "stage_creep.h",
                "stage_intersection_cruise.h",
                "traffic_light_left_turn_waiting_zone.h",
            ],
            copts = ["-DMODULE_NAME=\\\"planning\\\""],
            description = ":plugin_traffic_light_left_turn_waiting_zone_description.xml",
            deps = [
                "//cyber",
                "//modules/common/util:common_util",
                "//modules/common/util:util_tool",
                "//modules/common_msgs/planning_msgs:planning_cc_proto",
                "//modules/map:apollo_map",
                "//modules/planning/planning_base:apollo_planning_planning_base",
                "//modules/planning/scenarios/traffic_light_left_turn_waiting_zone/proto:traffic_light_left_turn_waiting_zone_cc_proto",
            ],
        )

        apollo_package()

        cpplint()
      ```

12. 编译开发好的包。

    ```bash
    buildtool build -p modules/planning/scenarios/traffic_light_left_turn_waiting_zone
    ```

13. 在 Dreamview 仿真验证。

    - a. 选择 planning2.0 场景集，左转待转场景。
      ![image (40).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2840%29_959df09.png)

      ![image (41).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2841%29_7b2da0d.png)

    - b. 将车放置在地图左下角的路口，左转车道上。

      ![image (42).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2842%29_13cf95c.png)

    - c. 打开 planning 模块。
    - d. 在终端启动红绿灯脚本，使直行灯和左转灯都为红色。

      ```bash
      python3 modules/planning/scenarios/traffic_light_left_turn_waiting_zone/tools/manual_traffic_light.py
      ```

    - e. 发送目标车道的路由请求。

      ![image (43).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2843%29_ee73fc3.png)

    - f. 车辆行驶到停止线后，修改为直行绿灯左转红灯。

      在终端输入：2
      ![image (44).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2844%29_95ca389.png)
      现象：
      ![image (45).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2845%29_a02ad93.png)

    - g. 车辆行驶到左转区停止线后，修改为左转绿灯。

      在终端输入：3
      ![image (46).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2846%29_7e71774.png)
      现象：
      ![image (47).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Doc_CN_9_0/image%20%2847%29_20c2b5d.png)

    - h. 重新仿真测试，当直行区左转灯是绿灯时测试效果
    - i. 重新仿真测试，当左转区由绿灯变成红灯测试效果
