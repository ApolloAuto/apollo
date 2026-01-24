# planning

## 介绍

`planning` 是planning模块的主要流程和入口package，包含planning模块的整体架构和流程。planning模块根据上游模块输入的感知周围环境信息，地图定位导航信息，以及全局路径信息，为自动驾驶车辆规划出一条运动轨迹（包含坐标，速度，加速度，jerk加加速度，时间等信息），然后将这些信息传递给控制模块。

![](./docs/images/architecture_all.png)

### planning package介绍

planning模块由以下几个目录构成：

- **planning_component**: 包含planning组件类和planning程序的启动以及配置文件。
- **planning_interface_base**: 包含planning中几个重要的插件类型（scenario, task, traffic rule和planner）的父类接口文件。
- **planning_base**: 包含planning的基础数据结构和算法库。
- **planner**: 包含planning模块的几种规划器子类插件。
- **pnc_map**: 生成参考线基于的pnc_map类，根据输入的planning导航命令或地图等信息，生成参考线数据，作为planning局部路径规划的路线参考。
- **scenarios**: lanning模块支持的场景插件，每个目录下包含一个独立的场景插件包，包含scenario和stage类的定义。
- **tasks**: lanning模块中支持的任务插件，每个目录下包含一个独立的任务插件包，包含task类的定义。
- **traffic_rules**: lanning模块支持的通用交通规则插件，每个目录下包含一个独立的traffic rule插件包，traffic rules作用于所有运行的场景中。

### planning框架介绍

#### 场景机制

planning模块从apollo 3.5开始使用了双层状态机的场景机制，相比于apollo 3.0之前的版本，每个驾驶用例都被当作不同的驾驶场景，现在在特定场景中报告的问题可以在不影响其他场景的工作的情况下得到修复，其中问题修复不会影响其他驾驶用例。

![](./docs/images/state_machine.png)

双层状态机的Top Layer是Scenario状态机，BottomLayer是Stage状态机。在Top Layer中进行场景切换，ScenarioManager根据周围的环境和地图信息，决定需要切换到哪个场景，如LaneFollow沿道路行驶的场景，PullOver靠边停车的场景。在Bottom Layer中进行Stage的切换，如果上个Stage已经执行完成，继续执行下一个Stage。如果所有的Stage都执行完毕，认为当前Scenario也执行完毕。

目前planning支持以下几种场景：

- **apollo::planning::LaneFollowScenario**: 道保持场景，是默认的自动驾驶场景，在这个场景中车辆沿着路由线路行驶，遇到障碍物根据情况在当前车道线内绕行，或者借道到相邻的车道绕行，并根据routing中的路线信息换道行驶。对道路上的交通标志，如停止，让行标志，人行道或减速带等，根据交通规则进行减速或停止让行。

![](./docs/images/lane_follow_scenario.png)

- **apollo::planning::PullOverScenario**: 边停车场景，如果参数配置 `enable_pull_over_at_destination` 设置为 `true`, 当车辆到达终点附近时，将自动切入 `PullOverScenario` 并完成靠边停车。

![](./docs/images/pull_over_scenario.png)

- **apollo::planning::BareIntersectionUnprotectedScenario**: 保护交通路口场景，在交通路口既没有停止标志，也没有交通灯，车辆在路口前一段距离范围内切换到此场景。

![](./docs/images/bare_intersection_scenario.png)

- **apollo::planning::TrafficLightProtectedScenario**: 保护的交通灯路口场景，在这种路口对前行，左转，右转都有明确的交通灯指示。

![](./docs/images/traffic_light_protected_scenario.png)

- **apollo::planning::TrafficLightUnprotectedLeftTurnScenario**: 保护的交通灯左转，这种场景下，车辆在交通灯路口左转时，仍然会有对向车辆通过路口，这时车辆需要让行。所以TrafficLightUnprotectedLeftTurnScenario场景设计思路与有保护交通灯的区别是，在通过交通路口时，需要增加减速慢行阶段（Creep），以便观察对向车辆，并根据情况让行。

![](./docs/images/traffic_light_unprotected_left_scenario.png)

- **apollo::planning::TrafficLightUnprotectedRightTurnScenario**: 保护的交通灯右转，这种场景下，车辆在交通灯右转时，可能会有对向车辆通过，这时车辆需要缓行，并观察红绿灯情况，在安全的前提下右转。

![](./docs/images/traffic_light_unprotected_right_scenario.png)

- **apollo::planning::StopSignUnprotectedScenario**: 保护停止标志路口场景，这种场景下的路口，只有一个双向停止标志，车辆在通过路口前，需要观察路口来往车辆，在路口通行车辆密度较小时才通过路口。

![](./docs/images/stop_sign_unprotected_scenario.png)

![](./docs/images/stop_sign_unprotected_1.png)

<center>双向停止标志路口</center>

![](./docs/images/stop_sign_unprotected_2.png)

<center>四向停止标志路口</center>

- **apollo::planning::YieldSignScenario**: 口有让行标志的场景，在车辆遇到让行标志时，对向有车辆时，要先让对向车辆先行。

![](./docs/images/yield_sign_scenario.png)

- **apollo::planning::ValetParkingScenario**: 车入库的场景，当planning的输入命令RoutingResponse中包含了parking_id的信息时，即是指定将车辆泊入地图中parking_id对应的停车位中。

![](./docs/images/valet_parking_scenario.png)

- **apollo::planning::EmergencyPullOverScenario**: 急停车场景，车辆在行驶过程中如果收到PadMessage命令“PULL_OVER”，车辆就近找到合适的位置在当前车道内停车，相比于直接停止，这样保证了行驶过程中的停车安全。

![](./docs/images/emergency_pull_over_scenario.png)

- **apollo::planning::ParkAndGoScenario**: 辆靠边停车后，从当前位置起步向指定的下一个目标行驶的命令，这种场景适用于巴士接驳的情况。场景中使用了Open Space的算法，保证车辆能够安全从指定位置泊出。

![](./docs/images/park_and_go_scenario.png)

#### planning模块运行流程

planning模块运行流程如下图所示，模块的入口是PlanningComponent，当有预测信息PredictionObstacles输入时，触发它的Proc函数，进行轨迹规划处理。

![](./docs/images/planning_flow.png)

planning 支持两种规划模式：OnLanePlanning 和 NaviPlanning，前者是基于高精地图的轨迹规划，也是默认的规划模式；后者是相对地图导航规划，主要用于交通规则较简单的高速公路。

每种规划模式可以通过 PlannerDispatcher 选择使用的 Planner ，目前 planning 模块中共有 4 种 Planner：

- **apollo::planning::PublicRoadPlanner**: 于高精地图的规划器；
- **apollo::planning::NaviPlanner**: 于实时相对地图的规划器；
- **apollo::planning::LatticePlanner**: 于网格算法的规划器
- **apollo::planning::RTKReplayPlanner**: 于录制轨迹的规划器

planning模块中有两个主要的线程，一个是根据输入环境和车辆信息，进行轨迹规划的主流程；另外一个是根据地图和输入的全局路线，生成参考线信息，这个线程是周期运行的线程，主流程规划都是在它生成的参考线基础上进行的。

在planning主流程中，默认使用OnLanePlanning->PublicRoadPlanner进行轨迹规划，在PublicRoadPlanner中，根据周围环境信息，切换到不同的场景中，规划生成轨迹并发送给control模块。

#### planning模块入口

PlanningComponent是planning模块的入口，它是一个由topic触发的Component，接口函数是：

```C
    bool Proc(
        const std::shared_ptr<prediction::PredictionObstacles>& prediction_obstacles,
        const std::shared_ptr<canbus::Chassis>& chassis,
        const std::shared_ptr<localization::LocalizationEstimate>&
            localization_estimate) override;
```

当接收到新的 PredictionObstacles 数据时，会触发执行Proc函数，并获取最新的Chassis车辆信息和 LocalizationEstimate 车辆定位数据进行处理，计算planning轨迹。

#### planning初始化

planning初始化在PlanningComponent::Init函数中进行，在这里创建PlanningBase对象（默认OnLanePlanning），它是轨迹规划的主体；除此之外，还需要创建planning其他输入消息的订阅对象，以及输出的消息发布对象：

| 成员对象                      | 类型                                                                                                                                | 描述                                     |
| ----------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------- |
| **planning_command_reader\_** | `std::shared_ptr< apollo::cyber::Reader < apollo::planning::PlanningCommand >>`                                                     | 输入导航命令订阅                         |
| **traffic_light_reader\_**    | `std::shared_ptr< apollo::cyber::Reader < apollo::perception::TrafficLightDetection >>`                                             | 交通灯消息订阅                           |
| **pad_msg_reader\_**          | `std::shared_ptr< apollo::cyber::Reader < apollo::planning::PadMessage >>`                                                          | planning操作命令（start，stop）消息订阅  |
| **story_telling_reader\_**    | `std::shared_ptr< apollo::cyber::Reader < apollo::storytelling::Stories >>`                                                         | storytelling消息订阅                     |
| **relative_map_reader\_**     | `std::shared_ptr< apollo::cyber::Reader < apollo::relative_map::MapMsg >>`                                                          | 实时相对地图消息订阅（用于NaviPlanning） |
| **planning_writer\_**         | `std::shared_ptr< apollo::cyber::Writer < apollo::planning::ADCTrajectory >>`                                                       | planning输出轨迹消息发布                 |
| **rerouting_client\_**        | `std::shared_ptr< apollo::cyber::Client < apollo::external_command::LaneFollowCommand , apollo::external_command::CommandStatus >>` | planning阻塞时需要重新路由的请求         |
| **command_status_writer\_**   | `std::shared_ptr< apollo::cyber::Writer < apollo::external_command::CommandStatus >>`                                               | planning实时任务状态消息发布             |

#### 场景配置

planning中目前主要使用 `OnLanePlanning->PublicRoadPlanner` ，后面的介绍也以PublicRoadPlanner 为主。在PublicRoadPlanner中使用了双层状态机的场景机制，用户希望具体运行时都支持哪些场景的处理，可以在配置文件 `modules/planning/planning_component/conf/planning_config.pb.txt` 中的 `standard_planning_config`字段指定，例如以下配置支持 `LaneFollowScenario` 和 `ValetParkingScenario` 两种场景：

```text
standard_planning_config {
  planner_type: PUBLIC_ROAD
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
}
```

配置列表中场景先后按照优先级从高到低排序，如果判断可以切入前面的场景，则后面的场景不再进行检查。

#### 场景切换

在主流程的线程中每次调用 `apollo::planning::PlanningComponent::Proc` 函数都要根据当前上下文环境，重新判断需要切入哪种场景。

场景更新切换在 `apollo::planning::ScenarioManager::Update` 中进行，它对场景列表中的所有场景遍历，调用场景子类的重写函数 `IsTransferable` ，用于判断当前环境是否能切入到这个场景子类中。因为场景列表优先级从高到低，如果遍历时遇到第一个可以切入的场景，后面的场景不需要再判断，直接使用这个场景作为本次运行周期的当前场景。

![](./docs/images/scenario_switch.png)

#### 场景运行

场景的运行也是在 `apollo::planning::PlanningComponent::Proc` 函数中调用，因为每个场景包含多个 `Stage`，每个 `Stage` 又包含多个 `Task`，所以执行一个场景，就是顺序执行不同阶段的不同任务。

<center><img src="./docs/images/scenario_execute_1.png" width = "50%" height = "50%" alt="图片名称" /></center>
<center>planning主流程</center>

<img src="./docs/images/scenario_execute_2.png" width = "100%" height = "100%" alt="图片名称" align=center />
<center>场景执行流程</center>

#### traffic rules

traffic rules是planning在运行场景之前，根据不同的交通规则，决策车辆是否需要停车，减速或其他操作。因为它是在运行场景之前进行的，所以对所有的场景都会起作用。

![](./docs/images/traffic_rules_flow.png)

目前支持的traffic rules有：

- **backside_vehicle**: 向车辆的处理，决定是否要忽略后向车辆
- **crosswalk**: 行道交通规则，根据人行道附近障碍物情况，决定是否要停车
- **destination**: 近终点的处理，在接近终点时设置停止墙
- **keepclear**: 停区域的处理，在禁停区域车辆不能停车，如果禁停区域内有block障碍物，需要在禁停区域外停车
- **reference_line_end**: 近参考线终点的处理，在参考线终点设置停止墙
- **rerouting**: lanning被阻塞时需要重新路由时的处理，发送rerouting请求
- **stop_sign**: 止标志交通规则的处理，在停止标志前停车
- **traffic_light**: 通信号灯的处理，在交通灯为绿色通行，否则需要停车
- **yield_sign**: 止让行标志的处理，在这个标志附近如果有冲突车辆，自车停止让行

#### reference line

参考线是planning规划算法的基础，ReferenceLineProvider根据车辆的实时位置，计算车辆前后一定范围内（几百米）的参考线信息，相对于全局路由线路来说，参考线是局部路线信息，但参考线中还附加了车辆周围的动态信息，如障碍物，交通灯等。

参考线相关重要的两个数据：

- **ReferenceLine**: 据全局路由线路生成的原始路径信息，包含地图上道路之间的静态关系，并且经过平滑之后的结果。
- **ReferenceLineInfo**: `ReferenceLine`的基础上添加了动态信息，如决策信息，ST图等，planning的规划操作基本都在这个数据结
  构上进行。可以建立理解为ReferenceLine提供的是轨迹信息，而ReferenceLineInfo在ReferenceLine的基础上新添加了决策信息。

参考线生成的流程如下图所示：

<center><img src="./docs/images/reference_line_flow.png" width = "50%" height = "50%" alt="图片名称" /></center>

其中 `CreateRouteSegments` 函数是将车辆附近范围内的全局路由线路转换成参考线的格式；SmoothRouteSegment函数是将原始的参考线进行平滑。

参考线一共有三种平滑方式，离散点的平滑（默认）、螺旋线的平滑以及样条曲线的平滑。

![](./docs/images/reference_line_smoother.png)

- @subpage md_modules_2planning_2planning\_\_base_2docs_2reference\_\_line\_\_smoother\_\_cn "参考线平滑算法"

### 如何进行planning扩展

planning模块框架如下图所示，planning-base包含了主流程，以及相关的外部接口类。planning模块可以让用户根据自己的场景灵活扩展或改造所需要的功能，主要通过以下几种方式： ![](./docs/images/planning_frame.png)

- **修改配置参数**: 户需要的功能已经实现，但需要修改某些场景下的性能表现，可以修改相应的参数来实现。
- **调用外部接口命令**: 用户的场景需要进行多次规划任务才能完成一次作业要求，或者需要在规划任务执行过程中动态改变任务状
  态，可以在业务层根据您的业务需求自行编排规划任务，对规划模块发送命令。目前支持的外部接口有：

  | 命令名称                                                        | 描述             |
  | --------------------------------------------------------------- | ---------------- |
  | `apollo::external_command::LaneFollowCommand`                   | 点到点沿道路行驶 |
  | `apollo::external_command::ValetParkingCommand`                 | 泊车             |
  | `apollo::external_command::ActionCommandType::PULL_OVER`        | 紧急靠边停车     |
  | `apollo::external_command::ActionCommandType::STOP`             | 紧急停车         |
  | `apollo::external_command::ActionCommandType::START`            | 继续行驶         |
  | `apollo::external_command::ActionCommandType::SWITCH_TO_MANUAL` | 切换到手动模式   |
  | `apollo::external_command::ActionCommandType::SWITCH_TO_AUTO`   | 切换到自动模式   |
  | `apollo::external_command::ActionCommandType::VIN_REQ`          | vin code验证     |

- **扩展插件**: 含scenario，task或traffic rule

  planning的二次开发扩展都是以开发插件的形式给出的，在开发planning插件之前需要先了
  解[插件的相关知识和概念](todo:插件的readme)。

  - **开发 scenario 插件**

    Scenario 可以根据地理位置来划分，当场景中主车在特定区域规划动作和默认场景（Lane Follow）存在有所不同是，为了避免影响默认场景的运行，可以为其开发一个新的场景，比如前方有红绿灯需要开发红绿灯场景，前方停止标志需要开发停止标志场景；

    Scenario 也可根据命令来划分，当接收到紧急靠边停车命令时，进入紧急靠边停车场景。比如接收到泊车命令时，进入泊车场景。开发一个新的Scenario需要继承 `apollo::planning::Scenario` 基类，并实现以下几个函数：

    - **初始化函数** ，从场景插件中加载场景的流水线，以及配置参数等： `apollo::planning::Scenario::Init` 。
    - **切换函数** ，被 `apollo::planning::ScenarioManager` 中调用，判断是否需要切入该场景： `apollo::planning::Scenario::IsTransferable` 。
    - **进入场景时的操作函数** ，在首次进入场景前调用做一些预处理的工作，重置场景内变量，如果不需要做任何操作可以不重写： `apollo::planning::Scenario::Enter`
    - **退出场景时的操作函数** ，在场景切出时会被调用，可以用来清除一些全局变量，如果不需要做任何操作可以不重写： `apollo::planning::Scenario::Exit`

  - **开发task插件**

    当Apollo中的任务(Task)无法满足场景需求时，需要开发全新的任务插件。Apollo中存在多种类型的 `apollo::planning::Task` 基类：

    - apollo::planning::PathGeneration ：主要用于在主路上生成路径，比如规划借道路径 LaneBorrowPath 、靠边停车路径 PullOverPath ，沿车道行驶路径 LaneFollowPath 等。
    - apollo::planning::SpeedOptimizer ：主要用于在主路上规划速度曲线，比如基于二次规划的速度规划，基于非线性规划的速度规划。
    - apollo::planning::TrajectoryOptimizer ：主要用于生成轨迹，比如开放空间规划 `apollo::planning::OpenSpaceTrajectoryProvider` 。

    如果上述任务不能满足场景要求，可以继承Task基类实现这个需求，开发新的Task子类需要实现以下几个函数：

    - **初始化函数** ， `apollo::planning::Stage` 在首次运行任务前，会调用任务的 `Init` 函数对任务进行初始化，初始化函数中主要对任务的成员变量进行初始化，以及加载配置参数： `apollo::planning::Task::Init`
    - **运行函数** ，运行函数包含任务的主要运行逻辑： `apollo::planning::Task::Execute`

  - **开发traffic rule插件**

    交通规则插件 traffic rule 主要是在规划模块执行 Scenario 前对交通规则进行处理，当需要增加新的对于全场景生效的决策逻辑时，可以开发新的交通规则插件。

    traffic rule 插件继承自 traffic rule 基类，而后由planning_base中的 traffic_decider 对各个插件进行生成并调用。planning 每进行一次规划任务，会通过 traffic_decider 调用各个 traffic rule , 从而使 traffic rule 插件生效。

    开发新的 traffic rule 插件子类继承自 apollo::planning::TrafficRule ，并实现以下函数：

    - **初始化函数** ，在这个函数中加配置插件的配置参数： `apollo::planning::TrafficRule::Init`
    - **运行函数** ，运行函数包含traffic rule的主要生效逻辑： `apollo::planning::TrafficRule::ApplyRule`

## 目录结构

```shell

modules/planning/planning_component
├── planning_component
    ├── conf                    // 公共（全局）参数配置文件
    ├── dag                     // 模块启动文件(mainboard)
    ├── docs                    // 说明文档
    ├── integration_tests       // 集成单元测试
    ├── launch                  // 模块启动文件(cyber_launch)
    ├── BUILD                   // 构建规则文件
    ├── cyberfile.xml           // 包管理配置文件
    ├── navi_planning.cc        // NaviPlanning相对地图导航模式源码
    ├── navi_planning.h         // NaviPlanning相对地图导航模式头文件
    ├── on_lane_planning.cc     // OnLanePlanning基于高精地图导航模式源码
    ├── on_lane_planning.h      // OnLanePlanning基于高精地图导航模式头文件
    ├── planning_base.cc        // planning模式父类源码
    ├── planning_base.h         // planning模式父类头文件
    ├── planning_component.cc   // planning组件外部接口类源码
    ├── planning_component.h    // planning组件外部接口类头文件
    └── README_cn.md            // 说明文档
```

## 模块

### PlanningComponent

apollo::planning::PlanningComponent

#### 输入

Planning模块需要获取外部环境信息，车辆自身信息进行轨迹规划，以下是planning的外部输入信息：

| Channel 名                         | 类型                                         | <div style="width: 300pt">描述</div>                                                                                                               |
| ---------------------------------- | -------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/apollo/prediction`               | `apollo::prediction::PredictionObstacles`    | 障碍物预测信息，可通过 `modules/planning/planning_component/dag/planning.dag` 启动文件修改channel名                                                     |
| `/apollo/perception/traffic_light` | `apollo::perception::TrafficLight`           | perception模块输出的交通灯感知信息，包含交通灯亮起的颜色，id等信息                                                                                 |
| `/apollo/localization/pose`        | `apollo::localization::LocalizationEstimate` | 定位信息，可通过 `modules/planning/planning_component/dag/planning.dag` 配置文件修改channel名                                                           |
| `/apollo/canbus/chassis`           | `apollo::canbus::Chassis`                    | canbus模块输出的车辆底盘信息，包含底盘速度，油门，刹车，档位，灯光等状态， `modules/planning/planning_component/dag/planning.dag` 配置文件修改channel名 |

此外，planning模块还需要外部输入的导航命令信息，用户首先向external_command发送导航命令请求，external_command再将这些命令进行处理后转发给planning模块。下面介绍用户可以发送的几种导航命令：

| Channel 名                               | 类型                                            | <div style="width: 300pt">描述</div>                                           |
| ---------------------------------------- | ----------------------------------------------- | ------------------------------------------------------------------------------ |
| `/apollo/external_command/lane_follow`   | `apollo::external_command::LaneFollowCommand`   | 基于高精地图导航的命令，给定终点的位置或朝向，从当前车辆位置导航到目标终点位置 |
| `/apollo/external_command/valet_parking` | `apollo::external_command::ValetParkingCommand` | 从当前位置导航泊车到停车位上                                                   |
| `/apollo/external_command/action`        | `apollo::external_command::ActionCommand`       | HMI发送的流程操作命令                                                          |

#### 输出

| Channel 名                             | 类型                                          | <div style="width: 300pt">描述</div>             |
| -------------------------------------- | --------------------------------------------- | ------------------------------------------------ |
| `/apollo/planning`                     | `apollo::planning::ADCTrajectory`             | 输出规划轨迹，包含轨迹点，速度和时间等信息       |
| `/apollo/planning/command_status`      | `apollo::external_command::CommandStatus`     | 导航命令的执行状态                               |
| `/apollo/external_command/lane_follow` | `apollo::external_command::LaneFollowCommand` | 在道路被阻塞，换道失败超时时，发送重新路由的申请 |

#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/planning/planning_component/conf/planning_config.pb.txt`                 | `apollo::planning::PlanningConfig`              | planning组件的配置文件               |
| `modules/planning/planning_component/conf/public_road_planner_config.pb.txt`       | `apollo::planning::PlannerOpenSpaceConfig`      | PublicRoadPlanner的配置文件           |
| `modules/planning/planning_component/conf/traffic_rule_config.pb.txt`             | `apollo::planning::TrafficRulesPipeline`        | 支持的traffic rules列表的配置文件    |
| `modules/planning/planning_component/conf/discrete_points_smoother_config.pb.txt` | `apollo::planning::ReferenceLineSmootherConfig` | 参考线使用离散点平滑时的配置文件     |
| `modules/planning/planning_component/conf/qp_spline_smoother_config.pb.txt`       | `apollo::planning::ReferenceLineSmootherConfig` | 参考线使用五次多项式平滑时的配置文件 |
| `modules/planning/planning_component/conf/spiral_smoother_config.pb.txt`          | `apollo::planning::ReferenceLineSmootherConfig` | 参考线使用五次螺旋线平滑时的配置文件 |
| `modules/planning/planning_component/conf/planner_open_space_config.pb.txt`       | `apollo::planning::PlannerOpenSpaceConfig`      | 开放空间规划算法的配置文件           |

#### Flags

| 文件路径                                            |  <div style="width: 300pt">说明</div> |
| --------------------------------------------------- |  ------------------------------------ |
| `modules/planning/planning_component/conf/planning.conf` |  planning模块的flag配置文件           |

#### 使用方式

##### 使用 mainboard 启动

```shell
mainboard -d modules/planning/planning_component/dag/planning.dag
```

##### 使用 cyber_launch 启动

```shell
cyber_launch start modules/planning/planning_component/launch/planning.launch
```
