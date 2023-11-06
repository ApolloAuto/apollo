# external-command-lane-follow

## 介绍

`external-command-lane-follow` 是用来处理转发外部命令`apollo::external_command::LaneFollowCommand`的处理模块。它接收通过cyber Service调用的LaneFollowCommand命令，提取其中的点到点行驶信息，转换成planning模块需要的`apollo::planning::PlanningCommand`的channel信息并发布给planning模块。

`apollo::external_command::LaneFollowCommand`是planning最常用的行驶命令，适用于结构化道路点到点沿道路行驶的场景。它主要包含以下信息：
- **header**：命令消息头，可调apollo::common::util::FillHeader方法注入信息
- **command_id**：命令唯一标识
- **is_start_pose_set**：是否设定起点标识，若为true，则以第一个路由点作为起点，否则以车辆当前位置作为起点
- **way_point**：指定途径点
- **end_pose**：指定终止点
- **blacklisted_lane**：指定不可通行的lanesegment列表
- **blacklisted_lane**：指定不可通行的road列表
- **target_speed**：指定巡航速度

LaneFollowCommandProcessor继承自模板类MotionCommandProcessorBase<LaneFollowCommand>，因此它具有父类的以下特性：
- **具有获取命令执行状态的接口**：通过GetCommandStatus函数，可以获取命令执行的实时状态。
- **接收外部命令并转换成planning的输入**：OnCommand是当用户通过cyber Service调用外部接口时的回调函数，在将外部命令数据转换成PlanningCommand之后，通过planning_command_writer_发布出去。

特别地，在LaneFollowCommandProcessor中，实现了对LaneFollowCommand的转换。主要有两种情况：
- **车辆在车道线范围内**：这种情况比较简单，将命令中起点和终点的位置，通过查询地图获取离它们最近的车道，作为`apollo::routing::RoutingRequest`的起终点位置发送即可。如果外部命令LaneFollowCommand中没有指定起点位置，获取车辆当前的位姿作为起点位置处理。
- **车辆在车道线范围外**：当前车辆离道路比较远，这种情况就需要先将车辆导航到最近的车道线上，然后再进行点到点的道路行驶。这种情况下分两个步骤：1.通过openspace算法将车辆导航到最近的车道线上; 2.车辆从最近车道线上的点沿道路点到点行驶。值得注意的是，虽然车辆实际需要分成两个步骤行驶，但转换的PlanningCommand仍然只需要包含一个点到点的RoutingRequest命令即可，planning模块运行时会自动根据车辆当前的定位，先切换成ParkAndGo的场景将车辆导航到最近车道线上。在LaneFollowCommandProcessor中，需要处理没有指定车道线上起点位置的情况，根据车辆当前的位姿，匹配车道线上最近的点。如果附近的车道线是双向单车道，则会匹配到两个点。这种情况下，根据匹配点到终点之间routing的结果，选routing路线最短的那条作为路径的起点。

## 目录结构

```shell

modules/external_command/command_processor/lane_follow_command_processor/
├── lane_follow_command_processor
    ├── conf                    // 参数配置文件
    ├── BUILD                   // 构建规则文件
    ├── cyberfile.xml           // 包管理配置文件
    ├── lane_follow_command_processor.cc        // 程序入口源码
    ├── lane_follow_command_processor.h         // 程序入口头文件
    └── README_cn.md            // 说明文档
```

## 插件

### LaneFollowCommandProcessor

apollo::planning::LaneFollowCommandProcessor

#### 输入

插件的输入有两个：

1.**执行导航命令**：用户通过cyber Service调用命令，执行点到点沿道路行驶的任务。

| service 名                         | Request类型                                         | Response类型                                         |<div style="width: 300pt">描述</div>                                                                                                               |
| ---------------------------------- | -------------------------------------------- | -------------------------------------------- |-------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/apollo/external_command/lane_follow`               | `apollo::external_command::LaneFollowCommand`   | `apollo::external_command::CommandStatus` | 沿道路点对点行驶命令 |

2.**获取命令状态**：通过cyber Reader监听planning模块发布的命令执行状态。
| Channel 名                             | 类型                                          | <div style="width: 300pt">描述</div>             |
| -------------------------------------- | --------------------------------------------- | ------------------------------------------------ |
| `/apollo/planning/command_status`                     | `apollo::external_command::CommandStatus`             | 接收planning模块发布的命令执行状态的topic |


#### 输出

插件的输出为命令转换的最终结果（PlanningCommand）和中间结果（RoutingResponse）：

| Channel 名                             | 类型                                          | <div style="width: 300pt">描述</div>             |
| -------------------------------------- | --------------------------------------------- | ------------------------------------------------ |
| `/apollo/planning/command`                     | `apollo::planning::PlanningCommand`             | 具有导航动作的外部命令转换成的内部命令，发送给planning模块 |
| `/apollo/routing_response`      | `apollo::routing::RoutingResponse`     | 在高精地图上沿车道线点对点行驶的外部命令，预处理时生成的routing线路，用于HMI显示时使用  |

#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/external_command/command_processor/lane_follow_command_processor/conf/config.pb.txt`                 | `apollo::planning::CommandProcessorConfig`              | 配置文件，外部命令处理器输入输出的channel或service名称等信息  |

#### 使用方式
##### 添加包依赖
如果使用包管理的方式运行apollo，在包管理的cyber.xml文件中添加对插件的依赖：
```shell
<depend repo_name="external-command-lane-follow" type="binary">external-command-lane-follow</depend>
```
##### 插件加载配置
所有外部命令的插件都是在external-command-process包中加载运行的，需要支持对LaneFollowCommand外部命令处理时，在配置文件modules/external_command/process_component/conf/config.pb.txt中添加以下配置：
```shell
processor: "apollo::external_command::LaneFollowCommandProcessor"
```
