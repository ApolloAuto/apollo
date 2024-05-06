# external-command-valet-parking

## 介绍

`external-command-valet-parking` 是用来处理转发外部命令`apollo::external_command::ValetParkingCommand`的处理模块。它接收通过cyber Service调用的ValetParkingCommand命令，提取其中的泊车信息，转换成planning模块需要的`apollo::planning::PlanningCommand`的channel信息并发布给planning模块。

`apollo::external_command::ValetParkingCommand`适用于指定地图上的停车位进行泊车的场景。它主要包含以下信息：
- **header**：命令消息头，可调apollo::common::util::FillHeader方法注入信息
- **command_id**：命令唯一标识
- **is_start_pose_set**：是否设定起点标识，若为true，则以第一个路由点作为起点，否则以车辆当前位置作为起点
- **way_point**：指定途径点
- **end_pose**：指定终止点
- **blacklisted_lane**：指定不可通行的lanesegment列表
- **blacklisted_lane**：指定不可通行的road列表
- **parking_spot_id**：指定停车位编号
- **target_speed**：指定巡航速度

ValetParkingCommandProcessor继承自模板类MotionCommandProcessorBase<ValetParkingCommand>，因此它具有父类的以下特性：
- **具有获取命令执行状态的接口**：通过GetCommandStatus函数，可以获取命令执行的实时状态。
- **接收外部命令并转换成planning的输入**：OnCommand是当用户通过cyber Service调用外部接口时的回调函数，在将外部命令数据转换成PlanningCommand之后，通过planning_command_writer_发布出去。

在ValetParkingCommandProcessor中，实现了对ValetParkingCommand的转换。主要的处理如下：
- **Convert**：提取ValetParkingCommand中的沿道路行驶信息。如果发送ValetParkingCommand的车辆起点位置不在停车位附近范围内，ValetParkingCommand命令将会包含一段从起点位置沿车道线行驶，到停车准备点停下，再进行泊车的过程。否则，将直接进行泊车。如果需要先沿车道线行驶到停车准备点，Convert将提取到RoutingRequest信息放入PlanningCommand中，由planning模块执行。提取过程主要有以下步骤：
  ![](./docs/images/parking_stages.png)
  - 处理起点位置，如果ValetParkingCommand中没有指定起点位置，获取当前车辆的位置作为起点位置。
  - 计算停车准备点。从ValetParkingCommand中获取到停车位的id：“parking_spot_id”，然后从地图中获取到停车位的位置信息，寻找离停车位位置最近的车道线，距离停车位在车道线上投影一定距离，取一个点作为停车准备点。车辆会先沿道路行驶到停车准备点停下，然后使用openspace算法规划泊车的路线。
  - 将起点位置到停车准备点的过程提取成RoutingRequest，存入`apollo::planning::PlanningCommand`的lane_follow_command中。
- **ProcessSpecialCommand**：除了Approach Stage需要的lane_follow_command外，Parking Stage阶段需要给出停车位的信息。ProcessSpecialCommand函数的主要处理过程是，将停车位信息，也就是“parking_spot_id”放在`apollo::planning::PlanningCommand`的parking_command中。

## 目录结构

```shell

modules/external_command/command_processor/valet_parking_command_processor/
├── valet_parking_command_processor
    ├── conf                    // 参数配置文件
    ├── docs                    // 说明文档引用内容
    ├── BUILD                   // 构建规则文件
    ├── cyberfile.xml           // 包管理配置文件
    ├── valet_parking_command_processor.cc        // 程序入口源码
    ├── valet_parking_command_processor.h         // 程序入口头文件
    └── README_cn.md            // 说明文档
```

## 插件

### ValetParkingCommandProcessor

apollo::planning::ValetParkingCommandProcessor

#### 输入

插件的输入有两个：

1.**执行导航命令**：用户通过cyber Service调用命令，执行点到点沿道路行驶的任务。

| service 名                         | Request类型                                         | Response类型                                         |<div style="width: 300pt">描述</div>                                                                                                               |
| ---------------------------------- | -------------------------------------------- | -------------------------------------------- |-------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/apollo/external_command/valet_parking`               | `apollo::external_command::ValetParkingCommand`   | `apollo::external_command::CommandStatus` | 泊车命令 |

2.**获取命令状态**：通过cyber Reader监听planning模块发布的命令执行状态。
| Channel 名                             | 类型                                          | <div style="width: 300pt">描述</div>             |
| -------------------------------------- | --------------------------------------------- | ------------------------------------------------ |
| `/apollo/planning/command_status`                     | `apollo::external_command::CommandStatus`             | 接收planning模块发布的命令执行状态的topic |


#### 输出

插件的输出为命令转换的最终结果（PlanningCommand）和中间结果（RoutingResponse）：

| Channel 名                             | 类型                                          | <div style="width: 300pt">描述</div>             |
| -------------------------------------- | --------------------------------------------- | ------------------------------------------------ |
| `/apollo/planning/command`                     | `apollo::planning::PlanningCommand`             | 具泊车命令转换成的内部命令，发送给planning模块 |
| `/apollo/routing_response`      | `apollo::routing::RoutingResponse`     | 在高精地图上沿车道线点对点行驶的外部命令，预处理时生成的routing线路，用于HMI显示时使用  |

#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/external_command/command_processor/valet_parking_command_processor/conf/config.pb.txt`                 | `apollo::planning::CommandProcessorConfig`              | 配置文件，外部命令处理器输入输出的channel或service名称等信息  |

#### 使用方式
##### 添加包依赖
如果使用包管理的方式运行apollo，在包管理的cyber.xml文件中添加对插件的依赖：
```shell
<depend repo_name="external-command-valet-parking" type="binary">external-command-valet-parking</depend>
```
##### 插件加载配置
所有外部命令的插件都是在external-command-process包中加载运行的，需要支持对ValetParkingCommand外部命令处理时，在配置文件modules/external_command/process_component/conf/config.pb.txt中添加以下配置：
```shell
processor: "apollo::external_command::ValetParkingCommandProcessor"
```
