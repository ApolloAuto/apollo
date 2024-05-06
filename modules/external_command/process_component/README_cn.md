# external-command-process

## 介绍

`external-command-process` 是外部接口命令处理模块的程序入口，从用户的业务模块发出的命令请求，在这个程序中接收处理。它根据配置文件中要处理的外部命令列表，加载处理相应命令的插件，在这些插件中对具体的命令进行处理和转发。用户业务模块发送外部命令时，需要同时启动external-command-process模块，发送的外部命令才会被转发和处理。

![](./docs/images/process_component.png)

## 目录结构

```shell

modules/external_command/process_component/
├── process_component
    ├── conf                    // 参数配置文件
    ├── dag                     // 模块启动文件(mainboard)
    ├── docs                    // 说明文档
    ├── launch                  // 模块启动文件(cyber_launch)
    ├── proto                   // 公共（全局）参数配置结构定义
    ├── BUILD                   // 构建规则文件
    ├── cyberfile.xml           // 包管理配置文件
    ├── external_command_process_component.cc        // 程序入口源码
    ├── external_command_process_component.h         // 程序入口头文件
    └── README_cn.md            // 说明文档
```

## 模块

### ExternalCommandProcessComponent

apollo::planning::ExternalCommandProcessComponent

#### 输入

用户业务模块发送的命令为`apollo::external_command::ExternalCommandProcessComponent`的输入，用户输入命令通过`apollo::cyber::Client`客户端调用。在每个命令对应的CommandProcessor中创建处理命令的`apollo::cyber::Service`，service的Response一般都是当前命令处理的状态。

| service 名                         | Request类型                                         | Response类型                                         |<div style="width: 300pt">描述</div>                                                                                                               |
| ---------------------------------- | -------------------------------------------- | -------------------------------------------- |-------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/apollo/external_command/action`               | `apollo::external_command::ActionCommand`   | `apollo::external_command::CommandStatus` | 流程干预命令，如暂停，启动，切换手动模式等 |
| `/apollo/external_command/chassis`               | `apollo::external_command::ChassisCommand`   | `apollo::external_command::CommandStatus` | 自定义底盘命令（园区） |
| `/apollo/external_command/free_space`               | `apollo::external_command::FreeSpaceCommand`   | `apollo::external_command::CommandStatus` | 指定位姿停车命令（园区） |
| `/apollo/external_command/lane_follow`               | `apollo::external_command::LaneFollowCommand`   | `apollo::external_command::CommandStatus` | 沿道路点对点行驶命令 |
| `/apollo/external_command/path_follow`               | `apollo::external_command::PathFollowCommand`   | `apollo::external_command::CommandStatus` | 指定线路行驶命令（园区） |
| `/apollo/external_command/speed`               | `apollo::external_command::SpeedCommand`   | `apollo::external_command::CommandStatus` | 更改速度命令（园区） |
| `/apollo/external_command/valet_parking`               | `apollo::external_command::ValetParkingCommand`   | `apollo::external_command::CommandStatus` | 指定停车位泊车命令 |

#### 输出

输入外部命令经过预处理，被转换成内部命令发送给planning，control或者canbus模块。被转换成的内部命令以cyber topic的形式发送，有如下几种：

| Channel 名                             | 类型                                          | <div style="width: 300pt">描述</div>             |
| -------------------------------------- | --------------------------------------------- | ------------------------------------------------ |
| `/apollo/planning/command`                     | `apollo::planning::PlanningCommand`             | 具有导航动作的外部命令转换成的内部命令，发送给planning模块 |
| `/apollo/routing_response`      | `apollo::routing::RoutingResponse`     | 在高精地图上沿车道线点对点行驶的外部命令，预处理时生成的routing线路，用于HMI显示时使用  |
| `/apollo/planning/pad`      | `apollo::planning::PadMessage`     | 外部命令ActionCommand转换成的内部命令，发送给planning模块 |
| `/apollo/control/pad`      | `apollo::control::PadMessage`     | 外部命令ActionCommand转换成的内部命令，发送给control模块 |
| `/apollo/canbus/chassis_control`      | `apollo::external_command::ChassisCommand`     | 外部命令ChassisCommand转换成的内部命令，发送给canbus模块 |

#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/external_command/process_component/conf/config.pb.txt`                 | `apollo::planning::PlanningConfig`              | 配置文件，包含程序运行时支持哪些外部命令的处理  |

#### 使用方式

##### 使用 mainboard 启动

```shell
mainboard -d modules/external_command/process_component/dag/external_command_process.dag
```

##### 使用 cyber_launch 启动

```shell
cyber_launch start modules/external_command/process_component/launch/external_command_process.launch
```
