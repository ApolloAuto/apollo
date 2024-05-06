# external-command-action

## 介绍

`external-command-action` 插件是用来实现通过外部操作命令`apollo::external_command::ActionCommand`对自动驾驶规划控制的流程进行操作的能力。它通过cyber Service接收外部操作命令ActionCommand，通过回调函数执行处理操作指令的方法，将外部的操作指令请求一种是转换成planning模块需要的`apollo::planning::PadMessage`的channel信息，完成planning的决策变化（如切换成沿主路行驶、变道、停车、继续行驶等）；或是另一种是转换成control和canbus模块所需要的`apollo::control::PadMessage`的channel信息，完成底盘驾驶模式的切换（如从非自动驾驶切换至自动驾驶、从自动驾驶切换会非自动驾驶）。

`apollo::external_command::ActionCommand`是一种外部控制自动驾驶的操作指令，适用于在自动驾驶过程中切换车辆的行为，也可以让车辆进入或退出自动驾驶。它主要包含以下信息：
- **header**：命令消息头，可调apollo::common::util::FillHeader方法注入信息
- **command_id**：命令唯一标识
- **command**：自定义数据结构`ActionCommandType`，指定操作指令的类型。包括跟车、换道、靠边停车、紧急停车、再次行驶、进入自动驾驶、退出自动驾驶、车辆识别码获取等相关指令。

ActionCommandProcessor继承自父类CommandProcessorBase，因此它具有父类的以下特性：
- **具有获取命令执行状态的接口**：通过GetCommandStatus函数，可以获取命令执行的实时状态。
- **接收外部命令并处理命令的方法**：OnCommand是当用户通过cyber Service调用外部接口时的回调函数，在将外部命令数据转换成对应的channel数据格式，并Writer(message)发布出去。


## 目录结构

```shell

modules/external_command/command_processor/
├── action_command_processor
    ├── conf                               // 参数配置文件
    ├── BUILD                              // 构建规则文件
    ├── cyberfile.xml                      // 包管理配置文件
    ├── action_command_processor.cc        // 程序入口源码
    ├── action_command_processor.h         // 程序入口头文件
    └── README_cn.md                       // 说明文档
```

## 插件

### ActionCommandProcessor

apollo::planning::ActionCommandProcessor

#### 输入

插件的输入有两个：

1.**执行外部操作命令**：用户通过cyber Service调用命令，执行外部对自动驾驶的操作指令。

| service 名称                        | Request类型                                         | Response类型                                         |<div style="width: 300pt">描述</div>                                                                                                               |
| ---------------------------------- | -------------------------------------------- | -------------------------------------------- |-------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/apollo/external_command/action`               | `apollo::external_command::ActionCommand`   | `apollo::external_command::CommandStatus` | 自动驾驶外部操作命令 |

2.**获取命令状态、获取planning规划信息，获取底盘信息**：通过cyber Reader监听planning模块发布的命令执行状态、planning规划信息，底盘信息等
| Channel 名称                            | 类型                                          | <div style="width: 300pt">描述</div>             |
| -------------------------------------- | --------------------------------------------- | ------------------------------------------------ |
| `/apollo/planning/command_status`                     | `apollo::external_command::CommandStatus`             | planning模块发布的命令执行状态 |
| `/apollo/planning` | `apollo::planning::ADCTrajectory` | planning轨迹信息中车辆规划的当前场景信息 |
| `/apollo/canbus/chassis` | `apollo::canbus::Chassis` | 车辆底盘信息中的驾驶模式信息 |

#### 输出

插件的输出为外部操作指令执行后对应发布的channel信息（/apollo/planning/pad 或 /apollo/control/pad）和更新planning模块发布的命令执行状态（CommandStatus）：

| Channel 名称                            | 类型                                          | <div style="width: 300pt">描述</div>             |
| -------------------------------------- | --------------------------------------------- | ------------------------------------------------ |
| `/apollo/planning/pad`                     | `apollo::planning::PadMessage`             | 改变planning场景行为的指令 |
| `/apollo/control/pad`      | `apollo::control::PadMessage`     | 改变底盘驾驶模式的指令 |
| `/apollo/planning/command_status`                     | `apollo::external_command::CommandStatus`             | 更新planning模块发布的针对外部命令的执行状态 |

#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/external_command/command_processor/action_command_processor/conf/config.pb.txt`                 | `apollo::external_command::CommandProcessorConfig`              | 配置文件，外部操作命令处理器输入输出的channel或service名称等信息  |
| `modules/external_command/command_processor/action_command_processor/conf/special_config.pb.txt`                 | `apollo::external_command::ActionCommandConfig`              | 配置文件，外部操作命令处理器模块的配置  |

#### 使用方式
##### 添加包依赖
如果使用包管理的方式运行apollo，在包管理的cyber.xml文件中添加对插件的依赖：
```shell
<depend repo_name="external-command-action" type="binary">external-command-action</depend>
```
##### 插件加载配置
所有外部命令的插件都是在external-command-process包中加载运行的，需要支持对ActionCommand外部操作指令处理时，在配置文件modules/external_command/process_component/conf/config.pb.txt中添加以下配置：
```shell
processor: "apollo::external_command::ActionCommandProcessor"
```

##### 使用mainboard启动
```shell
mainboard -d /apollo/modules/external_command/process_component/dag/external_command_process.dag
```
