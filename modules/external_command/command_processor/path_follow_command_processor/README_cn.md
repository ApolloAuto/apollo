# external-command-path-follow

## 介绍

`external-command-path-follow` 是用来处理转发外部命令`apollo::external_command::PathFollowCommand`的处理模块。它接收通过cyber Service调用的PathFollowCommand命令，将其转换成planning模块需要的`apollo::planning::PlanningCommand`的channel信息并发布给planning模块。

`apollo::external_command::PathFollowCommand`命令适用于无图由用户沿给定路线行驶的场景，用户在命令中给定全局路径上的坐标点，车辆沿着全局路径作为参考路线行驶。它主要包含以下信息：
- **header**：命令消息头，可调apollo::common::util::FillHeader方法注入信息
- **command_id**：命令唯一标识
- **way_point**： 为给定路线上的坐标点
- **boundary**：为车辆横向可行驶区域，有两种设置方式可选：
  * path_boundary：为每个路径点设置左右边界
  * boundary_with_width：对所有路径点设置统一的左右边界宽度

  如果没有设置boundary，默认参考车辆宽度作为左右边界，在这种情况下车辆遇到障碍物不能绕障，只能停止。
![](./docs/images/path_boundary.png)
PathFollowCommandProcessor继承自模板类MotionCommandProcessorBase<PathFollowCommand>，因此它具有父类的以下特性：
- **具有获取命令执行状态的接口**：通过GetCommandStatus函数，可以获取命令执行的实时状态。
- **接收外部命令并转换成planning的输入**：OnCommand是当用户通过cyber Service调用外部接口时的回调函数，在将外部命令数据转换成PlanningCommand之后，通过planning_command_writer_发布出去。

除了继承父类MotionCommandProcessorBase<PathFollowCommand>对命令处理流程外（参考`external-command-processor-base`的readme文档），PathFollowCommandProcessor对以下两个函数进行了特殊的实现：
- **Convert**：如果MotionCommandProcessorBase的子类接收的外部命令，有包含沿车道线行驶的信息，则需要特殊把这段信息提取出来，放在`apollo::planning::PlanningCommand`的`apollo::routing::RoutingRequest`成员变量中。而PathFollowCommand中并不需要沿车道线行驶，因此这个函数中不需要做任何转换，返回的RoutingRequest为空。
- **ProcessSpecialCommand**：直接将PathFollowCommand的信息拷贝作为`apollo::planning::PlanningCommand`的custom_command成员变量，在planning模块中进行处理。planning模块会根据接收到的custom_command类型进行相应的处理。

## 目录结构

```shell

modules/external_command/command_processor/path_follow_command_processor/
├── path_follow_command_processor
    ├── conf                    // 参数配置文件
    ├── BUILD                   // 构建规则文件
    ├── cyberfile.xml           // 包管理配置文件
    ├── path_follow_command_processor.cc        // 程序入口源码
    ├── path_follow_command_processor.h         // 程序入口头文件
    └── README_cn.md            // 说明文档
```

## 插件

### PathFollowCommandProcessor

apollo::planning::PathFollowCommandProcessor

#### 输入

插件的输入有两个：

1.**执行导航命令**：用户通过cyber Service调用命令，执行沿指定线路行驶的任务。

| service 名                             | Request类型                                   | Response类型                              | <div style="width: 300pt">描述</div> |
| -------------------------------------- | --------------------------------------------- | ----------------------------------------- | ------------------------------------ |
| `/apollo/external_command/path_follow` | `apollo::external_command::PathFollowCommand` | `apollo::external_command::CommandStatus` | 沿指定线路行驶的命令                 |

2.**获取命令状态**：通过cyber Reader监听planning模块发布的命令执行状态。
| Channel 名                        | 类型                                      | <div style="width: 300pt">描述</div>      |
| --------------------------------- | ----------------------------------------- | ----------------------------------------- |
| `/apollo/planning/command_status` | `apollo::external_command::CommandStatus` | 接收planning模块发布的命令执行状态的topic |


#### 输出

插件的输出为命令转换的最终结果：

| Channel 名                 | 类型                                | <div style="width: 300pt">描述</div>                       |
| -------------------------- | ----------------------------------- | ---------------------------------------------------------- |
| `/apollo/planning/command` | `apollo::planning::PlanningCommand` | 具有导航动作的外部命令转换成的内部命令，发送给planning模块 |

#### 配置

| 文件路径                                                                                      | 类型/结构                                          | <div style="width: 300pt">说明</div>                         |
| --------------------------------------------------------------------------------------------- | -------------------------------------------------- | ------------------------------------------------------------ |
| `modules/external_command/command_processor/path_follow_command_processor/conf/config.pb.txt` | `apollo::external_command::CommandProcessorConfig` | 配置文件，外部命令处理器输入输出的channel或service名称等信息 |

#### 使用方式
##### 添加包依赖
如果使用包管理的方式运行apollo，在包管理的cyber.xml文件中添加对插件的依赖：
```shell
<depend repo_name="external-command-path-follow" type="binary">external-command-path-follow</depend>
```
##### 插件加载配置
所有外部命令的插件都是在external-command-process包中加载运行的，需要支持对PathFollowCommand外部命令处理时，在配置文件modules/external_command/process_component/conf/config.pb.txt中添加以下配置：
```shell
processor: "apollo::external_command::PathFollowCommandProcessor"
```
