# external-command-speed

## 介绍

`external-command-speed`是用来处理转发外部命令`apollo::external_command::SpeedCommand`的处理模块。它接收通过cyber Service调用的SpeedCommand命令，将其转换成planning模块需要的`apollo::planning::PlanningCommand`的channel信息并发布给planning模块。

`apollo::external_command::SpeedCommand`命令用于在行驶过程中，实时改变当前的巡航速度。它主要包含以下信息：
- **header**：命令消息头，可调apollo::common::util::FillHeader方法注入信息
- **command_id**：命令唯一标识
- **linear_speed**：为设定的目标巡航速度，有3种设置情况：
  - target_speed：新的目标巡航速度的大小
  - target_speed_factor：当前巡航速度调整比例，新的巡航速度为当前巡航速度乘以target_speed_factor
  - is_restore_target_speed：是否恢复原来设定的巡航速度，为true时取消之前通过SpeedCommand设置的巡航速度

SpeedCommandProcessor继承自apollo::external_command::CommandProcessorBase（参考external-command-processor-base包的readme），在父类的框架下它对以下三个虚函数进行了自己的实现：
- **Init**：初始化函数中除了初始化父类的成员变量外，还创建了自身用于接收和发送信息的设备，主要有：
  - std::shared_ptr<cyber::Service<SpeedCommand, CommandStatus>> command_service_：处理SpeedCommand请求的服务。
  - std::shared_ptr<WriterHandle> planning_command_writer_：将SpeedCommand转换成PlanningCommand后，对PlanningCommand消息的发送者。
  - MessageReader* message_reader_：订阅planning模块发布的任务执行状态CommandStatus消息。
- **GetCommandStatus**：获取改变巡航速度命令的执行状态，一般在planning模块接收到之后就视为命令执行完成。
- **OnCommand**：接收到SpeedCommand请求时的回调函数，主要将接收到的SpeedCommand信息拷贝到PlanningCommand消息体中的custom_command中，作为扩展的用户自定义命令。planning模块会根据接收到的custom_command类型进行相应的处理。

## 目录结构

```shell

modules/external_command/command_processor/speed_command_processor/
├── speed_command_processor
    ├── conf                    // 参数配置文件
    ├── BUILD                   // 构建规则文件
    ├── cyberfile.xml           // 包管理配置文件
    ├── speed_command_processor.cc        // 程序入口源码
    ├── speed_command_processor.h         // 程序入口头文件
    └── README_cn.md            // 说明文档
```

## 插件

### SpeedCommandProcessor

apollo::planning::SpeedCommandProcessor

#### 输入

插件的输入有两个：

1.**执行导航命令**：用户通过cyber Service调用命令，执行设定巡航速度的命令。

| service 名                       | Request类型                              | Response类型                              | <div style="width: 300pt">描述</div> |
| -------------------------------- | ---------------------------------------- | ----------------------------------------- | ------------------------------------ |
| `/apollo/external_command/speed` | `apollo::external_command::SpeedCommand` | `apollo::external_command::CommandStatus` | 设定巡航速度的命令                   |

2.**获取命令状态**：通过cyber Reader监听planning模块发布的命令执行状态。
| Channel 名                        | 类型                                      | <div style="width: 300pt">描述</div>      |
| --------------------------------- | ----------------------------------------- | ----------------------------------------- |
| `/apollo/planning/command_status` | `apollo::external_command::CommandStatus` | 接收planning模块发布的命令执行状态的topic |


#### 输出

插件的输出为命令转换的最终结果：

| Channel 名                 | 类型                                | <div style="width: 300pt">描述</div>                 |
| -------------------------- | ----------------------------------- | ---------------------------------------------------- |
| `/apollo/planning/command` | `apollo::planning::PlanningCommand` | 指定巡航速度命令转换成的内部命令，发送给planning模块 |

#### 配置

| 文件路径                                                                                | 类型/结构                                          | <div style="width: 300pt">说明</div>                         |
| --------------------------------------------------------------------------------------- | -------------------------------------------------- | ------------------------------------------------------------ |
| `modules/external_command/command_processor/speed_command_processor/conf/config.pb.txt` | `apollo::external_command::CommandProcessorConfig` | 配置文件，外部命令处理器输入输出的channel或service名称等信息 |

#### 使用方式
##### 添加包依赖
如果使用包管理的方式运行apollo，在包管理的cyber.xml文件中添加对插件的依赖：
```shell
<depend repo_name="external-command-speed" type="binary">external-command-speed</depend>
```
##### 插件加载配置
所有外部命令的插件都是在external-command-process包中加载运行的，需要支持对SpeedCommand外部命令处理时，在配置文件modules/external_command/process_component/conf/config.pb.txt中添加以下配置：
```shell
processor: "apollo::external_command::SpeedCommandProcessor"
```
