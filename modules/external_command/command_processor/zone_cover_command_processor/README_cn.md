external-command-zone-cover
==============

## 简介

external-command-zone-cover 插件主要功能是外部接口命令进行封装处理，用户可以通过自定义的接口向PNC模块发送相关控制命令，处理后由PNC模块执行相关的Command，实现方式主要通过Service的通讯方式，通过Client端发送PNC外部请求控制指令，Service对Client端请求命令进行处理后交付PNC模块进行执行。

### Convert
该函数暂未做处理，在`external-command-zone-cover`中未做任何处理。

### ProcessSpecialCommand
该函数的主要调用在`motion_command_processor_base.cc`文件中的`OnCommand`函数中，`OnCommand`函数的主要目的是透传Client端的Request请求，该函数作为Service的主要功能实现函数，`command`为Client端Request，Client端的创建函数在`external_command_wrapper_demo`中的`Init()`函数中，`ProcessSpecialCommand`函数主要将Client端接收的请求透传给`planning_command`，`planning_command`后续写入`apollo::planning::PlanningCommand`channel中由PNC模块执行相关请求。

## 文件组织结构及说明

```shell
command_processor/command_processor/zone_cover_command_processor/
├── conf/                                                 // 控制器配置参数文件
├── BUILD                                                 // 规则构建文件
├── cyberfile.xml                                         // 插件包管理配置文件
├── zone_cover_command_processor.cc                       // 任务器实现文件
├── zone_cover_command_processor.h                        // 任务器实现文件
├── plugins.xml                                           // 插件配置文件
└── README_cn.md                                          // 说明文档
```


## 模块输入输出与配置

### external-command-chassis插件

#### 输入


| service 名                            | Request类型                                  | Response类型                              | <div style="width: 300pt">描述</div> |
| ------------------------------------- | -------------------------------------------- | ----------------------------------------- | ------------------------------------ |
| `/apollo/external_command/zone_cover` | `apollo::external_command::ZoneCoverCommand` | `apollo::external_command::CommandStatus` | 开阔场景下车辆的Command请求          |
#### 输入
| Channel名称                       | 类型                                    | 描述                           |
| --------------------------------- | --------------------------------------- | ------------------------------ |
| `/apollo/planning/command_status` | apollo::external_command::CommandStatus | 车辆Planning模块请求的实际状态 |


#### 配置文件
| 文件路径                                                                                | 类型/结构                                          | 说明                                                    |
| --------------------------------------------------------------------------------------- | -------------------------------------------------- | ------------------------------------------------------- |
| `modules/external_command/command_processor/zone_cover_command_processor/config.pb.txt` | `apollo::external_command::CommandProcessorConfig` | zone_cover_command_processor的输入、输出channel配置文件 |
