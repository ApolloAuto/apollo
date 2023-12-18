control-controller-demo-control-task
==============

## 简介

control-controller-demo-control-task 插件包是一个任务器的示例，它的主要任务是限制 ControlCommand 中的加速度控制指令的大小，这个加速度可能是前序控制器（如纵向 PID 控制器）输出结果，但是因为某种条件下，我们希望限制加速度的输出，而不是直接以算法控制的输出结果输入到车辆上，因此可以通过增加这样的任务器，任务器内实现人工规则，来干预控制的输出结果。本示例中通过修改配置文件，如果控制指令中加速度指令小于配置的加速度值，则改变加速度指令为配置的加速度值；如果大于，则不改变原来的加速度指令。


## 文件组织结构及说明

```shell
control/controllers/demo_control_task/
├── conf/                                                 // 控制器配置参数文件
├── docs/                                                 // 文档相关
├── proto
│   ├── BUILD
│   └── demo_control_task_conf.proto                      // 控制器配置参数定义
├── BUILD                                                 // 规则构建文件
├── cyberfile.xml                                         // 插件包管理配置文件
├── demo_control_task.cc                                  // 任务器实现文件
├── demo_control_task.h                                   // 任务器实现文件
├── plugins.xml                                           // 插件配置文件
└── README_cn.md                                          // 说明文档
```


## 模块输入输出与配置

### control-controller-demo-control-task插件

#### 输入
| Channel名称 | 类型 | 描述 |
| ---- | ---- | ---- |
| `/apollo/control` | apollo::control::ControlCommand | 车辆的控制指令：加速度指令 |

#### 输出
| Channel名称  | 类型  | 描述 |
| ---- | ---- | ---- |
| `/apollo/control` | apollo::control::ControlCommand | 车辆的控制指令：加速度指令 |

#### 配置文件
| 文件路径 | 类型/结构 | 说明 |
| ---- | ---- | ---- |
| `modules/control/control_component/conf/pipeline.pb.txt` | apollo::control::ControlPipeline | ControlComponent 的配置文件 |
| `modules/control/controllers/demo_control_task/conf/controller_conf.pb.txt` | apollo::control::DemoControlTaskConf | Demo 任务器配置文件 |

#### 使用方式

##### 配置加载DemoControlTask控制器

在 `modules/control/control_component/conf/pipeline.pb.txt` 中的增加配置加载的控制器参数，配置参数中 `name` 表示控制器的名称，这个由用户自定义，表达清楚是哪个控制器即可， `type` 是DemoControlTask控制器的类名称，即DemoControlTask。请注意，如果配置文件内的 `type` 名称和加载的类名称不一致，会导致加载控制器失败。
```
controller {
  name: "DEMO_CONTROLTASK"
  type: "DemoControlTask"
}
```

##### 配置合适的DemoControlTask控制器配置参数

demo任务器控制器配置参数的设置在 `modules/control/controllers/demo_control_task/conf/controller_conf.pb.txt` 文件，按照实际需求通过配置参数定义限制加速度控制指令的大小。

##### 使用 mainboard 启动

```shell
mainboard -d modules/control/dag/control.dag
```

##### 使用 cyber_launch 启动

```shell
cyber_launch start modules/control/launch/control.launch
```
