control-controller-antislip-control-task
==============

## 简介

control-controller-antislip-control-task 插件包是防溜坡任务器，主要作用是车辆行驶在有坡度的路况下，需要停止在坡上，避免车辆溜车；在坡度上停车后，并重新在坡度上启动，避免车辆溜车。通过修改配置文件，可以设置启动防溜车功能开启的条件，可以设置在坡度上的刹车指令的大小。防溜坡任务器在设计在纵向控制器之后的任务处理，对纵向控制器输出控制指令，再进行判断是否满足防溜坡条件，如果满足修改控制指令（刹车、油门）为启动防溜坡逻辑后的控制指令。


## 文件组织结构及说明

```shell
control/controllers/slope_anti_slip_control_task/
├── conf/                                                 // 控制器配置参数文件
├── docs/                                                 // 文档相关
├── proto
│   ├── BUILD
│   └── antislip_control_task_conf.proto                  // 控制器配置参数定义
├── BUILD                                                 // 规则构建文件
├── cyberfile.xml                                         // 插件包管理配置文件
├── anti_slip_control_task.cc                             // 任务器实现文件
├── anti_slip_control_task.h                              // 任务器实现文件
├── plugins.xml                                           // 插件配置文件
└── README_cn.md                                          // 说明文档
```


## 模块输入输出与配置

### control-controller-antislip-control-task插件

#### 输入
| Channel名称 | 类型 | 描述 |
| ---- | ---- | ---- |
| `/apollo/control` | apollo::control::ControlCommand | 车辆的控制指令：刹车、油门 |
| `/apollo/control` | apollo::control::ControlCommand::SimpleLongitudinalDebug | 车辆 PID 纵向控制 Debug 信息（path_remain） |
| - | apollo::common::VehicleState | 车身姿态信息（车身俯仰角） |

#### 输出
| Channel名称  | 类型  | 描述 |
| ---- | ---- | ---- |
| `/apollo/control` | apollo::control::ControlCommand | 车辆的控制指令：刹车、油门、EPB手刹指令 |

#### 配置文件
| 文件路径 | 类型/结构 | 说明 |
| ---- | ---- | ---- |
| `modules/control/control_component/conf/pipeline.pb.txt` | apollo::control::ControlPipeline | ControlComponent的配置文件 |
| `modules/control/control_component/conf/control.conf` | `command line flags` | 命令行参数配置，配置全局的 flag 变量 |
| `modules/control/controllers/slope_anti_slip_control_task/conf/controller_conf.pb.txt` | apollo::control::DemoControlTaskConf | AntiSlip （防溜坡）任务器配置文件 |

#### Flags
| flagfile | 类型 | 描述 |
| ---- | ---- | ------ |
| `modules/control/control_component/common/control_gflags.cc` | `flags` | 定义全局的 flag 变量在 AntiSlipControlTask 使用，通过control.conf进行配置 |
| `modules/control/control_component/common/control_gflags.h` | `declare` | flags 声明文件 |

#### 使用方式

##### 配置加载AntiSlipControlTask控制器

在 `modules/control/control_component/conf/pipeline.pb.txt` 中的增加配置加载的控制器参数，配置参数中 `name` 表示控制器的名称，这个由用户自定义，表达清楚是哪个控制器即可， `type` 是AntiSlipControlTask控制器的类名称，即AntiSlipControlTask。请注意，如果配置文件内的 `type` 名称和加载的类名称不一致，会导致加载控制器失败。
```
controller {
  name: "ANTI_CONTROLTASK"
  type: "AntiSlipControlTask"
}
```

##### 配置合适的AntiSlipControlTask控制器配置参数

AntiSlip任务器控制器配置参数的设置在 `modules/control/controllers/slope_anti_slip_control_task/conf/controller_conf.pb.txt` 文件，配置启动防溜坡的阈值，车身姿态俯仰角的滤波截止频率等。

##### 使用 mainboard 启动

```shell
mainboard -d modules/control/dag/control.dag
```

##### 使用 cyber_launch 启动

```shell
cyber_launch start modules/control/launch/control.launch
```
