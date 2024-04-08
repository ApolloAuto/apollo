# module-controls

## 模块概述

- Control模块由control组件包和controller控制器包组成，control组件包是control模块的基础组件，包含control的整体架构和流程。control根据上游模块输入planning模块的期望轨迹信息，定位模块的当前定位信息，车辆底盘及车身状态信息，通过不同的控制算法计算控制车辆的指令（包含转向、油门、刹车等）输出给canbus模块，为自动驾驶车辆提供舒适的驾驶体验。

## 输入输出

### 输入

| Channel名称                 | 类型                                        | 描述                                 |
| --------------------------- | ------------------------------------------- | ------------------------------------ |
| `/apollo/planning`          | apollo::planning::ADCTrajectory             | 车辆规划轨迹线信息                   |
| `/apollo/localization/pose` | apollo::localization::LocalizationEstimate  | 车辆定位信息                         |
| `/apollo/canbus/chassis`    | apollo::canbus::Chassis                     | 车辆底盘信息                         |
| -                           | apollo::common::VehicleState                | 车身姿态信息                         |
| `/apollo/control/pad`       | apollo::control::ControlCommand::PadMessage | 自动驾驶使能（请求进入自动驾驶）指令 |

### 输出

| Channel名称       | 类型                            | 描述                                       |
| ----------------- | ------------------------------- | ------------------------------------------ |
| `/apollo/control` | apollo::control::ControlCommand | 车辆的控制指令，如方向盘、油门、刹车等信息 |

## 参数

### 配置

| 文件路径                                                          | 类型/结构                          | 说明                       |
| ----------------------------------------------------------------- | ---------------------------------- | -------------------------- |
| `modules/control/control_component/conf/pipeline.pb.txt`          | apollo::control::ControlPipeline   | ControlComponent的配置文件 |
| `modules/control/control_component/conf/control.conf`             | `command line flags`               | 命令行参数配置             |
| `modules/control/control_component/conf/calibration_table.pb.txt` | apollo::control::calibration_table | 车辆纵向标定表配置         |

### Flags

| flagfile                                              | 类型    | 描述                 |
| ----------------------------------------------------- | ------- | -------------------- |
| `modules/control/control_component/conf/control.conf` | `flags` | control模块flags配置 |

## 包列表

| 包名                                                                                                             | 包路径                                         | 说明                                                                            |
| ---------------------------------------------------------------------------------------------------------------- | ---------------------------------------------- | ------------------------------------------------------------------------------- |
| [control](modules/control/control_component/README_cn.md)                                                        | `control/control_component`                    | canbus组件包，负责启动canbus模块，串联canbus运行流程                            |
| [control-controller-lon-based-pid-controller](modules/control/controllers/lon_based_pid_controller/README_cn.md) | `control/controllers/lon_based_pid_controller` | 基于PID算法的纵向控制器组件包，负责车辆纵向的速度和位置跟踪轨迹线控制           |
| [control-controller-lat-based-lqr-controller](modules/control/controllers/lat_based_lqr_controller/README_cn.md) | `control/controllers/lat_based_lqr_controller` | 基于LQR算法的横向控制器组件包，负责车辆横向位置和航向角跟踪轨迹线控制           |
| [control-controller-mpc-controller](modules/control/controllers/mpc_controller/README_cn.md)                     | `control/controllers/mpc_controller`           | 基于MPC算法的横纵向控制器组件包，负责车辆横纵向位置、速度、航向角跟踪轨迹线控制 |
| [control-controller-demo-control-task](modules/control/controllers/demo_control_task/README_cn.md)               | `control/controllers/demo_control_task`        | demo控制器组件包，示例如何添加一个控制器的demo                                  |
