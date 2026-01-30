# module-canbus

## 模块概述

- canbus模块的功能是实现与底盘线控CAN信号的通信，与底盘通信的信息包含4个部分：接收并解析底盘CAN总线数据，获取车辆的信息（如车辆速度、档位、方向盘转角、运行状态、安全等信息）；发送并控制底盘的CAN总线数据，接收控制模块指令，将控制指令解析成底盘CAN总线数据，发送至车辆进行执行控制；执行底盘交互逻辑，使能或退出自动驾驶；进行底盘故障监控，对底盘故障或接管等情况进行监控。

## 输入输出

### 输入

| Channel名称               | 类型                                       | 描述         |
| ------------------------- | ------------------------------------------ | ------------ |
| `/apollo/control`         | `apollo::control::ControlCommand`          | 控制指令     |
| `/apollo/guardian`        | `apollo::guardian::GuardianCommand`        | 安全指令     |
| `/apollo/chassis_control` | `apollo::external_command::ChassisCommand` | 外部底盘命令 |

### 输出

| Channel名称              | 类型                      | 描述                                                                 |
| ------------------------ | ------------------------- | -------------------------------------------------------------------- |
| `/apollo/chassis`        | `apollo::canbus::Chassis` | 车辆底盘信息接口数据，包括车辆速度、方向盘转角、档位、底盘状态等信息 |
| `/apollo/chassis_detail` | `apollo::${Vehicle_Type}` | 车辆底盘详细信息，展示发送和接收底盘报文解析数据                     |

## 参数

### 配置

| 文件路径                                 | 类型/结构                    | 说明                                         |
| ---------------------------------------- | ---------------------------- | -------------------------------------------- |
| `modules/canbus/conf/canbus_conf.pb.txt` | `apollo::canbus::CanbusConf` | `apollo::canbus::CanbusComponent` 的配置文件 |

### Flags

| flagfile                          | 类型     | 描述                |
| --------------------------------- | -------- | ------------------- |
| `modules/canbus/conf/canbus.conf` | `gflags` | canbus模块flags配置 |

## 包列表

| 包名                                                                        | 包路径                      | 说明                                                                         |
| --------------------------------------------------------------------------- | --------------------------- | ---------------------------------------------------------------------------- |
| [canbus](modules/canbus/README_cn.md)                                       | `canbus`                    | canbus组件包，负责启动canbus模块，串联canbus运行流程                         |
| [canbus-vehicle-lincoln](modules/canbus_vehicle/lincoln/README_cn.md)       | `canbus_vehicle/lincoln`    | lincoln车型组件包，负责lincoln车型的报文解析、交互控制、故障监控等功能       |
| [canbus-vehicle-ch](modules/canbus_vehicle/ch/README_cn.md)                 | `canbus_vehicle/ch`         | ch车型组件包，负责ch车型的报文解析、交互控制、故障监控等功能                 |
| [canbus-vehiclekit](modules/canbus_vehicle/devkit/README_cn.md)             | `canbus_vehicle/devkit`     | devkit车型组件包，负责devkit车型的报文解析、交互控制、故障监控等功能         |
| [canbus-vehicle-ge3](modules/canbus_vehicle/ge3/README_cn.md)               | `canbus_vehicle/ge3`        | devkit车型组件包，负责devkit车型的报文解析、交互控制、故障监控等功能         |
| [canbus-vehicle-gem](modules/canbus_vehicle/gem/README_cn.md)               | `canbus_vehicle/gem`        | gem车型组件包，负责gem车型的报文解析、交互控制、故障监控等功能               |
| [canbus-vehicle-lexus](modules/canbus_vehicle/lexus/README_cn.md)           | `canbus_vehicle/lexus`      | lexus车型组件包，负责lexus车型的报文解析、交互控制、故障监控等功能           |
| [canbus-vehicle-neolix-edu](modules/canbus_vehicle/neolix_edu/README_cn.md) | `canbus_vehicle/neolix_edu` | neolix_edu车型组件包，负责neolix_edu车型的报文解析、交互控制、故障监控等功能 |
| [canbus-vehicle-transit](modules/canbus_vehicle/transit/README_cn.md)       | `canbus_vehicle/transit`    | transit车型组件包，负责transit车型的报文解析、交互控制、故障监控等功能       |
| [canbus-vehicle-wey](modules/canbus_vehicle/wey/README_cn.md)               | `canbus_vehicle/wey`        | wey车型组件包，负责wey车型的报文解析、交互控制、故障监控等功能               |
| [canbus-vehicle-zhongyun](modules/canbus_vehicle/zhongyun/README_cn.md)     | `canbus_vehicle/zhongyun`   | zhongyun车型组件包，负责zhongyun车型的报文解析、交互控制、故障监控等功能     |
