# old-routing-adpter

## 介绍

`old-routing-adpter`是对9.0之前的版本中RoutingRequest进行适配兼容的模块。新的版本点到点沿车道线行驶的命令使用`apollo::external_command::LaneFollowCommand`，用户最少只需要指定终点的位置和朝向，即可发送点到点的导航命令。为了兼容旧版本的`apollo::routing::RoutingRequest`命令，需要`old-routing-adpter`模块订阅此topic，再将RoutingRequest转换成新的`apollo::external_command::LaneFollowCommand`命令，使用`apollo::cyber::Client`发送给外部命令处理模块`apollo::external_command::ExternalCommandProcessComponent`。

![](./docs/images/old_routing_adapter.png)

## 目录结构

```shell

modules/external_command/old_routing_adapter/
├── old_routing_adapter
    ├── conf                    // 参数配置文件
    ├── dag                     // 模块启动文件(mainboard)
    ├── docs                    // 说明文档
    ├── launch                  // 模块启动文件(cyber_launch)
    ├── proto                   // 公共（全局）参数配置结构定义
    ├── BUILD                   // 构建规则文件
    ├── cyberfile.xml           // 包管理配置文件
    ├── old_routing_adapter.cc  // routing适配器源码
    ├── old_routing_adapter.h   // routing适配器头文件
    └── README_cn.md            // 说明文档
```

## 模块

### OldRoutingAdapter

apollo::planning::OldRoutingAdapter

#### 输入

`old-routing-adpter`模块的输入为旧版`apollo::routing::RoutingRequest`。

| Channel 名                             | 类型                                          | <div style="width: 300pt">描述</div>             |
| -------------------------------------- | --------------------------------------------- | ------------------------------------------------ |
| `/apollo/routing_request`                     | `apollo::routing::RoutingRequest`             | 旧版RoutingRequest命令，包含起点和终点的位姿，以及对应的Lane信息等；如果是泊车命令，包含停车位的相关信息 |

#### 输出

`old-routing-adpter`模块将旧版`apollo::routing::RoutingRequest`转换成新的外部命令接口，通过`apollo::cyber::Client`发送：

| client 名                         | Request类型                                         | Response类型                                         |<div style="width: 300pt">描述</div>                                                                                                               |
| ---------------------------------- | -------------------------------------------- | -------------------------------------------- |-------------------------------------------------------------------------------------------------------------------------------------------------- |
| `/apollo/external_command/lane_follow`               | `apollo::external_command::LaneFollowCommand`   | `apollo::external_command::CommandStatus` | 包含点到点命令的RoutingRequest转换成的外部命令 |
| `/apollo/external_command/valet_parking`               | `apollo::external_command::ValetParkingCommand`   | `apollo::external_command::CommandStatus` | 包含泊车命令的RoutingRequest转换成的外部命令 |

#### 配置

| 文件路径                                                                     | 类型/结构                                       | <div style="width: 300pt">说明</div> |
| ---------------------------------------------------------------------------- | ----------------------------------------------- | ------------------------------------ |
| `modules/external_command/old_routing_adapter/conf/config.pb.txt`                 | `apollo::planning::PlanningConfig`              | 配置文件，包含输入输出的channel或服务名称  |

#### 使用方式

##### 使用 mainboard 启动

```shell
mainboard -d modules/external_command/old_routing_adapter/dag/old_routing_adapter.dag
```

##### 使用 cyber_launch 启动

```shell
cyber_launch start modules/external_command/old_routing_adapter/launch/old_routing_adapter.launch
```
