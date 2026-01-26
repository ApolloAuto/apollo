## PNC 外部接口调用方法概述

在新版本中对 PNC 接口进行了优化和升级，对 PNC 内部接口统一梳理和封装，当 PNC 内部模块接口升级时，可以保持外部命令接口不变，用户可自定义 HMI，任务管理模块等调用外部接口按需求定制发送命令至 PNC 内部，完成功能实现。

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_262ffef.png)

## PNC 接口调用流程

外部接口采用 service/client 调用方法，在调用外部接口时，Client 端负责将命令发送到 Service 端，Service 端会解析命令，执行相应的业务逻辑等，在处理完命令后传递给 PNC 相应的组件进行执行，并返回给 Client 状态值。

![image (1).png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image%20%281%29_5ee22ed.png)

调用 PNC 接口需要首先根据需求功能确定命令，进行 Client_Wrapper/Client 端注册，填写功能实现所需的命令字段信息，最后发送给 Service 端进行命令处理，以 LaneFollowCommand 命令为例详细过程如下所示：

### 步骤一：注册Client_Wrapper/Client端

开发者首先需要选择一类client注册，负责接口信息的发送,apollo提供两种client类型供开发者选择

1. client_wrapper(推荐)：向 Service 发送命令，并记录发送的命令信息，可在 cyber_monitor 中查看。

2. client：只向 Service 发送命令。

#### 注册Client_Wrapper端

```bash
// 注册Node节点
std::shared_ptr<apollo::cyber::Node> node_ =
      apollo::cyber::CreateNode("demo");
// 注册client发送Command命令，CommandName需要根据相应Command确定
std::shared_ptr<
      apollo::common::ClientWrapper<apollo::external_command::LaneFollowCommand,
                                    apollo::external_command::CommandStatus>>
      client_wrapper;
// service_name需要根据相应的service端设置确定
client_wrapper = std::make_shared<apollo::common::ClientWrapper<
      apollo::external_command::LaneFollowCommand, CommandStatus>>(
      node_, service_name);
```

> 注意：Client端与Service端为对应关系,service_name需通过查询Service与Client的配置文件确定

#### 注册Client端

```bash
// 注册Node节点
std::shared_ptr<apollo::cyber::Node> node =
      apollo::cyber::CreateNode("demo");
// 注册client发送Command命令，CommandName需要根据相应Command确定
std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::LaneFollowCommand,
                            apollo::external_command::CommandStatus>>
      client;
// service_name需要根据相应的service端设置确定
client = node_->CreateClient<apollo::external_command::LaneFollowCommand,
                          CommandStatus>(service_name);
```

> 注意：Client 端与 Service 端为对应关系，service_name 需通过查询 Service 与 Client 的配置文件确定
> 完成以上步骤即完成 Client 端注册及配置。

### 步骤二：填写接口命令信息

接口命令含有不同功能字段，用户通过对具体字段的修改来进行自定义功能的实现，开发者首先需要根据需求填写相应功能字段，完善接口信息内容，举例如下：

```bash
// 定义Command类型
  auto command =
      std::make_shared<apollo::external_command::LaneFollowCommand>();
  // 根据需求自定义Header信息注入
  FillCommandHeader(command);
  // 设置way_point为(0, 0),end_pose为(10, 0), 采用默认车辆当前位置为起始点
  // 自定义way_point
  apollo::external_command::Pose way_point;
  way_point.set_x(0.0);
  way_point.set_y(0.0);
  way_point.set_heading(0.0);
  std::vector<apollo::external_command::Pose> way_points;
  way_points.emplace_back(way_point);
  // 自定义end Pose
  apollo::external_command::Pose end_pose;
        end_pose.set_x(10.0);
        end_pose.set_y(0.0);
        end_pose.set_heading(0.0);
  // Command注入way_point信息
  for (const auto& point : way_points) {
    auto way_point = command->add_way_point();
    way_point->CopyFrom(point);
  }
  // Command注入end_pose信息
  command->mutable_end_pose()->CopyFrom(end);
  // Command注入targe_speed信息
  double target_speed = 10.0;
  command->set_target_speed(target_speed);
  // .......其他信息
  // ....
  // ....
```

完成以上步骤即完成具体接口命令的填写。

### 步骤三：Client_Wrapper/Client端发命令至Service端

在 LaneFollowCommand 命令填写完成后，Client_Wrapper/Client 端提供以下接口发送指令，可根据需求发送具体的命令格式，举例如下：

#### Client_Wrapper端

```cpp
// 发送Command指令至service端
std::cout << "Sending command: " << command->DebugString()
<< std::endl;
// Client_Wrapper/Client端发送命令接口相同
auto response = client_wrapper->SendRequest(command);
if (nullptr == response) {
std::cout << "Command sending failed, please check the service is on!\n"
<< std::endl;
} else {
std::cout << "**\*\***Finish sending command.**\*\***\n" << std::endl;
}
```

> 注意：常见问题：若 SendRequest 后长时间无响应，考虑 Service 端开启失败。

#### Client端

```bash
// 发送Command指令至service端
  std::cout << "Sending command: " << command->DebugString()
            << std::endl;
  // Client_Wrapper/Client端发送命令接口相同
  auto response = client->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Command sending failed, please check the service is on!\n"
              << std::endl;
  } else {
    std::cout << "******Finish sending command.******\n" << std::endl;
  }
```

> 注意：常见问题：若SendRequest 后长时间无响应，考虑Service端开启失败。

完成以上步骤后即完成LaneFollowCommand命令发送至service端

### 步骤四：查询client发送命令后状态

完成命令发送后,若需对指定命令的状态进行查询,需要发送查询命令,示例如下

#### 1. 注册Client_Wrapper/Client端

- 注册Client_Wrapper端
  
  ```bash
  // 注册Node节点
  std::shared_ptr<apollo::cyber::Node> node_ =
      apollo::cyber::CreateNode("demo");
  // 注册client发送CommandStatusRequest命令
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::CommandStatusRequest,
                            apollo::external_command::CommandStatus>>
      status_wrapper_client_;
  // service_name需要根据相应的service端设置确定
  status_wrapper_client_ = std::make_shared<apollo::common::ClientWrapper<
      apollo::external_command::CommandStatusRequest, CommandStatus>>(
      node_, service_name);
  ```
  
  > 注意：Client 端与 Service 端为对应关系，service_name 需通过查询 Service 与 Client 的配置文件确定。

- 注册 Client 端
  
  ```bash
  // 注册Node节点
  std::shared_ptr<apollo::cyber::Node> node_ =
      apollo::cyber::CreateNode("demo");
  // 注册client发送CommandStatusRequest命令
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::CommandStatusRequest,
                            apollo::external_command::CommandStatus>>
      status_client_;
  // service_name需要根据相应的service端设置确定
  client_ = node_->CreateClient<apollo::external_command::CommandStatusRequest,
                          CommandStatus>(service_name);
  ```
  
  > 注意：Client 端与 Service 端为对应关系，service_name 需通过查询 Service 与 Client 的配置文件确定
  > 完成以上步骤即完成 Client 端注册及配置。

#### 2. 填写接口命令信息

```bash
 // 定义Command类型
  auto command =
      std::make_shared<apollo::external_command::CommandStatusRequest>();
  // 根据需求自定义Header信息注入
  FillCommandHeader(command);
 // 注入需要查询的CommandId
 command->set_command_id(command_id);
```

#### 3. Client_Wrapper/Client端发命令至Service端

- Client_Wrapper端
  
  ```bash
  // 发送Command指令至service端
  std::cout << "Sending check command: " << command->DebugString() << std::endl;
  auto response = status_wrapper_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Check status failed!\n" << std::endl;
  } else {
    std::cout << "******Finish checking command.******\n" << std::endl;
  }
  ```
  
  > 注意：常见问题，若 SendRequest 后长时间无响应，考虑 Service 端开启失败。

- Client 端
  
  ```bash
  // 发送Command指令至service端
  std::cout << "Sending check command: " << command->DebugString() << std::endl;
  auto response = status_client_->SendRequest(command);
  if (nullptr == response) {
    std::cout << "Check status failed!\n" << std::endl;
  } else {
    std::cout << "******Finish checking command.******\n" << std::endl;
  }
  ```

## PNC 命令列表及功能介绍

![image.png](https://bce.bdstatic.com/doc/Apollo-Homepage-Document/Apollo_Beta_Doc/image_6b6e953.png)

PNC 模块是一种用于自动驾驶系统的模块，可以通过接收不同的命令来执行不同的功能。用户需发送LaneFollowCommand，ValetParkingCommand 等命令指定 PNC 模块完成道路行驶、泊车等功能，Apollo 提供的具体命令及功能列表如下。

<table><thead>
  <tr>
    <th colspan="2">命令名称</th>
    <th>层级</th>
    <th>功能</th>
  </tr></thead>
<tbody>
  <tr>
    <td colspan="2">LaneFollowCommand</td>
    <td>core</td>
    <td>点到点沿道路行驶</td>
  </tr>
  <tr>
    <td colspan="2">ValetParkingCommand</td>
    <td>core</td>
    <td>泊车</td>
  </tr>
  <tr>
    <td rowspan="6">ActionCommand</td>
    <td>PULL_OVER</td>
    <td rowspan="6">core</td>
    <td>紧急靠边停车</td>
  </tr>
  <tr>
    <td>STOP</td>
    <td>紧急停车</td>
  </tr>
  <tr>
    <td>START</td>
    <td>继续行驶</td>
  </tr>
  <tr>
    <td>SWITCH_TO_MANUAL</td>
    <td>切换到手动模式</td>
  </tr>
  <tr>
    <td>SWITCH_TO_AUTO</td>
    <td>切换到自动模式</td>
  </tr>
  <tr>
    <td>VIN_REQ</td>
    <td>vin code验证</td>
  </tr>
  <tr>
    <td colspan="2">ChassisCommand</td>
    <td>园区</td>
    <td>底盘控制命令</td>
  </tr>
  <tr>
    <td colspan="2">FreeSpaceCommand</td>
    <td>园区</td>
    <td>非结构化道路行驶</td>
  </tr>
  <tr>
    <td colspan="2">PathFollowCommand</td>
    <td>园区</td>
    <td>循迹行驶</td>
  </tr>
  <tr>
    <td colspan="2">SpeedCommand</td>
    <td>园区</td>
    <td>速度限制命令</td>
  </tr>
</tbody></table>

### 1.LaneFollowCommand接口命令及功能说明

```bash
message LaneFollowCommand {
  optional apollo.common.Header header = 1;
  // Unique identification for command.
  optional int64 command_id = 2 [default = -1];
  // If the start pose is set as the first point of "way_point".
  optional bool is_start_pose_set = 3 [default = false];
  // The points between "start_pose" and "end_pose".
  repeated Pose way_point = 4;
  // End pose of the lane follow command, must be given.
  required Pose end_pose = 5;
  // The lane segments which should not be passed by.
  repeated LaneSegment blacklisted_lane = 6;
  // The road which should not be passed by.
  repeated string blacklisted_road = 7;
  // Expected speed when executing this command. If "target_speed" > maximum
  // speed of the vehicle, use maximum speed of the vehicle instead. If it is
  // not given, the default target speed of system will be used.
  optional double target_speed = 8;
}
```

LaneFollowCommand 命令适用于结构化道路点到点沿道路行驶的场景。

接口说明：

1. header 为 message 础信息，可调 apollo::common::util::FillHeader 方法注入信息。
2. command_id 命令特征标识。
3. is_start_pose_set 是否设定起点标识，若为 true，则以第一个路由点作为起点，否则以车辆当前位置作为起点。
4. way_point 指定途径点。
5. end_pose 指定终止点。
6. blacklisted_lane 指定不可通行的 lanesegment 列表。
7. blacklisted_lane 指定不可通行的 road 列表。
8. target_speed 指定巡航速度。

### 2.ValetParkingCommand接口命令及功能说明

```bash
message ValetParkingCommand {
  optional apollo.common.Header header = 1;
  // Unique identification for command.
  optional int64 command_id = 2 [default = -1];
  // If the start pose is set as the first point of "way_point".
  optional bool is_start_pose_set = 3 [default = false];
  // The points between "start_pose" and "end_pose".
  repeated Pose way_point = 4;
  // The lane segments which should not be passed by.
  repeated LaneSegment blacklisted_lane = 5;
  // The road which should not be passed by.
  repeated string blacklisted_road = 6;
  // The id of the parking spot on the map.
  required string parking_spot_id = 7;
  // Expected speed when executing this command. If "target_speed" > maximum
  // speed of the vehicle, use maximum speed of the vehicle instead. If it is
  // not given, the default target speed of system will be used.
  optional double target_speed = 8;
}
```

ValetParkingCommand命令适用于需要泊车功能的场景。

接口说明：

1. header 为 message 基础信息，可调 apollo::common::util::FillHeader 方法注入信息。
2. command_id 命令特征标识。
3. is_start_pose_set 是否设定起点标识，若为 true，则以第一个路由点作为起点，否则以车辆当前位置作为起点。
4. way_point 指定途径点。
5. blacklisted_lane 指定不可通行的 lanesegment 列表。
6. blacklisted_lane 指定不可通行的 road 列表。
7. parking_spot_id 指定停车位编号。
8. target_speed 指定巡航速度。

### 3. ActionCommand接口命令及功能说明

```bash
message ActionCommand {
  optional apollo.common.Header header = 1;
  // Unique identification for command.
  optional int64 command_id = 2 [default = -1];
  // The action command.
  required ActionCommandType command = 3;
}
```

ActionCommand命令适用于直接控制车辆状态的场景

接口说明：

1. header 为 message 基础信息，可调 apollo::common::util::FillHeader 方法注入信息。

2. command_id 命令特征标识。

3. ActionCommandType 动作命令。
   
   a. PULL_OVER 紧急靠边停车。
   
   b. START 继续行驶。
   
   c. SWITCH_TO_MANUAL 切换到手动模式。
   
   d. SWITCH_TO_AUTO 切换到自动模式。
   
   e. VIN_REQ vin code 验证。

### 4.ChassisCommand接口命令及功能说明

```bash
message ChassisCommand {
  optional apollo.common.Header header = 1;
  // Unique identification for command.
  optional int64 command_id = 2 [default = -1];
  // The basic vehicle signals which can also be controlled by apollo system.
  optional apollo.common.VehicleSignal basic_signal = 3;
  // Custom chassis operation command defined by user for extensibility.
  optional google.protobuf.Any custom_operation = 4;
}
```

ActionCommand 命令适用于直接控制车辆状态的场景。

接口说明：

1. header 为 message 基础信息，可调 apollo::common::util::FillHeader 方法注入信息。
2. command_id 命令特征标识。
3. VehicleSignal 基本车辆信号、如喇叭、灯光。
4. custom_operation 用户自定义底盘命令，根据实际需要结合CANBUS设置。

### 5. FreeSpaceCommand接口命令及功能说明

```bash
message FreeSpaceCommand {
  optional apollo.common.Header header = 1;
  // Unique identification for command.
  optional int64 command_id = 2 [default = -1];
  // Pose of the parking spot.
  required Pose parking_spot_pose = 3;
  // Region where openspace trajectory will be searched. Junction containing
  // "non_drivable_roi" should be contained by "drivable_roi"
  // polygon points should be clockwise if outer polygon can drive.
  // otherwise polygon points should be counter-clockwise if inner polygon can drive
  repeated RoiPolygon non_drivable_roi = 4;
  required RoiPolygon drivable_roi = 5;
  // Expected speed when executing this command. If "target_speed" > maximum
  // speed of the vehicle, use maximum speed of the vehicle instead. If it is
  // not given, the default target speed of system will be used.
  optional double target_speed = 6;
}
```

FreeSpaceCommand命令适用于非结构化道路车辆行驶的场景

接口说明：

1. header 为 message 基础信息，可调 apollo::common::util::FillHeader 方法注入信息。
2. command_id 命令特征标识。
3. parking_spot_pose 停止点。
4. non_drivable_roi 不可行驶区域角点集，需要顺指针给出行驶区域角点，并且需要包含于可行驶区域之内。
5. drivable_roi 可行驶区域角点集，需要逆指针给出行驶区域角点，可行驶区域角点集要求必须给出。
6. target_speed 指定巡航速度。

### 6.PathFollowCommand接口命令及功能说明

```bash
message PathFollowCommand {
  optional apollo.common.Header header = 1;
  // Unique identification for command.
  optional int64 command_id = 2 [default = -1];
  // Path point to be followed, a valid path should contain >= 2 points. No lane
  // on the map is followed for this command.
  repeated Point way_point = 3;
  oneof boundary {
    // PathBoundary with left and right boundary.
    PathBoundary path_boundary = 4;
    // Path boundary generated with the distance from center to left and right
    // boundary given.
    BoundaryWithWidth boundary_with_width = 5;
  }
  // Expected speed when executing this command. If "target_speed" > maximum
  // speed of the vehicle, use maximum speed of the vehicle instead. If it is
  // not given, the default target speed of system will be used.
  optional double target_speed = 6;
}
```

PathFollowCommand 命令适用于无图由用户沿给定路线行驶的场景。

接口说明：

header命令消息头，可调 apollo::common::util::FillHeader 方法注入信息。

- command_id 命令唯一标识。

- way_point 为给定路线上的坐标点。

- boundary 为车辆横向可行驶区域，有两种设置方式可选：
  
  - path_boundary：为每个路径点设置左右边界
  - boundary_with_width：对所有路径点设置统一的左右边界宽度
  
  如果没有设置 boundary，默认参考车辆宽度作为左右边界，在这种情况下车辆遇到障碍物不能绕障，只能停止。

### 7. SpeedCommand接口命令及功能说明

```bash
message SpeedCommand {
  optional apollo.common.Header header = 1;
  // Unique identification for command.
  optional int64 command_id = 2 [default = -1];
  oneof linear_speed {
    // Replace the target speed of current motion command with this new target
    // speed.
    double target_speed = 3;
    // Multiple the target speed in current motion command with the factor. The
    // factor should be in range [0, 1.0].
    double target_speed_factor = 4;
    // Restore the default cruise speed.
    bool is_restore_default_cruise_speed = 5 [default = false];
  }
}
```

SpeedCommand命令适用于对速度进行附加限制的的场景。

接口说明：

1. header 为 message 基础信息，可调 apollo::common::util::FillHeader 方法注入信息。

2. command_id 命令特征标识。

3. linear_speed 速度。
   
   a. target_speed 巡航速度，该速度会覆盖之前设置的巡航速度。
   
   b. target_speed_factor 巡航速度系数，将巡航速度替换为原巡航速度\*系数。
   
   c. is_restore_default_cruise_speed 布尔量，当不设置 target_speed/target_speed_factor 时默认为 false，恢复默认巡航速度。
