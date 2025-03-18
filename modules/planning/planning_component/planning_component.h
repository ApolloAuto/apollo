/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"  // 车辆底盘数据 (Chassis)
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/external_command_msgs/lane_follow_command.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"  // 车辆定位数据 (LocalizationEstimate)
#include "modules/common_msgs/perception_msgs/traffic_light_detection.pb.h"  // 交通信号灯检测数据 (TrafficLightDetection)
#include "modules/common_msgs/planning_msgs/pad_msg.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"  // 规划轨迹 (ADCTrajectory)
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/storytelling_msgs/story.pb.h"
#include "modules/planning/planning_base/proto/learning_data.pb.h"  //  机器学习数据 (PlanningLearningData)
#include "modules/planning/planning_base/proto/planning_config.pb.h" // 规划模块的参数配置 (PlanningConfig)

#include "cyber/class_loader/class_loader.h"  // 用于加载 Cyber 组件
#include "cyber/component/component.h" // 使 PlanningComponent 继承 cyber::Component，从而成为 Apollo Cyber RT 组件
#include "cyber/message/raw_message.h" // 处理原始消息
#include "modules/planning/planning_base/common/message_process.h"   // 消息处理逻辑
#include "modules/planning/planning_base/gflags/planning_gflags.h"   //  规划模块的全局标志变量
#include "modules/planning/planning_component/planning_base.h"       // 规划模块的基类

namespace apollo {
namespace planning {
/// @brief PlanningComponent 继承自 cyber::Component，表示它是 Apollo Cyber 组件，可以订阅和发布消息
// 预测模块的障碍物信息
// 车辆底盘信息
// 车辆定位信息
class PlanningComponent final
    : public cyber::Component<prediction::PredictionObstacles, canbus::Chassis,
                              localization::LocalizationEstimate> {
 public:
  PlanningComponent() = default;

  ~PlanningComponent() = default;

 public:
 // 在组件启动时调用，用于初始化 订阅者、发布者、参数配置 等
  bool Init() override;

  bool Proc(const std::shared_ptr<prediction::PredictionObstacles>&
                prediction_obstacles,
            const std::shared_ptr<canbus::Chassis>& chassis,
            const std::shared_ptr<localization::LocalizationEstimate>&
                localization_estimate) override;

 private:
 // 检查是否需要重新规划路径
  void CheckRerouting();
 // 检查输入数据是否有效
  bool CheckInput();

 private:
 // 订阅交通信号灯检测数据
  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
 // Apollo Cyber 服务客户端，用于向 rerouting 服务发送车道跟随命令并接收状态反馈
  std::shared_ptr<
      apollo::cyber::Client<apollo::external_command::LaneFollowCommand,
                            apollo::external_command::CommandStatus>>
      rerouting_client_;
 // 其他订阅者
  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;
  std::shared_ptr<cyber::Reader<storytelling::Stories>> story_telling_reader_;
  std::shared_ptr<cyber::Reader<PlanningCommand>> planning_command_reader_;

 // 发布者
 // 规划轨迹 (ADCTrajectory)
  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
 // 重新规划请求 (RoutingRequest)
  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;
 // 学习数据 (PlanningLearningData)
  std::shared_ptr<cyber::Writer<PlanningLearningData>>
      planning_learning_data_writer_;
 // 指令状态 (CommandStatus)
  std::shared_ptr<cyber::Writer<external_command::CommandStatus>>
      command_status_writer_;

  std::mutex mutex_;
  perception::TrafficLightDetection traffic_light_;
  routing::RoutingResponse routing_;
  planning::PadMessage pad_msg_;
  relative_map::MapMsg relative_map_;
  storytelling::Stories stories_;
  PlanningCommand planning_command_;
  LocalView local_view_;

  std::unique_ptr<PlanningBase> planning_base_;
  std::shared_ptr<DependencyInjector> injector_;

  PlanningConfig config_;
  MessageProcess message_process_;
};

// 组件注册
// CYBER_REGISTER_COMPONENT() 注册 PlanningComponent 组件，使其可以在 Apollo Cyber 框架 中运行
CYBER_REGISTER_COMPONENT(PlanningComponent)

}  // namespace planning
}  // namespace apollo
