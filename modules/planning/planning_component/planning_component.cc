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
#include "modules/planning/planning_component/planning_component.h" // 定义了这个组件的所有功能
// 包含 Cyber 框架和 Apollo 公共模块的配置，用于处理 gflags 配置参数
#include "cyber/common/file.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/config_gflags.h"
// 导入工具函数和 HDMap 相关功能，用于读取地图信息和处理消息
#include "modules/common/util/message_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
// 包含规划模块的通用组件，如历史记录 (history)、规划上下文 (planning_context)
#include "modules/planning/planning_base/common/history.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/util/util.h"
// 包含两种规划模式的实现
#include "modules/planning/planning_component/navi_planning.h"  // 导航模式
#include "modules/planning/planning_component/on_lane_planning.h"  // 基于车道的规划模式
namespace apollo {
namespace planning {

using apollo::cyber::ComponentBase;
using apollo::hdmap::HDMapUtil;
using apollo::perception::TrafficLightDetection;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingRequest;
using apollo::routing::RoutingResponse;
using apollo::storytelling::Stories;
/// @brief 初始化组件
/// @return 
bool PlanningComponent::Init() {
// 依赖注入器，本质就是一个数据缓存中心，以便于规划任务前后帧之间的承接，以及异常处理的回溯
// 创建 DependencyInjector 对象，用于依赖注入
  injector_ = std::make_shared<DependencyInjector>();
// 规划模式的选择: apollo/modules/common/configs/config_gflags.cc
  if (FLAGS_use_navigation_mode) {   // 默认是false
    planning_base_ = std::make_unique<NaviPlanning>(injector_); // 相对地图规划器
  } else {
    planning_base_ = std::make_unique<OnLanePlanning>(injector_); // 默认规划器
  }
// 加载config文件  proto
// ACHECK 检查配置文件是否成功加载，配置文件包含路径规划的相关参数
// 配置文件 planning_config.pb.txt
  ACHECK(ComponentBase::GetProtoConfig(&config_))
      << "failed to load planning config file "
      << ComponentBase::ConfigFilePath();
// 消息处理初始化:
// 初始化消息处理模块，如果启用了离线学习或非零学习模式
  if (FLAGS_planning_offline_learning ||
      config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    if (!message_process_.Init(config_, injector_)) {
      AERROR << "failed to init MessageProcess";
      return false;
    }
  }
// 在这里执行的是OnLanePlanning::Init的初始化
  planning_base_->Init(config_); // 参考线优化主要在这部分
// 订阅ROS2话题
// 订阅 PlanningCommand 话题，收到消息后存入 planning_command_
  planning_command_reader_ = node_->CreateReader<PlanningCommand>(
      config_.topic_config().planning_command_topic(),
      [this](const std::shared_ptr<PlanningCommand>& planning_command) {
        AINFO << "Received planning data: run planning callback."
              << planning_command->header().DebugString();
        std::lock_guard<std::mutex> lock(mutex_);
        planning_command_.CopyFrom(*planning_command);
      });
// 订阅交通信号灯检测数据，存入 traffic_light_
  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      config_.topic_config().traffic_light_detection_topic(),
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        ADEBUG << "Received traffic light data: run traffic light callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        traffic_light_.CopyFrom(*traffic_light);
      });
// 订阅 PadMessage（手动输入指令）
  pad_msg_reader_ = node_->CreateReader<PadMessage>(
      config_.topic_config().planning_pad_topic(),
      [this](const std::shared_ptr<PadMessage>& pad_msg) {
        ADEBUG << "Received pad data: run pad callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        pad_msg_.CopyFrom(*pad_msg);
      });

  story_telling_reader_ = node_->CreateReader<Stories>(
      config_.topic_config().story_telling_topic(),
      [this](const std::shared_ptr<Stories>& stories) {
        ADEBUG << "Received story_telling data: run story_telling callback.";
        std::lock_guard<std::mutex> lock(mutex_);
        stories_.CopyFrom(*stories);
      });
// 如果启用了导航模式，则订阅 relative_map（相对地图）
  if (FLAGS_use_navigation_mode) {
    relative_map_reader_ = node_->CreateReader<MapMsg>(
        config_.topic_config().relative_map_topic(),
        [this](const std::shared_ptr<MapMsg>& map_message) {
          ADEBUG << "Received relative map data: run relative map callback.";
          std::lock_guard<std::mutex> lock(mutex_);
          relative_map_.CopyFrom(*map_message);
        });
  }
  // 创建消息的发布者（Writer）
// planning发布话题部分: 话题也是写在配置中
// 发布规划路径给control模块
  planning_writer_ = node_->CreateWriter<ADCTrajectory>(
      config_.topic_config().planning_trajectory_topic());

  rerouting_client_ =
      node_->CreateClient<apollo::external_command::LaneFollowCommand,
                          external_command::CommandStatus>(
          config_.topic_config().routing_request_topic());
  planning_learning_data_writer_ = node_->CreateWriter<PlanningLearningData>(
      config_.topic_config().planning_learning_data_topic());
  command_status_writer_ = node_->CreateWriter<external_command::CommandStatus>(
      FLAGS_planning_command_status);
  return true;
}
/// @brief 检查数据，并且执行注册好的Planning，生成路线并且发布
/// @param prediction_obstacles 预测模块
/// @param chassis 来自总线CanBus,主要是汽车底盘所给出的机械状态信息
/// @param localization_estimate 定位信息
/// @return 
bool PlanningComponent::Proc(
    const std::shared_ptr<prediction::PredictionObstacles>&  //   // modules/common_msgs/prediction_msgs/prediction_obstacle.proto
        prediction_obstacles,
    const std::shared_ptr<canbus::Chassis>& chassis,
    const std::shared_ptr<localization::LocalizationEstimate>&
        localization_estimate) {
  ACHECK(prediction_obstacles != nullptr);

  // check and process possible rerouting request
  // 1.检查是否需要重新规划线路
  CheckRerouting();

  // process fused input data
  // 2.数据放入local_view_中，并且检查输入数据
  // std::mutex: 互斥锁，用于保护共享资源的访问
  // 当多个线程需要同时访问共享资源时，为了避免出现数据竞争等问题，需要使用互斥锁进行同步控制
  // 存储传感器数据，用于规划计算
  local_view_.prediction_obstacles = prediction_obstacles;
  local_view_.chassis = chassis;
  local_view_.localization_estimate = localization_estimate;
  {
    std::lock_guard<std::mutex> lock(mutex_);  // 互斥锁
    if (!local_view_.planning_command ||
        !common::util::IsProtoEqual(local_view_.planning_command->header(),
                                    planning_command_.header())) {
      local_view_.planning_command =
          std::make_shared<PlanningCommand>(planning_command_);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.traffic_light =
        std::make_shared<TrafficLightDetection>(traffic_light_);
    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!local_view_.pad_msg ||
        !common::util::IsProtoEqual(local_view_.pad_msg->header(),
                                    pad_msg_.header())) {
      // Check if "CLEAR_PLANNING" PadMessage is received and process.
      if (pad_msg_.action() == PadMessage::CLEAR_PLANNING) {
        local_view_.planning_command = nullptr;
        planning_command_.Clear();
      }
      local_view_.pad_msg = std::make_shared<PadMessage>(pad_msg_);
    }
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    local_view_.stories = std::make_shared<Stories>(stories_);
  }
// 3.0 
// 输入性检查，如果检查失败，则退出
  if (!CheckInput()) {
    AINFO << "Input check failed";
    return false;
  }
// 4.0 
// 如果配置文件为深度学习模式，非主流方法，跳过
  if (config_.learning_mode() != PlanningConfig::NO_LEARNING) {
    // data process for online training
    message_process_.OnChassis(*local_view_.chassis);
    message_process_.OnPrediction(*local_view_.prediction_obstacles);
    if (local_view_.planning_command->has_lane_follow_command()) {
      message_process_.OnRoutingResponse(
          local_view_.planning_command->lane_follow_command());
    }
    message_process_.OnStoryTelling(*local_view_.stories);
    message_process_.OnTrafficLightDetection(*local_view_.traffic_light);
    message_process_.OnLocalization(*local_view_.localization_estimate);
  }
// 5.0
  // publish learning data frame for RL test
  // 非主流
  if (config_.learning_mode() == PlanningConfig::RL_TEST) {
    PlanningLearningData planning_learning_data;
    LearningDataFrame* learning_data_frame =
        injector_->learning_based_data()->GetLatestLearningDataFrame();
    if (learning_data_frame) {
      planning_learning_data.mutable_learning_data_frame()->CopyFrom(
          *learning_data_frame);
      common::util::FillHeader(node_->Name(), &planning_learning_data);
      planning_learning_data_writer_->Write(planning_learning_data);
    } else {
      AERROR << "fail to generate learning data frame";
      return false;
    }
    return true;
  }
// 6.0  生成并发布轨迹
// adc_trajectory_pb为最终获取的路径信息
  ADCTrajectory adc_trajectory_pb;
  // 在这里配置的是on_lane_planning方法，路径规划函数入口
  // planning逻辑主循环
  // 输入:local_view_   输出:adc_trajectory_pb  轨迹
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
    // 7.0 此时路径规划已经完成，获取路径规划计算最开始的时间戳，参看RunOnce时间戳赋值
  // 获取计算完成后的时间戳，参看FillHeader时间戳赋值
  auto start_time = adc_trajectory_pb.header().timestamp_sec(); // 从 adc_trajectory_pb 中获取起始时间（start_time），即该消息头中的时间戳
  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

  // modify trajectory relative time due to the timestamp change in header
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }
   // 8.0 
  // 发布路径信息
  planning_writer_->Write(adc_trajectory_pb);

  // Send command execution feedback.
  // Error occured while executing the command.
  external_command::CommandStatus command_status;
  common::util::FillHeader(node_->Name(), &command_status);
  if (nullptr != local_view_.planning_command) {
    command_status.set_command_id(local_view_.planning_command->command_id());
  }

  ADCTrajectory::TrajectoryType current_trajectory_type =
      adc_trajectory_pb.trajectory_type();
  if (adc_trajectory_pb.header().status().error_code() !=
      common::ErrorCode::OK) {
    command_status.set_status(external_command::CommandStatusType::ERROR);
    command_status.set_message(adc_trajectory_pb.header().status().msg());
  } else if (planning_base_->IsPlanningFinished(current_trajectory_type)) {
    AINFO << "Set the external_command: FINISHED";
    command_status.set_status(external_command::CommandStatusType::FINISHED);
  } else {
    AINFO << "Set the external_command: RUNNING";
    command_status.set_status(external_command::CommandStatusType::RUNNING);
  }
  command_status_writer_->Write(command_status);

  // record in history
  // 记录历史轨迹信息
  // record in history
  // 将轨迹添加到历史记录
  auto* history = injector_->history();
  history->Add(adc_trajectory_pb);

  return true;
}
/// @brief 检查是否需要进行路径重新规划。如果需要，填充相关命令并发送请求，然后将标志位need_rerouting置为false
void PlanningComponent::CheckRerouting() {
  auto* rerouting = injector_->planning_context()
                        ->mutable_planning_status()
                        ->mutable_rerouting();
  if (!rerouting->need_rerouting()) {
    return;
  }
  common::util::FillHeader(node_->Name(),
                           rerouting->mutable_lane_follow_command());
  auto lane_follow_command_ptr =
      std::make_shared<apollo::external_command::LaneFollowCommand>(
          rerouting->lane_follow_command());
  rerouting_client_->SendRequest(lane_follow_command_ptr);
  rerouting->set_need_rerouting(false);
}
/// @brief 检查输入数据是否完整，如定位数据、底盘数据、地图数据是否准备好。如果数据不完整，则跳过当前的规划周期
/// @return 
bool PlanningComponent::CheckInput() {
  ADCTrajectory trajectory_pb;
  auto* not_ready = trajectory_pb.mutable_decision()
                        ->mutable_main_decision()
                        ->mutable_not_ready();
// 检查定位，底盘，地图数据
  if (local_view_.localization_estimate == nullptr) {
    not_ready->set_reason("localization not ready");
  } else if (local_view_.chassis == nullptr) {
    not_ready->set_reason("chassis not ready");
  } else if (HDMapUtil::BaseMapPtr() == nullptr) {
    not_ready->set_reason("map not ready");
  } else {
    // nothing
  }

  if (FLAGS_use_navigation_mode) {
    if (!local_view_.relative_map->has_header()) {
      not_ready->set_reason("relative map not ready");
    }
  } else {
    if (!local_view_.planning_command ||
        !local_view_.planning_command->has_header()) {
      not_ready->set_reason("planning_command not ready");
    }
  }

  if (not_ready->has_reason()) {
    AINFO << not_ready->reason() << "; skip the planning cycle.";
    common::util::FillHeader(node_->Name(), &trajectory_pb);
    planning_writer_->Write(trajectory_pb);
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
