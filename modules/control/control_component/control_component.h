/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include <string>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/control_msgs/pad_msg.pb.h"
#include "modules/common_msgs/external_command_msgs/command_status.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"
#include "modules/control/control_component/proto/preprocessor.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/time/time.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/common/util/util.h"
#include "modules/control/control_component/controller_task_base/common/dependency_injector.h"
#include "modules/control/control_component/controller_task_base/control_task_agent.h"
#include "modules/control/control_component/submodules/preprocessor_submodule.h"

/**
 * @namespace apollo::control
 * @brief apollo::control
 */
namespace apollo {
namespace control {

/**
 * @class Control
 *
 * @brief control module main class, it processes localization, chassis, and
 * pad data to compute throttle, brake and steer values.
 */
class ControlComponent final : public apollo::cyber::TimerComponent {
  friend class ControlTestBase;

 public:
 // 构造函数
  ControlComponent();
 // 初始化函数
  bool Init() override;  // 初始化订阅+写入
 
 // 每周期执行函数
  bool Proc() override;

 private:
  // Upon receiving pad message
  // 处理驾驶员操作指令（如启停）
  void OnPad(const std::shared_ptr<PadMessage> &pad);

  // 处理底盘状态信息
  void OnChassis(const std::shared_ptr<apollo::canbus::Chassis> &chassis);

  // 接收规划模块输出的轨迹
  void OnPlanning(
      const std::shared_ptr<apollo::planning::ADCTrajectory> &trajectory);
  
  // 接收任务状态
  void OnPlanningCommandStatus(
      const std::shared_ptr<external_command::CommandStatus>
          &planning_command_status);
  
  // 接收定位信息
  void OnLocalization(
      const std::shared_ptr<apollo::localization::LocalizationEstimate>
          &localization);

  // 用于日志/系统监控
  // Upon receiving monitor message
  void OnMonitor(
      const apollo::common::monitor::MonitorMessage &monitor_message);
  
  // 计算控制命令（转向、加速、刹车）
  common::Status ProduceControlCommand(ControlCommand *control_command);
  // 检查输入是否完整或有效
  common::Status CheckInput(LocalView *local_view);
  // 时间戳对齐检查
  common::Status CheckTimestamp(const LocalView &local_view);
  common::Status CheckPad();
  // // 特殊情况输出零控制命令（如急停）
  void ResetAndProduceZeroControlCommand(ControlCommand *control_command);
  // 获取车辆俯仰角辅助控制
  void GetVehiclePitchAngle(ControlCommand *control_command);

 private:
  apollo::cyber::Time init_time_; // 控制器初始化时间

  localization::LocalizationEstimate latest_localization_; // 最新定位信息
  canbus::Chassis latest_chassis_;  // 最新底盘状态
  planning::ADCTrajectory latest_trajectory_; // 最新规划轨迹
  external_command::CommandStatus planning_command_status_; // 任务状态
  PadMessage pad_msg_;  // 驾驶员操作信息
  common::Header latest_replan_trajectory_header_;  // 最近的轨迹重规划头信息

  ControlTaskAgent control_task_agent_;  // 控制任务分发与执行

  bool estop_ = false; // 紧急停车标志
  std::string estop_reason_;   // 紧急停车原因
  bool pad_received_ = false;   // 是否收到 Pad 信息

  unsigned int status_lost_ = 0;
  unsigned int status_sanity_check_failed_ = 0;
  unsigned int total_status_lost_ = 0;
  unsigned int total_status_sanity_check_failed_ = 0;

  ControlPipeline control_pipeline_;   // 控制流水线（如预处理、主控制器）

  std::mutex mutex_;  // 互斥锁保护共享资源

  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<apollo::planning::ADCTrajectory>>
      trajectory_reader_;
  std::shared_ptr<cyber::Reader<apollo::external_command::CommandStatus>>
      planning_command_status_reader_;

  std::shared_ptr<cyber::Writer<ControlCommand>> control_cmd_writer_; // 控制命令发布器
  // when using control submodules
  std::shared_ptr<cyber::Writer<LocalView>> local_view_writer_;  // 用于子模块调试或结构解耦

  common::monitor::MonitorLogBuffer monitor_logger_buffer_; // 监控日志输出缓冲区

  LocalView local_view_;  // 本周期使用的数据快照

  std::shared_ptr<DependencyInjector> injector_;   // 控制器依赖注入器

  double previous_steering_command_ = 0.0;  // 上一次控制器的转向输出（用于滤波/约束）
};

CYBER_REGISTER_COMPONENT(ControlComponent)
}  // namespace control
}  // namespace apollo
