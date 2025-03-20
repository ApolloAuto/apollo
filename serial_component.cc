// Copyright 2025 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Created Date: 2025-01-16
// Author: daohu527

#include "modules/serial/serial_component.h"

#include "cyber/common/file.h"
#include "modules/serial/vehicle_factory.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

namespace apollo {
namespace serial {

SerialComponent::SerialComponent() {}

SerialComponent::~SerialComponent() { Clear(); }

bool SerialComponent::Init() {
  if (!GetProtoConfig(&serial_conf_)) {
    AINFO << "Unable to load serial conf file: " << ConfigFilePath();
    return false;
  }
  AINFO << "Serial conf loaded: " << serial_conf_.ShortDebugString();

  // 创建车辆控制对象
  control_ = VehicleFactory::CreateControl(serial_conf_);
  if (control_ == nullptr) {
    AINFO << "Failed to create control object.";
    return false;
  }

  // 创建控制命令 Reader
  cyber::ReaderConfig control_cmd_reader_config;
  control_cmd_reader_config.channel_name = FLAGS_control_command_topic;
  control_command_reader_ = node_->CreateReader<ControlCommand>(
      control_cmd_reader_config,
      [this](const std::shared_ptr<ControlCommand>& cmd) {
        AINFO << "Received ControlCommand in cyber_monitor: speed=" << cmd->speed()
               << ", steering_rate=" << cmd->steering_rate();
        OnControlCommand(*cmd);
      });

  // 创建 chassis Writer
  chassis_writer_ = node_->CreateWriter<Chassis>(FLAGS_chassis_topic);
  if (chassis_writer_ == nullptr) {
    AINFO << "Failed to create chassis writer.";
    return false;
  }

  // 启动车辆控制模块
  control_->Start();
  AINFO << "SerialComponent initialized successfully.";
  return true;
}

void SerialComponent::Clear() {
  if (control_ != nullptr) {
    control_->Stop();
    AINFO << "Control stopped.";
  }
}

bool SerialComponent::Proc() {
  // 从车辆控制对象获取 chassis 数据
  Chassis chassis = control_->GetChassis();
  // 填充 header 信息，便于追踪消息来源和时间
  common::util::FillHeader(node_->Name(), &chassis);

  // 输出 chassis 中关键信息的调试日志，便于在 monitor 中检查
  AINFO << "Publishing Chassis data in cyber_monitor: "
         << "Speed(mps)=" << chassis.speed_mps() 
         << ", BatterySOC(%)=" << chassis.battery_soc_percentage();

  // 发布 chassis 消息到指定 topic
  chassis_writer_->Write(chassis);
  return true;
}

void SerialComponent::OnControlCommand(const ControlCommand& control_command) {
  AINFO << "Processing ControlCommand in OnControlCommand for cyber_monitor: "
         << "Speed=" << control_command.speed()
         << ", SteeringRate=" << control_command.steering_rate();
  // 将控制命令发送到车辆控制模块，内部会进行编码、串口传输等操作
  control_->Send(control_command);
}

std::string SerialComponent::Name() const {
  return "SerialComponent";
}

}  // namespace serial
}  // namespace apollo