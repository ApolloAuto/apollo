// Copyright 2025 daohu527@gmail.com
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

//  Created Date: 2025-01-16
//  Author: daohu527

#include "modules/canbus_vehicle/ros/ros_controller.h"

#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/canbus_vehicle/ros/ros_message_manager.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace ros {

using ::apollo::common::ErrorCode;
using ::apollo::common::VehicleSignal;
using ::apollo::control::ControlCommand;


ErrorCode RosController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::Ros>* const can_sender,
    MessageManager<::apollo::canbus::Ros>* const message_manager) {
  if (is_initialized_) {
    AINFO << "RosController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

  vehicle_params_.CopyFrom(
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param());

  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  twist_cmd_ = dynamic_cast<TwistCmd*>(
      message_manager_->GetMutableProtocolDataById(TwistCmd::ID));
  if (twist_cmd_ == nullptr) {
    AERROR << "TwistCmd does not exist in the ROSMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(TwistCmd::ID, twist_cmd_, false);

  // need sleep to ensure all messages received
  AINFO << "RosController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

RosController::~RosController() {}

bool RosController::Start() {
  if (!is_initialized_) {
    AERROR << "RosController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void RosController::Stop() {
  if (!is_initialized_) {
    AERROR << "RosController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "RosController stopped.";
  }
}

Chassis RosController::chassis() {
  chassis_.Clear();

  Ros chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // if (chassis_detail.has_twist_fb() &&
  //     chassis_detail.twist_fb().has_x_speed()) {
  //   chassis_.set_x_speed(
  //       static_cast<float>(chassis_detail.twist_fb().x_speed()));
  // }

  // if (chassis_detail.has_twist_fb() &&
  //     chassis_detail.twist_fb().has_y_speed()) {
  //   chassis_.set_y_speed(
  //       static_cast<float>(chassis_detail.twist_fb().y_speed()));
  // }

  // if (chassis_detail.has_twist_fb() &&
  //     chassis_detail.twist_fb().has_z_angle_speed()) {
  //   chassis_.set_z_angle_speed(
  //       static_cast<float>(chassis_detail.twist_fb().z_angle_speed()));
  // }

  // if (chassis_detail.has_acc_fb() &&
  //     chassis_detail.acc_fb().has_acceleration_x()) {
  //   chassis_.set_acceleration_x(
  //       static_cast<float>(chassis_detail.acc_fb().acceleration_x()));
  // }

  // if (chassis_detail.has_acc_fb() &&
  //     chassis_detail.acc_fb().has_acceleration_y()) {
  //   chassis_.set_acceleration_y(
  //       static_cast<float>(chassis_detail.acc_fb().acceleration_y()));
  // }

  // if (chassis_detail.has_acc_fb() &&
  //     chassis_detail.acc_fb().has_acceleration_z()) {
  //   chassis_.set_acceleration_z(
  //       static_cast<float>(chassis_detail.acc_fb().acceleration_z()));
  // }

  // if (chassis_detail.has_acc_fb() &&
  //     chassis_detail.acc_fb().has_angular_velocity_x()) {
  //   chassis_.set_angular_velocity_x(
  //       static_cast<float>(chassis_detail.acc_fb().angular_velocity_x()));
  // }

  // if (chassis_detail.has_ang_vel_fb() &&
  //     chassis_detail.ang_vel_fb().has_angular_velocity_y()) {
  //   chassis_.set_angular_velocity_y(
  //       chassis_detail.ang_vel_fb().angular_velocity_y());
  // }

  // if (chassis_detail.has_ang_vel_fb() &&
  //     chassis_detail.ang_vel_fb().has_angular_velocity_z()) {
  //   chassis_.set_angular_velocity_z(
  //       chassis_detail.ang_vel_fb().angular_velocity_z());
  // }

  // if (chassis_detail.has_battery_voltage() &&
  //     chassis_detail.ang_vel_fb().has_battery_voltage()) {
  //   chassis_.set_battery_voltage(chassis_detail.ang_vel_fb().battery_voltage());
  // }

  return chassis_;
}

void RosController::Emergency() {}

ErrorCode RosController::EnableAutoMode() { return ErrorCode::OK; }

ErrorCode RosController::DisableAutoMode() { return ErrorCode::OK; }

ErrorCode RosController::EnableSteeringOnlyMode() { return ErrorCode::OK; }

ErrorCode RosController::EnableSpeedOnlyMode() { return ErrorCode::OK; }

// NEUTRAL, REVERSE, DRIVE
void RosController::Gear(Chassis::GearPosition gear_position) {}

void RosController::Brake(double pedal) {}

void RosController::Throttle(double pedal) {}

void RosController::Acceleration(double acc) {}

// ros default, -30 ~ 00, left:+, right:-
// need to be compatible with control module, so reverse
// steering with default angle speed, 25-250 (default:250)
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void RosController::Steer(double angle) {}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:25~250, unit:deg/s
void RosController::Steer(double angle, double angle_spd) {}

void RosController::SetSpeed(double speed) {
  twist_cmd_->set_x_target_speed(speed);
}

void RosController::SetAngularSpeed(double angular_speed) {
  twist_cmd_->set_angular_velocity_z(angular_speed);
}

void RosController::SetEpbBreak(const ControlCommand& command) {}

ErrorCode RosController::HandleCustomOperation(
  const external_command::ChassisCommand& command) {
return ErrorCode::OK;
}

void RosController::SetBeam(const VehicleSignal& vehicle_signal) {
  if (vehicle_signal.high_beam()) {
    // None
  } else if (vehicle_signal.low_beam()) {
    // None
  }
}

void RosController::SetHorn(const VehicleSignal& vehicle_signal) {
  if (vehicle_signal.horn()) {
    // None
  } else {
    // None
  }
}

void RosController::SetTurningSignal(const VehicleSignal& vehicle_signal) {
  // Set Turn Signal
  auto signal = vehicle_signal.turn_signal();
  if (signal == common::VehicleSignal::TURN_LEFT) {
    // None
  } else if (signal == common::VehicleSignal::TURN_RIGHT) {
    // None
  } else {
    // None
  }
}

bool RosController::VerifyID() { return true; }

void RosController::ResetProtocol() { message_manager_->ResetSendMessages(); }

bool RosController::CheckChassisError() {
  Ros chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // TODO(zero): check fault_code
  if (!chassis_detail.has_twist_fb()) {
    AERROR_EVERY(100) << "ChassisDetail has no ROS vehicle info."
                      << chassis_detail.DebugString();
    return false;
  }

  return false;
}

void RosController::SecurityDogThreadFunc() {
  if (can_sender_ == nullptr) {
    AERROR << "Failed to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = ::apollo::cyber::Time::Now().ToMicrosecond();
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // chassis error check
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR);
      emergency_mode = true;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      message_manager_->ResetSendMessages();
    }
    end = ::apollo::cyber::Time::Now().ToMicrosecond();
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR << "Too much time consumption in RosController looping process:"
             << elapsed.count();
    }
  }
}

Chassis::ErrorCode RosController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void RosController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace ros
}  // namespace canbus
}  // namespace apollo
