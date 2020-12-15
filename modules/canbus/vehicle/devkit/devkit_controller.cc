/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/devkit/devkit_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus/vehicle/devkit/devkit_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}  // namespace

ErrorCode DevkitController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "DevkitController has already been initiated.";
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
  brake_command_101_ = dynamic_cast<Brakecommand101*>(
      message_manager_->GetMutableProtocolDataById(Brakecommand101::ID));
  if (brake_command_101_ == nullptr) {
    AERROR << "Brakecommand101 does not exist in the DevkitMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  gear_command_103_ = dynamic_cast<Gearcommand103*>(
      message_manager_->GetMutableProtocolDataById(Gearcommand103::ID));
  if (gear_command_103_ == nullptr) {
    AERROR << "Gearcommand103 does not exist in the DevkitMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  park_command_104_ = dynamic_cast<Parkcommand104*>(
      message_manager_->GetMutableProtocolDataById(Parkcommand104::ID));
  if (park_command_104_ == nullptr) {
    AERROR << "Parkcommand104 does not exist in the DevkitMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  steering_command_102_ = dynamic_cast<Steeringcommand102*>(
      message_manager_->GetMutableProtocolDataById(Steeringcommand102::ID));
  if (steering_command_102_ == nullptr) {
    AERROR << "Steeringcommand102 does not exist in the DevkitMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  throttle_command_100_ = dynamic_cast<Throttlecommand100*>(
      message_manager_->GetMutableProtocolDataById(Throttlecommand100::ID));
  if (throttle_command_100_ == nullptr) {
    AERROR << "Throttlecommand100 does not exist in the DevkitMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Brakecommand101::ID, brake_command_101_, false);
  can_sender_->AddMessage(Gearcommand103::ID, gear_command_103_, false);
  can_sender_->AddMessage(Parkcommand104::ID, park_command_104_, false);
  can_sender_->AddMessage(Steeringcommand102::ID, steering_command_102_, false);
  can_sender_->AddMessage(Throttlecommand100::ID, throttle_command_100_, false);

  // need sleep to ensure all messages received
  AINFO << "DevkitController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

DevkitController::~DevkitController() {}

bool DevkitController::Start() {
  if (!is_initialized_) {
    AERROR << "DevkitController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void DevkitController::Stop() {
  if (!is_initialized_) {
    AERROR << "DevkitController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "DevkitController stopped.";
  }
}

Chassis DevkitController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);
  // 4 engine rpm ch has no engine rpm
  // chassis_.set_engine_rpm(0);
  // 5 wheel spd
  if (chassis_detail.devkit().has_wheelspeed_report_506()) {
    if (chassis_detail.devkit().wheelspeed_report_506().has_rr()) {
      chassis_.mutable_wheel_speed()->set_wheel_spd_rr(
          chassis_detail.devkit().wheelspeed_report_506().rr());
    }
    if (chassis_detail.devkit().wheelspeed_report_506().has_rl()) {
      chassis_.mutable_wheel_speed()->set_wheel_spd_rl(
          chassis_detail.devkit().wheelspeed_report_506().rl());
    }
    if (chassis_detail.devkit().wheelspeed_report_506().has_fr()) {
      chassis_.mutable_wheel_speed()->set_wheel_spd_fr(
          chassis_detail.devkit().wheelspeed_report_506().fr());
    }
    if (chassis_detail.devkit().wheelspeed_report_506().has_fl()) {
      chassis_.mutable_wheel_speed()->set_wheel_spd_fl(
          chassis_detail.devkit().wheelspeed_report_506().fl());
    }
  }
  // 6 speed_mps
  if (chassis_detail.devkit().has_vcu_report_505() &&
      chassis_detail.devkit().vcu_report_505().has_speed()) {
    chassis_.set_speed_mps(
        static_cast<float>(chassis_detail.devkit().vcu_report_505().speed()));
  } else {
    chassis_.set_speed_mps(0);
  }
  // 7 no odometer
  // chassis_.set_odometer_m(0);
  // 8 no fuel. do not set;
  // chassis_.set_fuel_range_m(0);
  // 9 throttle
  if (chassis_detail.devkit().has_throttle_report_500() &&
      chassis_detail.devkit()
          .throttle_report_500()
          .has_throttle_pedal_actual()) {
    chassis_.set_throttle_percentage(static_cast<float>(
        chassis_detail.devkit().throttle_report_500().throttle_pedal_actual()));
  } else {
    chassis_.set_throttle_percentage(0);
  }
  // 10 brake
  if (chassis_detail.devkit().has_brake_report_501() &&
      chassis_detail.devkit().brake_report_501().has_brake_pedal_actual()) {
    chassis_.set_brake_percentage(static_cast<float>(
        chassis_detail.devkit().brake_report_501().brake_pedal_actual()));
  } else {
    chassis_.set_brake_percentage(0);
  }
  // 23, previously 11 gear
  if (chassis_detail.devkit().has_gear_report_503() &&
      chassis_detail.devkit().gear_report_503().has_gear_actual()) {
    Chassis::GearPosition gear_pos = Chassis::GEAR_INVALID;

    if (chassis_detail.devkit().gear_report_503().gear_actual() ==
        Gear_report_503::GEAR_ACTUAL_INVALID) {
      gear_pos = Chassis::GEAR_INVALID;
    }
    if (chassis_detail.devkit().gear_report_503().gear_actual() ==
        Gear_report_503::GEAR_ACTUAL_NEUTRAL) {
      gear_pos = Chassis::GEAR_NEUTRAL;
    }
    if (chassis_detail.devkit().gear_report_503().gear_actual() ==
        Gear_report_503::GEAR_ACTUAL_REVERSE) {
      gear_pos = Chassis::GEAR_REVERSE;
    }
    if (chassis_detail.devkit().gear_report_503().gear_actual() ==
        Gear_report_503::GEAR_ACTUAL_DRIVE) {
      gear_pos = Chassis::GEAR_DRIVE;
    }
    if (chassis_detail.devkit().gear_report_503().gear_actual() ==
        Gear_report_503::GEAR_ACTUAL_PARK) {
      gear_pos = Chassis::GEAR_PARKING;
    }
    chassis_.set_gear_location(gear_pos);
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }
  // 12 steering
  if (chassis_detail.devkit().has_steering_report_502() &&
      chassis_detail.devkit().steering_report_502().has_steer_angle_actual()) {
    chassis_.set_steering_percentage(static_cast<float>(
        chassis_detail.devkit().steering_report_502().steer_angle_actual() *
        100.0 / vehicle_params_.max_steer_angle() * M_PI / 180));
  } else {
    chassis_.set_steering_percentage(0);
  }
  // 13 parking brake
  if (chassis_detail.devkit().has_park_report_504() &&
      chassis_detail.devkit().park_report_504().has_parking_actual()) {
    if (chassis_detail.devkit().park_report_504().parking_actual() ==
        Park_report_504::PARKING_ACTUAL_PARKING_TRIGGER) {
      chassis_.set_parking_brake(true);
    } else {
      chassis_.set_parking_brake(false);
    }
  } else {
    chassis_.set_parking_brake(false);
  }
  // 14 battery soc
  if (chassis_detail.devkit().has_bms_report_512() &&
      chassis_detail.devkit().bms_report_512().has_battery_soc()) {
    chassis_.set_battery_soc_percentage(
        chassis_detail.devkit().bms_report_512().battery_soc());
  } else {
    chassis_.set_battery_soc_percentage(0);
  }

  return chassis_;
}

void DevkitController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode DevkitController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  // set enable
  brake_command_101_->set_brake_en_ctrl(
      Brake_command_101::BRAKE_EN_CTRL_ENABLE);
  throttle_command_100_->set_throttle_en_ctrl(
      Throttle_command_100::THROTTLE_EN_CTRL_ENABLE);
  steering_command_102_->set_steer_en_ctrl(
      Steering_command_102::STEER_EN_CTRL_ENABLE);
  gear_command_103_->set_gear_en_ctrl(Gear_command_103::GEAR_EN_CTRL_ENABLE);
  park_command_104_->set_park_en_ctrl(Park_command_104::PARK_EN_CTRL_ENABLE);

  // set AEB enable
  if (FLAGS_enable_aeb) {
    brake_command_101_->set_aeb_en_ctrl(
        Brake_command_101::AEB_EN_CTRL_ENABLE_AEB);
  }

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
  return ErrorCode::OK;
}

ErrorCode DevkitController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode DevkitController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode.";
    return ErrorCode::OK;
  }
  AFATAL << "SpeedOnlyMode is not supported in devkit!";
  return ErrorCode::CANBUS_ERROR;
}

ErrorCode DevkitController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  AFATAL << "SpeedOnlyMode is not supported in devkit!";
  return ErrorCode::CANBUS_ERROR;
}

// NEUTRAL, REVERSE, DRIVE
void DevkitController::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }
  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      gear_command_103_->set_gear_target(Gear_command_103::GEAR_TARGET_NEUTRAL);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      gear_command_103_->set_gear_target(Gear_command_103::GEAR_TARGET_REVERSE);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      gear_command_103_->set_gear_target(Gear_command_103::GEAR_TARGET_DRIVE);
      break;
    }
    case Chassis::GEAR_PARKING: {
      gear_command_103_->set_gear_target(Gear_command_103::GEAR_TARGET_PARK);
      break;
    }
    case Chassis::GEAR_INVALID: {
      gear_command_103_->set_gear_target(Gear_command_103::GEAR_TARGET_NEUTRAL);
      break;
    }
    default: {
      gear_command_103_->set_gear_target(Gear_command_103::GEAR_TARGET_NEUTRAL);
      break;
    }
  }
}

// brake with pedal
// pedal:0.00~99.99, unit:%
void DevkitController::Brake(double pedal) {
  // double real_value = params_.max_acc() * acceleration / 100;
  // TODO(All) :  Update brake value based on mode
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
  brake_command_101_->set_brake_pedal_target(pedal);
}

// drive with pedal
// pedal:0.0~99.9 unit:%
void DevkitController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  throttle_command_100_->set_throttle_pedal_target(pedal);
}

// confirm the car is driven by acceleration command instead of throttle/brake
// pedal drive with acceleration/deceleration acc:-7.0 ~ 5.0, unit:m/s^2
void DevkitController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  // None
}

// devkit default, -30 ~ 00, left:+, right:-
// need to be compatible with control module, so reverse
// steering with default angle speed, 25-250 (default:250)
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void DevkitController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  steering_command_102_->set_steer_angle_target(real_angle)
      ->set_steer_angle_spd(250);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:25~250, unit:deg/s
void DevkitController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  steering_command_102_->set_steer_angle_target(real_angle)
      ->set_steer_angle_spd(250);
}

void DevkitController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void DevkitController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void DevkitController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void DevkitController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal do not support on devkit
}

void DevkitController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool DevkitController::CheckChassisError() {
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  if (!chassis_detail.has_devkit()) {
    AERROR_EVERY(100) << "ChassisDetail has no devkit vehicle info."
                      << chassis_detail.DebugString();
    return false;
  }

  Devkit devkit = chassis_detail.devkit();

  // steer fault
  if (devkit.has_steering_report_502()) {
    if (Steering_report_502::STEER_FLT1_STEER_SYSTEM_HARDWARE_FAULT ==
        devkit.steering_report_502().steer_flt1()) {
      return true;
    }
  }
  // drive fault
  if (devkit.has_throttle_report_500()) {
    if (Throttle_report_500::THROTTLE_FLT1_DRIVE_SYSTEM_HARDWARE_FAULT ==
        devkit.throttle_report_500().throttle_flt1()) {
      return true;
    }
  }
  // brake fault
  if (devkit.has_brake_report_501()) {
    if (Brake_report_501::BRAKE_FLT1_BRAKE_SYSTEM_HARDWARE_FAULT ==
        devkit.brake_report_501().brake_flt1()) {
      return true;
    }
  }

  return false;
}

void DevkitController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

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

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      horizontal_ctrl_fail = 0;
    }

    // 2. vertical control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false)) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }
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
      AERROR << "Too much time consumption in DevkitController looping process:"
             << elapsed.count();
    }
  }
}

bool DevkitController::CheckResponse(const int32_t flags, bool need_wait) {
  int32_t retry_num = 20;
  ChassisDetail chassis_detail;
  bool is_eps_online = false;
  bool is_vcu_online = false;
  bool is_esp_online = false;

  do {
    if (message_manager_->GetSensorData(&chassis_detail) != ErrorCode::OK) {
      AERROR_EVERY(100) << "get chassis detail failed.";
      return false;
    }
    bool check_ok = true;
    if (flags & CHECK_RESPONSE_STEER_UNIT_FLAG) {
      is_eps_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_eps_online() &&
                      chassis_detail.check_response().is_eps_online();
      check_ok = check_ok && is_eps_online;
    }

    if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG) {
      is_vcu_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_vcu_online() &&
                      chassis_detail.check_response().is_vcu_online();
      is_esp_online = chassis_detail.has_check_response() &&
                      chassis_detail.check_response().has_is_esp_online() &&
                      chassis_detail.check_response().is_esp_online();
      check_ok = check_ok && is_vcu_online && is_esp_online;
    }
    if (check_ok) {
      return true;
    } else {
      AINFO << "Need to check response again.";
    }
    if (need_wait) {
      --retry_num;
      std::this_thread::sleep_for(
          std::chrono::duration<double, std::milli>(20));
    }
  } while (need_wait && retry_num);

  AINFO << "check_response fail: is_eps_online:" << is_eps_online
        << ", is_vcu_online:" << is_vcu_online
        << ", is_esp_online:" << is_esp_online;

  return false;
}

void DevkitController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t DevkitController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode DevkitController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void DevkitController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace devkit
}  // namespace canbus
}  // namespace apollo
