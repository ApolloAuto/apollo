/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/canbus/vehicle/gem/gem_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "modules/canbus/vehicle/gem/gem_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace gem {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}  // namespace

ErrorCode GemController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "GemController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

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

  brake_cmd_6b_ = dynamic_cast<Brakecmd6b*>(
      message_manager_->GetMutableProtocolDataById(Brakecmd6b::ID));
  if (brake_cmd_6b_ == nullptr) {
    AERROR << "Brakecmd6b does not exist in the GemMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  accel_cmd_67_ = dynamic_cast<Accelcmd67*>(
      message_manager_->GetMutableProtocolDataById(Accelcmd67::ID));
  if (accel_cmd_67_ == nullptr) {
    AERROR << "Accelcmd67 does not exist in the GemMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  steering_cmd_6d_ = dynamic_cast<Steeringcmd6d*>(
      message_manager_->GetMutableProtocolDataById(Steeringcmd6d::ID));
  if (steering_cmd_6d_ == nullptr) {
    AERROR << "Steeringcmd6d does not exist in the GemMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  shift_cmd_65_ = dynamic_cast<Shiftcmd65*>(
      message_manager_->GetMutableProtocolDataById(Shiftcmd65::ID));
  if (shift_cmd_65_ == nullptr) {
    AERROR << "Shiftcmd65 does not exist in the GemMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }
  turn_cmd_63_ = dynamic_cast<Turncmd63*>(
      message_manager_->GetMutableProtocolDataById(Turncmd63::ID));
  if (turn_cmd_63_ == nullptr) {
    AERROR << "Turncmd63 does not exist in the GemMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Brakecmd6b::ID, brake_cmd_6b_, false);
  can_sender_->AddMessage(Accelcmd67::ID, accel_cmd_67_, false);
  can_sender_->AddMessage(Steeringcmd6d::ID, steering_cmd_6d_, false);
  can_sender_->AddMessage(Shiftcmd65::ID, shift_cmd_65_, false);
  can_sender_->AddMessage(Turncmd63::ID, turn_cmd_63_, false);

  // need sleep to ensure all messages received
  AINFO << "GemController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

GemController::~GemController() {}

bool GemController::Start() {
  if (!is_initialized_) {
    AERROR << "GemController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void GemController::Stop() {
  if (!is_initialized_) {
    AERROR << "GemController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "GemController stopped.";
  }
}

Chassis GemController::chassis() {
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

  // 5
  if (chassis_detail.gem().has_vehicle_speed_rpt_6f() &&
      chassis_detail.gem().vehicle_speed_rpt_6f().has_vehicle_speed()) {
    chassis_.set_speed_mps(
        chassis_detail.gem().vehicle_speed_rpt_6f().vehicle_speed());
  } else {
    chassis_.set_speed_mps(0);
  }

  // 7
  chassis_.set_fuel_range_m(0);
  // 8
  if (chassis_detail.gem().has_accel_cmd_67() &&
      chassis_detail.gem().accel_cmd_67().has_accel_cmd()) {
    chassis_.set_throttle_percentage(
        chassis_detail.gem().accel_cmd_67().accel_cmd());
  } else {
    chassis_.set_throttle_percentage(0);
  }
  // 9
  if (chassis_detail.gem().has_brake_rpt_6c() &&
      chassis_detail.gem().brake_rpt_6c().has_output_value()) {
    chassis_.set_brake_percentage(
        chassis_detail.gem().brake_rpt_6c().output_value());
  } else {
    chassis_.set_brake_percentage(0);
  }

  // 23, previously 10
  // TODO(QiL): refine here.
  /*
  if (chassis_detail.gem().has_shift_rpt_66() &&
      chassis_detail.gem().shift_rpt_66().has_output_value()) {
    chassis_.set_gear_location(
        chassis_detail.gem().shift_rpt_66().output_value());
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }
  */

  // 11
  // TODO(QiL): verify the unit here.
  if (chassis_detail.gem().has_steering_rpt_1_6e() &&
      chassis_detail.gem().steering_rpt_1_6e().has_output_value()) {
    chassis_.set_steering_percentage(
        chassis_detail.gem().steering_rpt_1_6e().output_value() * 100.0 /
        params_.max_steer_angle() / M_PI * 180);
  } else {
    chassis_.set_steering_percentage(0);
  }

  // TODO(QiL): implement the turn light signal here
  /*
    // 16, 17
    if (chassis_detail.has_light() &&
        chassis_detail.light().has_turn_light_type() &&
        chassis_detail.light().turn_light_type() != Light::TURN_LIGHT_OFF) {
      if (chassis_detail.light().turn_light_type() == Light::TURN_LEFT_ON) {
        chassis_.mutable_signal()->set_turn_signal(
            common::VehicleSignal::TURN_LEFT);
      } else if (chassis_detail.light().turn_light_type() ==
                 Light::TURN_RIGHT_ON) {
        chassis_.mutable_signal()->set_turn_signal(
            common::VehicleSignal::TURN_RIGHT);
      } else {
        chassis_.mutable_signal()->set_turn_signal(
            common::VehicleSignal::TURN_NONE);
      }
    } else {
      chassis_.mutable_signal()->set_turn_signal(
          common::VehicleSignal::TURN_NONE);
    }
  */
  // TODO(all): implement the rest here/
  // 26
  if (chassis_error_mask_) {
    chassis_.set_chassis_error_mask(chassis_error_mask_);
  }

  // give engage_advice based on error_code and canbus feedback
  if (!chassis_error_mask_ && !chassis_.parking_brake() &&
      (chassis_.throttle_percentage() != 0.0) &&
      (chassis_.brake_percentage() != 0.0)) {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::READY_TO_ENGAGE);
  } else {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    chassis_.mutable_engage_advice()->set_reason(
        "CANBUS not ready, firmware error or emergency button pressed!");
  }

  return chassis_;
}

void GemController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
  set_chassis_error_code(Chassis::CHASSIS_ERROR);
}

ErrorCode GemController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;

  /*
    brake_cmd_6b_->set_enable();
    accel_cmd_67_->set_enable();
    steering_cmd_6d_->set_enable();
    shift_cmd_65_->set_enable();
    turn_cmd_63_->set_enable();
  */

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
    AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
    return ErrorCode::OK;
  }
}

ErrorCode GemController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode GemController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode";
    return ErrorCode::OK;
  }

  /*
    brake_cmd_6b_->set_disable();
    accel_cmd_67_->set_disable();
    steering_cmd_6d_->set_enable();
    shift_cmd_65_->set_enable();
  */

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
    return ErrorCode::OK;
  }
}

ErrorCode GemController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;

  /*
    brake_cmd_6b_->set_enable();
    accel_cmd_67_->set_enable();
    steering_cmd_6d_->set_disable();
    shift_cmd_65_->set_enable();
  */

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
    return ErrorCode::OK;
  }
}

// NEUTRAL, REVERSE, DRIVE
void GemController::Gear(Chassis::GearPosition gear_position) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "this drive mode no need to set gear.";
    return;
  }

  /*
    switch (gear_position) {
      case Chassis::GEAR_NEUTRAL: {
        shift_cmd_65_->set_gear_neutral();
        break;
      }
      case Chassis::GEAR_REVERSE: {
        shift_cmd_65_->set_gear_reverse();
        break;
      }
      case Chassis::GEAR_DRIVE: {
        shift_cmd_65_->set_gear_drive();
        break;
      }
      case Chassis::GEAR_PARKING: {
        shift_cmd_65_->set_gear_park();
        break;
      }
      case Chassis::GEAR_LOW: {
        shift_cmd_65_->set_gear_low();
        break;
      }
      case Chassis::GEAR_NONE: {
        shift_cmd_65_->set_gear_none();
        break;
      }
      case Chassis::GEAR_INVALID: {
        AERROR << "Gear command is invalid!";
        shift_cmd_65_->set_gear_none();
        break;
      }
      default: {
        shift_cmd_65_->set_gear_none();
        break;
      }
    }
    */
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void GemController::Brake(double pedal) {
  // double real_value = params_.max_acc() * acceleration / 100;
  // TODO(QiL) Update brake value based on mode
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  /*
  brake_cmd_6b_->set_pedal(pedal);
  */
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void GemController::Throttle(double pedal) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  /*
  accel_cmd_67_->set_pedal(pedal);
  */
}

// gem default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void GemController::Steer(double angle) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle = params_.max_steer_angle() * angle / 100.0;
  // reverse sign
  /*
    steering_cmd_6d_->set_steering_angle(real_angle)
        ->set_steering_angle_speed(200);
        */
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void GemController::Steer(double angle, double angle_spd) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle = params_.max_steer_angle() * angle / 100.0;
  /*
  const double real_angle_spd = (
      params_.min_steer_angle_spd(), params_.max_steer_angle_spd(),
      params_.max_steer_angle_spd() * angle_spd / 100.0);
  steering_cmd_6d_->set_steering_angle(real_angle)
      ->set_steering_angle_speed(real_angle_spd);
      */
}

void GemController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void GemController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void GemController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void GemController::SetTurningSignal(const ControlCommand& command) {
  // TODO(QiL) : implement it here
}

void GemController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool GemController::CheckChassisError() {
  // TODO(QiL) : implement it here
  return false;
}

void GemController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Fail to run SecurityDogThreadFunc() because can_sender_ is "
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
    start = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
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
        CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false) == false) {
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
    end = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR << "Too much time consumption in GemController looping process:"
             << elapsed.count();
    }
  }
}

bool GemController::CheckResponse(const int32_t flags, bool need_wait) {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
   */
  return false;
}

void GemController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t GemController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode GemController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void GemController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace gem
}  // namespace canbus
}  // namespace apollo
