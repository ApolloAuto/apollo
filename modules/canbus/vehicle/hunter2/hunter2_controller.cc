/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus/vehicle/hunter2/hunter2_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/hunter2/hunter2_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "cyber/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace hunter2 {

using ::apollo::drivers::canbus::ProtocolData;
using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}

ErrorCode Hunter2Controller::Init(
	const VehicleParameter& params,
	CanSender<::apollo::canbus::ChassisDetail> *const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail> *const message_manager) {
  if (is_initialized_) {
    AINFO << "Hunter2Controller has already been initiated.";
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
  brake_control_command_131_ = dynamic_cast<Brakecontrolcommand131*>
          (message_manager_->GetMutableProtocolDataById(Brakecontrolcommand131::ID));
  if (brake_control_command_131_ == nullptr) {
     AERROR << "Brakecontrolcommand131 does not exist in the Hunter2MessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  control_mode_setting_421_ = dynamic_cast<Controlmodesetting421*>
          (message_manager_->GetMutableProtocolDataById(Controlmodesetting421::ID));
  if (control_mode_setting_421_ == nullptr) {
     AERROR << "Controlmodesetting421 does not exist in the Hunter2MessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  motion_control_instruction_111_ = dynamic_cast<Motioncontrolinstruction111*>
          (message_manager_->GetMutableProtocolDataById(Motioncontrolinstruction111::ID));
  if (motion_control_instruction_111_ == nullptr) {
     AERROR << "Motioncontrolinstruction111 does not exist in the Hunter2MessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  status_setting_441_ = dynamic_cast<Statussetting441*>
          (message_manager_->GetMutableProtocolDataById(Statussetting441::ID));
  if (status_setting_441_ == nullptr) {
     AERROR << "Statussetting441 does not exist in the Hunter2MessageManager!";
     return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Brakecontrolcommand131::ID, brake_control_command_131_, false);
  can_sender_->AddMessage(Controlmodesetting421::ID, control_mode_setting_421_, false);
  can_sender_->AddMessage(Motioncontrolinstruction111::ID, motion_control_instruction_111_, false);
  can_sender_->AddMessage(Statussetting441::ID, status_setting_441_, false);

  // need sleep to ensure all messages received
  AINFO << "Hunter2Controller is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

Hunter2Controller::~Hunter2Controller() {}

bool Hunter2Controller::Start() {
  if (!is_initialized_) {
    AERROR << "Hunter2Controller has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void Hunter2Controller::Stop() {
  if (!is_initialized_) {
    AERROR << "Hunter2Controller stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "Hunter2Controller stopped.";
  }
}

Chassis Hunter2Controller::chassis() {
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
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
  // 4 engine rpm ch has no engine rpm
  // chassis_.set_engine_rpm(0);
  // 5 ch has no wheel spd.
  if (chassis_detail.hunter2().has_motion_control_feedback_221() &&
      chassis_detail.hunter2().motion_control_feedback_221().has_speed()) {
    chassis_.set_speed_mps(
        static_cast<float>(chassis_detail.hunter2().motion_control_feedback_221().speed()));
  } else {
    chassis_.set_speed_mps(0);
  }
  // 6 hunter2 has no odometer
  // chassis_.set_odometer_m(0);
  // 7 hunter2 has no fuel. do not set;
  // chassis_.set_fuel_range_m(0);
  // 8 throttle
  // 9 brake
  // 10  gear
  // 11 steering
  if (chassis_detail.hunter2().has_motion_control_feedback_221() &&
      chassis_detail.hunter2().motion_control_feedback_221().has_steer()) {
    chassis_.set_steering_percentage(static_cast<float>(
        chassis_detail.hunter2().motion_control_feedback_221().steer() 
        *57.3* 100.0 /36.5));
  } else {
    chassis_.set_steering_percentage(0);
  }

//12 battry soc
if (chassis_detail.hunter2().has_bms_data_feedback_361() &&
      chassis_detail.hunter2().bms_data_feedback_361().has_bms_battery_soc()) {
    chassis_.set_battery_soc_percentage(
        chassis_detail.hunter2().bms_data_feedback_361().bms_battery_soc());
  } else {
    chassis_.set_battery_soc_percentage(0);
  }

  return chassis_;
}

void Hunter2Controller::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode Hunter2Controller::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  AINFO << "steer COMPLETE_AUTO_DRIVE mode";
  control_mode_setting_421_->set_control_mode_setting(
    Control_mode_setting_421::CONTROL_MODE_SETTING_CAN_COMMAND_CONTROL);

  AINFO << "\n\n\n set enable \n\n\n";
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
  return ErrorCode::OK;
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_enable();
  throttle_62_->set_enable();
  steering_64_->set_enable();

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
  */
}

ErrorCode Hunter2Controller::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode Hunter2Controller::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode.";
    return ErrorCode::OK;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_disable();
  throttle_62_->set_disable();
  steering_64_->set_enable();

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_STEER_ONLY);
  AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
  return ErrorCode::OK;
  */
  return ErrorCode::OK;
}

ErrorCode Hunter2Controller::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_enable();
  throttle_62_->set_enable();
  steering_64_->set_disable();

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_SPEED_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_SPEED_ONLY);
  AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
  return ErrorCode::OK;
  */
  return ErrorCode::OK;
}

// NEUTRAL, REVERSE, DRIVE
void Hunter2Controller::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      gear_66_->set_gear_neutral();
      break;
    }
    case Chassis::GEAR_REVERSE: {
      gear_66_->set_gear_reverse();
      break;
    }
    case Chassis::GEAR_DRIVE: {
      gear_66_->set_gear_drive();
      break;
    }
    case Chassis::GEAR_PARKING: {
      gear_66_->set_gear_park();
      break;
    }
    case Chassis::GEAR_LOW: {
      gear_66_->set_gear_low();
      break;
    }
    case Chassis::GEAR_NONE: {
      gear_66_->set_gear_none();
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      gear_66_->set_gear_none();
      break;
    }
    default: {
      gear_66_->set_gear_none();
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
void Hunter2Controller::Brake(double pedal) {
  // double real_value = params_.max_acc() * acceleration / 100;
  // TODO(All) :  Update brake value based on mode
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_pedal(pedal);
  */
 if(chassis_.speed_mps()<=0)
    motion_control_instruction_111_->set_speed_instruction(-1.0*pedal/100);

}

// drive with old acceleration
// gas:0.00~99.99 unit:
void Hunter2Controller::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  throttle_62_->set_pedal(pedal);
  */
  if(chassis_.speed_mps()>=0)
    motion_control_instruction_111_->set_speed_instruction(pedal/100);

}

// confirm the car is driven by acceleration command or throttle/brake pedal
// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void Hunter2Controller::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration."<<driving_mode();
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
}

// hunter2 default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void Hunter2Controller::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle = 36.5/57.3 * angle / 100.0;
  // reverse sign
  // ADD YOUR OWN CAR CHASSIS OPERATION
  motion_control_instruction_111_->set_steer_instruction(real_angle);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void Hunter2Controller::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  // ADD YOUR OWN CAR CHASSIS OPERATION
  const double real_angle = 36.5/57.3 * angle / 100.0;
  motion_control_instruction_111_->set_steer_instruction(real_angle);
}

void Hunter2Controller::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void Hunter2Controller::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void Hunter2Controller::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void Hunter2Controller::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  auto signal = command.signal().turn_signal();
  if (signal == Signal::TURN_LEFT) {
    turnsignal_68_->set_turn_left();
  } else if (signal == Signal::TURN_RIGHT) {
    turnsignal_68_->set_turn_right();
  } else {
    turnsignal_68_->set_turn_none();
  }
  */
}

void Hunter2Controller::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool Hunter2Controller::CheckChassisError() {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  if (!chassis_detail.has_hunter2()) {
    AERROR_EVERY(100) << "ChassisDetail has NO hunter2 vehicle info."
                      << chassis_detail.DebugString();
    return false;
  }
  return false;
}

void Hunter2Controller::SecurityDogThreadFunc() {
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
      AERROR
          << "Too much time consumption in Hunter2Controller looping process:"
          << elapsed.count();
    }
  }
}

bool Hunter2Controller::CheckResponse(const int32_t flags, bool need_wait) {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
  return true;
}

void Hunter2Controller::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t Hunter2Controller::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode Hunter2Controller::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void Hunter2Controller::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace hunter2
}  // namespace canbus
}  // namespace apollo
