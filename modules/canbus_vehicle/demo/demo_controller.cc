/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/canbus_vehicle/demo/demo_controller.h"

#include <string>

#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/canbus_vehicle/demo/demo_message_manager.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace demo {
using ::apollo::common::ErrorCode;
using ::apollo::common::VehicleSignal;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {
const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;

}  // namespace

void DemoController::AddSendMessage() {
  can_sender_->AddMessage(Brakecommand101::ID, brake_command_101_, false);
  can_sender_->AddMessage(Gearcommand103::ID, gear_command_103_, false);
  can_sender_->AddMessage(Parkcommand104::ID, park_command_104_, false);
  can_sender_->AddMessage(Steeringcommand102::ID, steering_command_102_, false);
  can_sender_->AddMessage(Throttlecommand100::ID, throttle_command_100_, false);
  can_sender_->AddMessage(Vehiclemodecommand105::ID, vehicle_mode_command_105_,
                          false);
}

ErrorCode DemoController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::Demo>* const can_sender,
    MessageManager<::apollo::canbus::Demo>* const message_manager) {
  if (is_initialized_) {
    AINFO << "DemoController has already been initiated.";
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
    AERROR << "Canbus sender is null.";
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
    AERROR << "Brakecommand101 does not exist in the DemoMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  gear_command_103_ = dynamic_cast<Gearcommand103*>(
      message_manager_->GetMutableProtocolDataById(Gearcommand103::ID));
  if (gear_command_103_ == nullptr) {
    AERROR << "Gearcommand103 does not exist in the DemoMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  park_command_104_ = dynamic_cast<Parkcommand104*>(
      message_manager_->GetMutableProtocolDataById(Parkcommand104::ID));
  if (park_command_104_ == nullptr) {
    AERROR << "Parkcommand104 does not exist in the DemoMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  steering_command_102_ = dynamic_cast<Steeringcommand102*>(
      message_manager_->GetMutableProtocolDataById(Steeringcommand102::ID));
  if (steering_command_102_ == nullptr) {
    AERROR << "Steeringcommand102 does not exist in the DemoMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  throttle_command_100_ = dynamic_cast<Throttlecommand100*>(
      message_manager_->GetMutableProtocolDataById(Throttlecommand100::ID));
  if (throttle_command_100_ == nullptr) {
    AERROR << "Throttlecommand100 does not exist in the DemoMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  vehicle_mode_command_105_ = dynamic_cast<Vehiclemodecommand105*>(
      message_manager_->GetMutableProtocolDataById(Vehiclemodecommand105::ID));
  if (vehicle_mode_command_105_ == nullptr) {
    AERROR << "Vehiclemodecommand105 does not exist in the DemoMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  AddSendMessage();

  AINFO << "DemoController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

DemoController::~DemoController() {}

bool DemoController::Start() {
  if (!is_initialized_) {
    AERROR << "DemoController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void DemoController::Stop() {
  if (!is_initialized_) {
    AERROR << "DemoController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "DemoController stopped.";
  }
}

Chassis DemoController::chassis() {
  chassis_.Clear();
  Demo chassis_detail = GetNewRecvChassisDetail();

  // 1, 2
  // if (driving_mode() == Chassis::EMERGENCY_MODE) {
  //   set_chassis_error_code(Chassis::NO_ERROR);
  // }
  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);

  // 4 chassis spd
  if (chassis_detail.has_vcu_report_505() &&
      chassis_detail.vcu_report_505().has_speed()) {
    chassis_.set_speed_mps(
        static_cast<float>(chassis_detail.vcu_report_505().speed()));
  } else {
    chassis_.set_speed_mps(0);
  }

  // 5 throttle
  if (chassis_detail.has_throttle_report_500() &&
      chassis_detail.throttle_report_500().has_throttle_pedal_actual()) {
    chassis_.set_throttle_percentage(static_cast<float>(
        chassis_detail.throttle_report_500().throttle_pedal_actual()));
  } else {
    chassis_.set_throttle_percentage(0);
  }

  // 6 brake
  if (chassis_detail.has_brake_report_501() &&
      chassis_detail.brake_report_501().has_brake_pedal_actual()) {
    chassis_.set_brake_percentage(static_cast<float>(
        chassis_detail.brake_report_501().brake_pedal_actual()));
  } else {
    chassis_.set_brake_percentage(0);
  }

  // 7 gear
  if (chassis_detail.has_gear_report_503() &&
      chassis_detail.gear_report_503().has_gear_actual()) {
    Chassis::GearPosition gear_pos = Chassis::GEAR_INVALID;

    if (chassis_detail.gear_report_503().gear_actual() ==
        Gear_report_503::GEAR_ACTUAL_NEUTRAL) {
      gear_pos = Chassis::GEAR_NEUTRAL;
    }
    if (chassis_detail.gear_report_503().gear_actual() ==
        Gear_report_503::GEAR_ACTUAL_REVERSE) {
      gear_pos = Chassis::GEAR_REVERSE;
    }
    if (chassis_detail.gear_report_503().gear_actual() ==
        Gear_report_503::GEAR_ACTUAL_DRIVE) {
      gear_pos = Chassis::GEAR_DRIVE;
    }
    if (chassis_detail.gear_report_503().gear_actual() ==
        Gear_report_503::GEAR_ACTUAL_PARK) {
      gear_pos = Chassis::GEAR_PARKING;
    }

    chassis_.set_gear_location(gear_pos);
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }

  // 8 steer
  if (chassis_detail.has_steering_report_502() &&
      chassis_detail.steering_report_502().has_steer_angle_actual()) {
    chassis_.set_steering_percentage(static_cast<float>(
        chassis_detail.steering_report_502().steer_angle_actual() * 100.0 /
        vehicle_params_.max_steer_angle()));
  } else {
    chassis_.set_steering_percentage(0);
  }

  // 9 checkresponse signal
  // 9 checkresponse signal
  // 9 checkresponse signal
  if (chassis_detail.has_brake_report_501() &&
      chassis_detail.brake_report_501().has_brake_en_state()) {
    chassis_.mutable_check_response()->set_is_esp_online(
        chassis_detail.brake_report_501().brake_en_state() == 1);
  }

  if (chassis_detail.has_steering_report_502() &&
      chassis_detail.steering_report_502().has_steer_en_state()) {
    chassis_.mutable_check_response()->set_is_eps_online(
        chassis_detail.steering_report_502().steer_en_state() == 1);
  }

  if (chassis_detail.has_throttle_report_500() &&
      chassis_detail.throttle_report_500().has_throttle_en_state()) {
    chassis_.mutable_check_response()->set_is_vcu_online(
        chassis_detail.throttle_report_500().throttle_en_state() == 1);
  }

  // check chassis error
  if (CheckChassisError()) {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    chassis_.mutable_engage_advice()->set_reason(
        "Chassis has some fault, please check the chassis_detail.");
  }

  // check the chassis detail lost
  if (is_chassis_communication_error_) {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    chassis_.mutable_engage_advice()->set_reason(
        "demo chassis detail is lost! Please check the communication error.");
    set_chassis_error_code(Chassis::CHASSIS_CAN_LOST);
    set_driving_mode(Chassis::EMERGENCY_MODE);
  }

  /* ADD YOUR OWN CAR CHASSIS OPERATION
  // 14 battery soc
  // 16 sonor list
  // 17 set vin
  // 18,19 bumper event
  // 20 add checkresponse signal
  */

  return chassis_;
}

void DemoController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode DemoController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  // set enable
  brake_command_101_->set_brake_en_ctrl(
      Brake_command_101::BRAKE_EN_CTRL_ENABLE);

  gear_command_103_->set_gear_en_ctrl(Gear_command_103::GEAR_EN_CTRL_ENABLE);

  steering_command_102_->set_steer_en_ctrl(
      Steering_command_102::STEER_EN_CTRL_ENABLE);

  throttle_command_100_->set_throttle_en_ctrl(
      Throttle_command_100::THROTTLE_EN_CTRL_ENABLE);

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode. Please check the "
              "emergency button or chassis.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
  return ErrorCode::OK;
}

ErrorCode DemoController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode DemoController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode.";
    return ErrorCode::OK;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  // TODO(ALL): CHECK YOUR VEHICLE WHETHER SUPPORT THIS MODE OR NOT
  // set enable
    brake_command_101_->set_brake_en_ctrl(
      Brake_command_101::BRAKE_EN_CTRL_DISABLE);

  gear_command_103_->set_gear_en_ctrl(
      Gear_command_103::GEAR_EN_CTRL_DISABLE);

  steering_command_102_->set_steer_en_ctrl(
      Steering_command_102::STEER_EN_CTRL_ENABLE);

  throttle_command_100_->set_throttle_en_ctrl(
      Throttle_command_100::THROTTLE_EN_CTRL_DISABLE);


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

ErrorCode DemoController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  // TODO(ALL): CHECK YOUR VEHICLE WHETHER SUPPORT THIS MODE OR NOT
  // set enable
    brake_command_101_->set_brake_en_ctrl(
      Brake_command_101::BRAKE_EN_CTRL_ENABLE);

  gear_command_103_->set_gear_en_ctrl(
      Gear_command_103::GEAR_EN_CTRL_ENABLE);

  steering_command_102_->set_steer_en_ctrl(
      Steering_command_102::STEER_EN_CTRL_DISABLE);

  throttle_command_100_->set_throttle_en_ctrl(
      Throttle_command_100::THROTTLE_EN_CTRL_ENABLE);


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
void DemoController::Gear(Chassis::GearPosition gear_position) {
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
// pedal:0.00~99.99, unit:percentage
void DemoController::Brake(double pedal) {
  // double real_value = vehicle_params_.max_acceleration() * acceleration /
  // 100;
  // TODO(All) :  Update brake value based on mode
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
  brake_command_101_->set_brake_pedal_target(pedal);
}

// drive with pedal
// pedal:0.0~99.9 unit:percentage
void DemoController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  throttle_command_100_->set_throttle_pedal_target(pedal);
}

// confirm the car is driven by acceleration command instead of
// throttle/brake pedal drive with acceleration/deceleration acc:-7.0 ~ 5.0,
// unit:m/s^2
void DemoController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  // TODO(ALL): CHECK YOUR VEHICLE WHETHER SUPPORT THIS DRIVE MODE
  */
}

// confirm the car is driven by speed command
// speed:-xx.0~xx.0, unit:m/s
void DemoController::Speed(double speed) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set speed.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  // TODO(ALL): CHECK YOUR VEHICLE WHETHER SUPPORT THIS DRIVE MODE
  */
}

// demo default, +470 ~ -470 or other, left:+, right:-
// need to be compatible with control module, so reverse
// steering with steering angle
// angle:99.99~0.00~-99.99, unit:deg, left:+, right:-
void DemoController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  steering_command_102_->set_steer_angle_target(real_angle);
}

// demo default, steering with new angle and angle speed
// angle:99.99~0.00~-99.99, unit:deg, left:+, right:-
// angle_spd:0.00~99.99, unit:deg/s
void DemoController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  steering_command_102_->set_steer_angle_target(real_angle);
}

void DemoController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void DemoController::SetBeam(const VehicleSignal& vehicle_signal) {
  if (vehicle_signal.high_beam()) {
    // None
  } else if (vehicle_signal.low_beam()) {
    // None
  } else {
    // None
  }
}

void DemoController::SetHorn(const VehicleSignal& vehicle_signal) {
  if (vehicle_signal.horn()) {
    // None
  } else {
    // None
  }
}

void DemoController::SetTurningSignal(const VehicleSignal& vehicle_signal) {
  // Set Turn Signal
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  auto signal = vehicle_signal.turn_signal();
  if (signal == common::VehicleSignal::TURN_LEFT) {

  } else if (signal == common::VehicleSignal::TURN_RIGHT) {

  } else {

  }
  */
}

ErrorCode DemoController::HandleCustomOperation(
    const external_command::ChassisCommand& command) {
  return ErrorCode::OK;
}

bool DemoController::VerifyID() {
  if (!CheckVin()) {
    AERROR << "Failed to get the vin. Get vin again.";
    GetVin();
    return false;
  } else {
    ResetVin();
    return true;
  }
}

bool DemoController::CheckVin() {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  if (chassis_.vehicle_id().vin().size() >= 7) {
    AINFO << "Vin check success! Vehicel vin is "
          << chassis_.vehicle_id().vin();
    return true;
  } else {
    AINFO << "Vin check failed! Current vin size is "
          << chassis_.vehicle_id().vin().size();
    return false;
  }
  */
  return false;
}

void DemoController::GetVin() {
  // Get vin from vehicle if exist
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  vehicle_mode_command_116_->set_vin_req_cmd(
      Vehicle_mode_command_116::VIN_REQ_CMD_VIN_REQ_ENABLE);
  AINFO << "Get vin";
  can_sender_->Update();
  */
}

void DemoController::ResetVin() {
  // Reset vin from vehicle if exist
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  vehicle_mode_command_116_->set_vin_req_cmd(
      Vehicle_mode_command_116::VIN_REQ_CMD_VIN_REQ_DISABLE);
  AINFO << "Reset vin";
  can_sender_->Update();
  */
}

void DemoController::ResetProtocol() { message_manager_->ResetSendMessages(); }

bool DemoController::CheckChassisError() {
  if (is_chassis_communication_error_) {
    AERROR_EVERY(100) << "ChassisDetail has no demo vehicle info.";
    return false;
  }

  /* ADD YOUR OWN CAR CHASSIS OPERATION
  // steer fault
  // drive fault
  // brake fault
  */
  return false;
}

void DemoController::SecurityDogThreadFunc() {
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
        !CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false)) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        AERROR << "Driving_mode is into emergency by steer manual intervention";
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
        AERROR << "Driving_mode is into emergency by speed manual intervention";
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }

    // 3. chassis fault check
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR);
      emergency_mode = true;
    }

    // process emergency_mode
    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      message_manager_->ResetSendMessages();
      can_sender_->Update();
    }

    // recove error code
    if (!emergency_mode && !is_chassis_communication_error_ &&
        mode == Chassis::EMERGENCY_MODE) {
      set_chassis_error_code(Chassis::NO_ERROR);
    }

    end = ::apollo::cyber::Time::Now().ToMicrosecond();
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR << "Too much time consumption in DemoController looping process:"
             << elapsed.count();
    }
  }
}

bool DemoController::CheckResponse(const int32_t flags, bool need_wait) {
  int32_t retry_num = 20;
  bool is_eps_online = false;
  bool is_vcu_online = false;
  bool is_esp_online = false;

  do {
    bool check_ok = true;
    if (flags & CHECK_RESPONSE_STEER_UNIT_FLAG) {
      is_eps_online = chassis_.has_check_response() &&
                      chassis_.check_response().has_is_eps_online() &&
                      chassis_.check_response().is_eps_online();
      check_ok = check_ok && is_eps_online;
    }

    if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG) {
      is_vcu_online = chassis_.has_check_response() &&
                      chassis_.check_response().has_is_vcu_online() &&
                      chassis_.check_response().is_vcu_online();
      is_esp_online = chassis_.has_check_response() &&
                      chassis_.check_response().has_is_esp_online() &&
                      chassis_.check_response().is_esp_online();
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

  if (flags & CHECK_RESPONSE_STEER_UNIT_FLAG) {
    AERROR << "steer check_response fail: is_eps_online:" << is_eps_online;
  }

  if (flags & CHECK_RESPONSE_SPEED_UNIT_FLAG) {
    AERROR << "speed check_response fail: " << "is_vcu_online:" << is_vcu_online
           << ", is_esp_online:" << is_esp_online;
  }

  return false;
}

void DemoController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t DemoController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode DemoController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void DemoController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace demo
}  // namespace canbus
}  // namespace apollo
