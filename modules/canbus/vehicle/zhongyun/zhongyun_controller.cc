/* Copyright 2019 The Apollo Authors. All Rights Reserved.

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

#include "modules/canbus/vehicle/zhongyun/zhongyun_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/canbus/vehicle/zhongyun/zhongyun_message_manager.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace zhongyun {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}  // namespace

ErrorCode ZhongyunController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "ZhongyunController has already been initialized.";
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
    AERROR << "Protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // Sender part
  brake_control_a4_ = dynamic_cast<Brakecontrola4*>(
      message_manager_->GetMutableProtocolDataById(Brakecontrola4::ID));
  if (brake_control_a4_ == nullptr) {
    AERROR << "Brakecontrola4 does not exist in the ZhongyunMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  gear_control_a1_ = dynamic_cast<Gearcontrola1*>(
      message_manager_->GetMutableProtocolDataById(Gearcontrola1::ID));
  if (gear_control_a1_ == nullptr) {
    AERROR << "Gearcontrola1 does not exist in the ZhongyunMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  parking_control_a5_ = dynamic_cast<Parkingcontrola5*>(
      message_manager_->GetMutableProtocolDataById(Parkingcontrola5::ID));
  if (parking_control_a5_ == nullptr) {
    AERROR << "Parkingcontrola5 does not exist in the ZhongyunMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  steering_control_a2_ = dynamic_cast<Steeringcontrola2*>(
      message_manager_->GetMutableProtocolDataById(Steeringcontrola2::ID));
  if (steering_control_a2_ == nullptr) {
    AERROR << "Steeringcontrola2 does not exist in the ZhongyunMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  torque_control_a3_ = dynamic_cast<Torquecontrola3*>(
      message_manager_->GetMutableProtocolDataById(Torquecontrola3::ID));
  if (torque_control_a3_ == nullptr) {
    AERROR << "Torquecontrola3 does not exist in the ZhongyunMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Brakecontrola4::ID, brake_control_a4_, false);
  can_sender_->AddMessage(Gearcontrola1::ID, gear_control_a1_, false);
  can_sender_->AddMessage(Parkingcontrola5::ID, parking_control_a5_, false);
  can_sender_->AddMessage(Steeringcontrola2::ID, steering_control_a2_, false);
  can_sender_->AddMessage(Torquecontrola3::ID, torque_control_a3_, false);

  // Need to sleep to ensure all messages received
  AINFO << "ZhongyunController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

ZhongyunController::~ZhongyunController() {}

bool ZhongyunController::Start() {
  if (!is_initialized_) {
    AERROR << "ZhongyunController has NOT been initialized.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void ZhongyunController::Stop() {
  if (!is_initialized_) {
    AERROR << "ZhongyunController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "ZhongyunController stopped.";
  }
}

Chassis ZhongyunController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  // 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);
  // if there is not zhongyun, no chassis detail can be retrieved and return
  if (!chassis_detail.has_zhongyun()) {
    AERROR << "NO ZHONGYUN chassis information!";
    return chassis_;
  }
  Zhongyun zhy = chassis_detail.zhongyun();

  // 4 engine_rpm
  if (zhy.has_vehicle_state_feedback_2_c4() &&
      zhy.vehicle_state_feedback_2_c4().has_motor_speed()) {
    chassis_.set_engine_rpm(
        static_cast<float>(zhy.vehicle_state_feedback_2_c4().motor_speed()));
  } else {
    chassis_.set_engine_rpm(0);
  }
  // 5 speed_mps
  if (zhy.has_vehicle_state_feedback_c1() &&
      zhy.vehicle_state_feedback_c1().has_speed()) {
    chassis_.set_speed_mps(
        static_cast<float>(zhy.vehicle_state_feedback_c1().speed()));
  } else {
    chassis_.set_speed_mps(0);
  }
  // 6
  chassis_.set_fuel_range_m(0);

  // 7 acc_pedal
  if (zhy.has_vehicle_state_feedback_2_c4() &&
      zhy.vehicle_state_feedback_2_c4().has_driven_torque_feedback()) {
    chassis_.set_throttle_percentage(static_cast<float>(
        zhy.vehicle_state_feedback_2_c4().driven_torque_feedback()));
  } else {
    chassis_.set_throttle_percentage(0);
  }
  // 8 brake_pedal
  if (zhy.has_vehicle_state_feedback_c1() &&
      zhy.vehicle_state_feedback_c1().has_brake_torque_feedback()) {
    chassis_.set_brake_percentage(static_cast<float>(
        zhy.vehicle_state_feedback_c1().brake_torque_feedback()));
  } else {
    chassis_.set_brake_percentage(0);
  }
  // 9 gear position
  if (zhy.has_vehicle_state_feedback_c1() &&
      zhy.vehicle_state_feedback_c1().has_gear_state_actual()) {
    switch (zhy.vehicle_state_feedback_c1().gear_state_actual()) {
      case Vehicle_state_feedback_c1::GEAR_STATE_ACTUAL_D: {
        chassis_.set_gear_location(Chassis::GEAR_DRIVE);
      } break;
      case Vehicle_state_feedback_c1::GEAR_STATE_ACTUAL_N: {
        chassis_.set_gear_location(Chassis::GEAR_NEUTRAL);
      } break;
      case Vehicle_state_feedback_c1::GEAR_STATE_ACTUAL_R: {
        chassis_.set_gear_location(Chassis::GEAR_REVERSE);
      } break;
      case Vehicle_state_feedback_c1::GEAR_STATE_ACTUAL_P: {
        chassis_.set_gear_location(Chassis::GEAR_PARKING);
      } break;
      default:
        chassis_.set_gear_location(Chassis::GEAR_INVALID);
        break;
    }
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }
  // 11 steering_percentage
  if (zhy.has_vehicle_state_feedback_c1() &&
      zhy.vehicle_state_feedback_c1().has_steering_actual()) {
    chassis_.set_steering_percentage(static_cast<float>(
        zhy.vehicle_state_feedback_c1().steering_actual() * 100.0 /
        vehicle_params_.max_steer_angle() * M_PI / 180));
  } else {
    chassis_.set_steering_percentage(0);
  }
  // 12 epb
  if (zhy.has_vehicle_state_feedback_c1() &&
      zhy.vehicle_state_feedback_c1().has_parking_actual()) {
    chassis_.set_parking_brake(
        zhy.vehicle_state_feedback_c1().parking_actual() ==
        Vehicle_state_feedback_c1::PARKING_ACTUAL_PARKING_TRIGGER);
  } else {
    chassis_.set_parking_brake(false);
  }
  // 13 error mask
  if (chassis_error_mask_) {
    chassis_.set_chassis_error_mask(chassis_error_mask_);
  }
  // Give engage_advice based on error_code and canbus feedback
  if (!chassis_error_mask_ && !chassis_.parking_brake()) {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::READY_TO_ENGAGE);
  } else {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    chassis_.mutable_engage_advice()->set_reason(
        "CANBUS not ready, epb is not released or firmware error!");
  }
  return chassis_;
}

void ZhongyunController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode ZhongyunController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "Already in COMPLETE_AUTO_DRIVE mode.";
    return ErrorCode::OK;
  }
  steering_control_a2_->set_steering_enable_control(
      Steering_control_a2::STEERING_ENABLE_CONTROL_STEERING_AUTOCONTROL);
  gear_control_a1_->set_gear_enable_control(
      Gear_control_a1::GEAR_ENABLE_CONTROL_GEAR_AUTOCONTROL);
  torque_control_a3_->set_driven_enable_control(
      Torque_control_a3::DRIVEN_ENABLE_CONTROL_DRIVE_AUTO);
  brake_control_a4_->set_brake_enable_control(
      Brake_control_a4::BRAKE_ENABLE_CONTROL_BRAKE_AUTO);
  parking_control_a5_->set_parking_enable_control(
      Parking_control_a5::PARKING_ENABLE_CONTROL_PARKING_AUTOCONTROL);

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

ErrorCode ZhongyunController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode ZhongyunController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode";
    return ErrorCode::OK;
  }
  steering_control_a2_->set_steering_enable_control(
      Steering_control_a2::STEERING_ENABLE_CONTROL_STEERING_AUTOCONTROL);
  gear_control_a1_->set_gear_enable_control(
      Gear_control_a1::GEAR_ENABLE_CONTROL_GEAR_MANUALCONTROL);
  torque_control_a3_->set_driven_enable_control(
      Torque_control_a3::DRIVEN_ENABLE_CONTROL_DRIVE_MANUAL);
  brake_control_a4_->set_brake_enable_control(
      Brake_control_a4::BRAKE_ENABLE_CONTROL_BRAKE_MANUAL);
  parking_control_a5_->set_parking_enable_control(
      Parking_control_a5::PARKING_ENABLE_CONTROL_PARKING_MANUALCONTROL);

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_STEER_ONLY);
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
  return ErrorCode::OK;
}

ErrorCode ZhongyunController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  steering_control_a2_->set_steering_enable_control(
      Steering_control_a2::STEERING_ENABLE_CONTROL_STEERING_MANUALCONTROL);
  gear_control_a1_->set_gear_enable_control(
      Gear_control_a1::GEAR_ENABLE_CONTROL_GEAR_AUTOCONTROL);
  torque_control_a3_->set_driven_enable_control(
      Torque_control_a3::DRIVEN_ENABLE_CONTROL_DRIVE_AUTO);
  brake_control_a4_->set_brake_enable_control(
      Brake_control_a4::BRAKE_ENABLE_CONTROL_BRAKE_AUTO);
  parking_control_a5_->set_parking_enable_control(
      Parking_control_a5::PARKING_ENABLE_CONTROL_PARKING_AUTOCONTROL);

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_SPEED_ONLY);
  AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
  return ErrorCode::OK;
}

// NEUTRAL, REVERSE, DRIVE, PARK
void ZhongyunController::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }

  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      gear_control_a1_->set_gear_state_target(
          Gear_control_a1::GEAR_STATE_TARGET_N);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      gear_control_a1_->set_gear_state_target(
          Gear_control_a1::GEAR_STATE_TARGET_R);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      gear_control_a1_->set_gear_state_target(
          Gear_control_a1::GEAR_STATE_TARGET_D);
      break;
    }
    case Chassis::GEAR_PARKING: {
      gear_control_a1_->set_gear_state_target(
          Gear_control_a1::GEAR_STATE_TARGET_P);
      break;
    }

    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      gear_control_a1_->set_gear_state_target(
          Gear_control_a1::GEAR_STATE_TARGET_INVALID);
      break;
    }
    default: {
      gear_control_a1_->set_gear_state_target(
          Gear_control_a1::GEAR_STATE_TARGET_P);
      break;
    }
  }
}

// brake with brake pedal
// pedal:0.00~99.99, unit:percentage
void ZhongyunController::Brake(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
  brake_control_a4_->set_brake_torque(pedal);
}

// drive with throttle pedal
// pedal:0.00~99.99 unit:percentage
void ZhongyunController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  torque_control_a3_->set_driven_torque(pedal);
}

// confirm the car is driven by acceleration command or throttle/brake pedal
// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void ZhongyunController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  // None
}

// zhongyun default, -30 ~ 30, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void ZhongyunController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  steering_control_a2_->set_steering_target(real_angle);
}

// steering with new angle speed
// zhongyun has no angle_speed
// angle:-30~30, unit:deg, left:+, right:-
void ZhongyunController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  steering_control_a2_->set_steering_target(real_angle);
}

void ZhongyunController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    parking_control_a5_->set_parking_target(
        Parking_control_a5::PARKING_TARGET_PARKING_TRIGGER);
  } else {
    parking_control_a5_->set_parking_target(
        Parking_control_a5::PARKING_TARGET_RELEASE);
  }
}

void ZhongyunController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void ZhongyunController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void ZhongyunController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  // None
}

void ZhongyunController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool ZhongyunController::CheckChassisError() {
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  if (!chassis_detail.has_zhongyun()) {
    AERROR_EVERY(100) << "ChassisDetail has NO zhongyun vehicle info."
                      << chassis_detail.DebugString();
    return false;
  }
  Zhongyun zhy = chassis_detail.zhongyun();
  // check steer error
  if (zhy.has_error_state_e1() &&
      zhy.error_state_e1().has_steering_error_code()) {
    if (zhy.error_state_e1().steering_error_code() ==
        Error_state_e1::STEERING_ERROR_CODE_ERROR) {
      return true;
    }
  }
  // check ems error
  if (zhy.has_error_state_e1() &&
      zhy.error_state_e1().has_driven_error_code()) {
    if (zhy.error_state_e1().driven_error_code() ==
        Error_state_e1::DRIVEN_ERROR_CODE_ERROR) {
      return true;
    }
  }
  // check eps error
  if (zhy.has_error_state_e1() && zhy.error_state_e1().has_brake_error_code()) {
    if (zhy.error_state_e1().brake_error_code() ==
        Error_state_e1::BRAKE_ERROR_CODE_ERROR) {
      return true;
    }
  }
  // check gear error
  if (zhy.has_error_state_e1() && zhy.error_state_e1().has_gear_error_msg()) {
    if (zhy.error_state_e1().gear_error_msg() ==
        Error_state_e1::GEAR_ERROR_MSG_ERROR) {
      return true;
    }
  }
  // check parking error
  if (zhy.has_error_state_e1() &&
      zhy.error_state_e1().has_parking_error_code()) {
    if (zhy.error_state_e1().parking_error_code() ==
        Error_state_e1::PARKING_ERROR_CODE_ERROR) {
      return true;
    }
  }
  return false;
}

void ZhongyunController::SecurityDogThreadFunc() {
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
    start = absl::ToUnixMicros(::apollo::common::time::Clock::Now());
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false)) {
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
    end = absl::ToUnixMicros(::apollo::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR
          << "Too much time consumption in ZhongyunController looping process:"
          << elapsed.count();
    }
  }
}

bool ZhongyunController::CheckResponse(const int32_t flags, bool need_wait) {
  // for Zhongyun, CheckResponse commonly takes 300ms. We leave a 100ms buffer
  // for it.
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
    if (need_wait) {
      --retry_num;
      std::this_thread::sleep_for(
          std::chrono::duration<double, std::milli>(20));
    }
    if (check_ok) {
      return true;
    } else {
      AINFO << "Need to check response again.";
    }
  } while (need_wait && retry_num);

  AINFO << "check_response fail: is_eps_online:" << is_eps_online
        << ", is_vcu_online:" << is_vcu_online
        << ", is_esp_online:" << is_esp_online;
  return false;
}

void ZhongyunController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t ZhongyunController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode ZhongyunController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void ZhongyunController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace zhongyun
}  // namespace canbus
}  // namespace apollo
