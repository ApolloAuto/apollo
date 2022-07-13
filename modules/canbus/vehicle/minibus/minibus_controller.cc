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

#include "modules/canbus/vehicle/minibus/minibus_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/canbus/vehicle/minibus/minibus_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace minibus {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}  // namespace

ErrorCode MinibusController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "MinibusController has already been initiated.";
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
  // 驻车控制
  controller_parking_18ff8ca9_ = dynamic_cast<Controllerparking18ff8ca9*>(
      message_manager_->GetMutableProtocolDataById(
          Controllerparking18ff8ca9::ID));
  if (controller_parking_18ff8ca9_ == nullptr) {
    AERROR << "Controllerparking18ff8ca9 does not exist in the "
              "MinibusMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }
  // 驱动、制动控制
  controller_pedal_cmd_18ff84a9_ = dynamic_cast<Controllerpedalcmd18ff84a9*>(
      message_manager_->GetMutableProtocolDataById(
          Controllerpedalcmd18ff84a9::ID));
  if (controller_pedal_cmd_18ff84a9_ == nullptr) {
    AERROR << "Controllerpedalcmd18ff84a9 does not exist in the "
              "MinibusMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }
  // 灯光控制
  controller_status_requset_18ff86a9_ =
      dynamic_cast<Controllerstatusrequset18ff86a9*>(
          message_manager_->GetMutableProtocolDataById(
              Controllerstatusrequset18ff86a9::ID));
  if (controller_status_requset_18ff86a9_ == nullptr) {
    AERROR << "Controllerstatusrequset18ff86a9 does not exist in the "
              "MinibusMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }
  // 转向控制
  controller_steering_cmd_18ff82a9_ =
      dynamic_cast<Controllersteeringcmd18ff82a9*>(
          message_manager_->GetMutableProtocolDataById(
              Controllersteeringcmd18ff82a9::ID));
  if (controller_steering_cmd_18ff82a9_ == nullptr) {
    AERROR << "Controllersteeringcmd18ff82a9 does not exist in the "
              "MinibusMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Controllerparking18ff8ca9::ID,
                          controller_parking_18ff8ca9_, false);
  can_sender_->AddMessage(Controllerpedalcmd18ff84a9::ID,
                          controller_pedal_cmd_18ff84a9_, false);
  can_sender_->AddMessage(Controllerstatusrequset18ff86a9::ID,
                          controller_status_requset_18ff86a9_, false);
  can_sender_->AddMessage(Controllersteeringcmd18ff82a9::ID,
                          controller_steering_cmd_18ff82a9_, false);

  // need sleep to ensure all messages received
  AINFO << "MinibusController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

MinibusController::~MinibusController() {}

bool MinibusController::Start() {
  if (!is_initialized_) {
    AERROR << "MinibusController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void MinibusController::Stop() {
  if (!is_initialized_) {
    AERROR << "MinibusController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "MinibusController stopped.";
  }
}

Chassis MinibusController::chassis() {
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
  // 4 speed
  if (chassis_detail.minibus().has_bus_vehicle_speed_msg_cfe6c17() &&
      chassis_detail.minibus()
          .bus_vehicle_speed_msg_cfe6c17()
          .has_bus_vehicle_speed()) {
    chassis_.set_speed_mps(
        static_cast<float>(chassis_detail.minibus()
                               .bus_vehicle_speed_msg_cfe6c17()
                               .bus_vehicle_speed()));
  } else {
    chassis_.set_speed_mps(0);
  }
  // 8 throttle
  if (chassis_detail.minibus().has_vcu_drive_feedback_18ff7097() &&
      chassis_detail.minibus()
          .vcu_drive_feedback_18ff7097()
          .has_vcu_drive_throttle_pedal_position()) {
    chassis_.set_throttle_percentage(
        static_cast<float>(chassis_detail.minibus()
                               .vcu_drive_feedback_18ff7097()
                               .vcu_drive_throttle_pedal_position()));
  } else {
    chassis_.set_throttle_percentage(0);
  }
  // 9 brake
  if (chassis_detail.minibus().has_vcu_breaksys_cmd_18ff85a7()) {
    double break_pedal = (chassis_detail.minibus()
                              .vcu_breaksys_cmd_18ff85a7()
                              .vcu_brk_left_pressure() +
                          chassis_detail.minibus()
                              .vcu_breaksys_cmd_18ff85a7()
                              .vcu_brk_right_pressure()) /
                         2.0 / 9.0 * 100;
    chassis_.set_brake_percentage(static_cast<float>(break_pedal));
  } else {
    chassis_.set_brake_percentage(0);
  }
  // 10 gear
  if (chassis_detail.minibus().has_vcu_basic_message_18ffea97() &&
      chassis_detail.minibus()
          .vcu_basic_message_18ffea97()
          .has_vcu_basic_real_gear()) {
    Chassis::GearPosition gear_pos = Chassis::GEAR_INVALID;

    if (chassis_detail.minibus()
            .vcu_basic_message_18ffea97()
            .vcu_basic_real_gear() ==
        Vcu_basic_message_18ffea97::VCU_BASIC_REAL_GEAR_N) {
      gear_pos = Chassis::GEAR_NEUTRAL;
    }
    if (chassis_detail.minibus()
            .vcu_basic_message_18ffea97()
            .vcu_basic_real_gear() ==
        Vcu_basic_message_18ffea97::VCU_BASIC_REAL_GEAR_R) {
      gear_pos = Chassis::GEAR_REVERSE;
    }
    if (chassis_detail.minibus()
            .vcu_basic_message_18ffea97()
            .vcu_basic_real_gear() ==
        Vcu_basic_message_18ffea97::VCU_BASIC_REAL_GEAR_D) {
      gear_pos = Chassis::GEAR_DRIVE;
    }
    chassis_.set_gear_location(gear_pos);
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }
  // 11 steering
  if (chassis_detail.minibus().has_eps_feedback_18ff83aa() &&
      chassis_detail.minibus().eps_feedback_18ff83aa().has_eps_real_angle()) {
    chassis_.set_steering_percentage(static_cast<float>(
        chassis_detail.minibus().eps_feedback_18ff83aa().eps_real_angle() *
        100.0 / vehicle_params_.max_steer_angle() * M_PI / 180));
  } else {
    chassis_.set_steering_percentage(0);
  }
  // 12 battery soc
  if (chassis_detail.minibus().has_soc_18ffeb97() &&
      chassis_detail.minibus().soc_18ffeb97().has_soc()) {
    chassis_.set_battery_soc_percentage(
        chassis_detail.minibus().soc_18ffeb97().soc());
  }
  // 13 parking brake
  if (chassis_detail.minibus()
          .parkingmode_feedback_18ff8dac()
          .has_pmf_current_status() &&
      chassis_detail.minibus()
              .parkingmode_feedback_18ff8dac()
              .pmf_current_status() ==
          Parkingmode_feedback_18ff8dac::PMF_CURRENT_STATUS_EPB_LOCK) {
    chassis_.set_parking_brake(true);
  } else {
    chassis_.set_parking_brake(false);
  }
  // 14 give engage_advice based on battery low soc warn
  if (chassis_.battery_soc_percentage() <= 30) {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    chassis_.mutable_engage_advice()->set_reason(
        "Battery soc percentage is lower than 30%, please charge it quickly!");
  }
  return chassis_;
}

bool MinibusController::VerifyID() { return true; }

void MinibusController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode MinibusController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  controller_steering_cmd_18ff82a9_->set_steering_ctrl_status(
      Controller_steering_cmd_18ff82a9::
          STEERING_CTRL_STATUS_RESET_FROM_MAUNAL_INTERVENTION);
  can_sender_->Update();
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(20));
  controller_steering_cmd_18ff82a9_->set_steering_ctrl_status(
      Controller_steering_cmd_18ff82a9::STEERING_CTRL_STATUS_AUTO_DRIVE);
  controller_pedal_cmd_18ff84a9_->set_pedal_ctrl_request(
      Controller_pedal_cmd_18ff84a9::PEDAL_CTRL_REQUEST_ON);
  AINFO << "set complete auto mode enable.";
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

ErrorCode MinibusController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode MinibusController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode.";
    return ErrorCode::OK;
  }
  controller_steering_cmd_18ff82a9_->set_steering_ctrl_status(
      Controller_steering_cmd_18ff82a9::
          STEERING_CTRL_STATUS_RESET_FROM_MAUNAL_INTERVENTION);
  can_sender_->Update();
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(20));
  controller_steering_cmd_18ff82a9_->set_steering_ctrl_status(
      Controller_steering_cmd_18ff82a9::STEERING_CTRL_STATUS_AUTO_DRIVE);
  controller_pedal_cmd_18ff84a9_->set_pedal_ctrl_request(
      Controller_pedal_cmd_18ff84a9::PEDAL_CTRL_REQUEST_OFF);
  AINFO << "set auto steer only mode enable.";
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
}

ErrorCode MinibusController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  controller_steering_cmd_18ff82a9_->set_steering_ctrl_status(
      Controller_steering_cmd_18ff82a9::
          STEERING_CTRL_STATUS_RESET_FROM_MAUNAL_INTERVENTION);
  can_sender_->Update();
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(20));
  controller_steering_cmd_18ff82a9_->set_steering_ctrl_status(
      Controller_steering_cmd_18ff82a9::STEERING_CTRL_STATUS_READY);
  controller_pedal_cmd_18ff84a9_->set_pedal_ctrl_request(
      Controller_pedal_cmd_18ff84a9::PEDAL_CTRL_REQUEST_ON);
  AINFO << "set auto speed only mode enable.";
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
}

// NEUTRAL, REVERSE, DRIVE
void MinibusController::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION  */
  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      controller_pedal_cmd_18ff84a9_->set_pedal_gear(
          Controller_pedal_cmd_18ff84a9::PEDAL_GEAR_N);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      controller_pedal_cmd_18ff84a9_->set_pedal_gear(
          Controller_pedal_cmd_18ff84a9::PEDAL_GEAR_R);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      controller_pedal_cmd_18ff84a9_->set_pedal_gear(
          Controller_pedal_cmd_18ff84a9::PEDAL_GEAR_D);
      break;
    }
    case Chassis::GEAR_INVALID: {
      controller_pedal_cmd_18ff84a9_->set_pedal_gear(
          Controller_pedal_cmd_18ff84a9::PEDAL_GEAR_INVALID);
      break;
    }
    case Chassis::GEAR_PARKING: {
      controller_pedal_cmd_18ff84a9_->set_pedal_gear(
          Controller_pedal_cmd_18ff84a9::PEDAL_GEAR_N);
      break;
    }
    default: {
      controller_pedal_cmd_18ff84a9_->set_pedal_gear(
          Controller_pedal_cmd_18ff84a9::PEDAL_GEAR_N);
      break;
    }
  }
}

// brake with pedal
// pedal:0.00~99.99 unit:%
void MinibusController::Brake(double pedal) {
  // double real_value = params_.max_acc() * acceleration / 100;
  // TODO(All) :  Update brake value based on mode
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
  if (pedal >= 2.0) {
    controller_status_requset_18ff86a9_->set_sr_breaklight(
        Controller_status_requset_18ff86a9::SR_BREAKLIGHT_ON);
  } else {
    controller_status_requset_18ff86a9_->set_sr_breaklight(
        Controller_status_requset_18ff86a9::SR_BREAKLIGHT_OFF);
  }
  controller_pedal_cmd_18ff84a9_->set_pedal_break(pedal);
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void MinibusController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  controller_pedal_cmd_18ff84a9_->set_pedal_throttle(pedal);
}

// confirm the car is driven by acceleration command or drive/brake pedal
// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void MinibusController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  // None
}

// minibus default, -700 ~ +700, right:-, left:+
// need to be compatible with control module, so reverse
// steering with angle
// angle:-99.99~0.00~99.99, unit:%, left:+, right:-
void MinibusController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  controller_steering_cmd_18ff82a9_->set_streering_angle(real_angle)
      ->set_steering_velocity(400);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:%, left:+, right:-
// angle_spd:0.00~99.99, unit:%(deg/s)
void MinibusController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  controller_steering_cmd_18ff82a9_->set_streering_angle(real_angle)
      ->set_steering_velocity(400);
}

void MinibusController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // do not enable epb parking control because it can not control by manual
    // if cp_park_active enabled.
    // controller_parking_18ff8ca9_->set_cp_park_active(
    //     Controller_parking_18ff8ca9::CP_PARK_ACTIVE_ACTIVE);
    controller_parking_18ff8ca9_->set_cp_epb_enable(
        Controller_parking_18ff8ca9::CP_EPB_ENABLE_EPB_TRIGGER);
  } else {
    // controller_parking_18ff8ca9_->set_cp_park_active(
    //     Controller_parking_18ff8ca9::CP_PARK_ACTIVE_ACTIVE);
    controller_parking_18ff8ca9_->set_cp_epb_enable(
        Controller_parking_18ff8ca9::CP_EPB_ENABLE_EPB_RELEASE);
  }
}

void MinibusController::SetBeam(const ControlCommand& command) {
  if (command.signal().low_beam()) {
    controller_status_requset_18ff86a9_->set_sr_lowbeam(
        Controller_status_requset_18ff86a9::SR_LOWBEAM_ON);
  } else {
    controller_status_requset_18ff86a9_->set_sr_lowbeam(
        Controller_status_requset_18ff86a9::SR_LOWBEAM_OFF);
  }
}

void MinibusController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    controller_status_requset_18ff86a9_->set_sr_horn(
        Controller_status_requset_18ff86a9::SR_HORN_ON);
  } else {
    controller_status_requset_18ff86a9_->set_sr_horn(
        Controller_status_requset_18ff86a9::SR_HORN_OFF);
  }
}

void MinibusController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  auto signal = command.signal().turn_signal();
  if (signal == common::VehicleSignal::TURN_LEFT) {
    controller_status_requset_18ff86a9_->set_sr_turnleft(
        Controller_status_requset_18ff86a9::SR_TURNLEFT_ON);
    controller_status_requset_18ff86a9_->set_sr_turnright(
        Controller_status_requset_18ff86a9::SR_TURNRIGHT_OFF);
  } else if (signal == common::VehicleSignal::TURN_RIGHT) {
    controller_status_requset_18ff86a9_->set_sr_turnleft(
        Controller_status_requset_18ff86a9::SR_TURNLEFT_OFF);
    controller_status_requset_18ff86a9_->set_sr_turnright(
        Controller_status_requset_18ff86a9::SR_TURNRIGHT_ON);
  } else {
    controller_status_requset_18ff86a9_->set_sr_turnleft(
        Controller_status_requset_18ff86a9::SR_TURNLEFT_OFF);
    controller_status_requset_18ff86a9_->set_sr_turnright(
        Controller_status_requset_18ff86a9::SR_TURNRIGHT_OFF);
  }
}

void MinibusController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool MinibusController::CheckChassisError() {
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  if (!chassis_detail.has_minibus()) {
    AERROR_EVERY(100) << "ChassisDetail has no devkit vehicle info."
                      << chassis_detail.DebugString();
    return false;
  }

  Minibus minibus = chassis_detail.minibus();

  // door open fault
  if (minibus.has_bus_vehicle_status_18fa0517()) {
    if (minibus.bus_vehicle_status_18fa0517().busv_fdoor_status() !=
        Bus_vehicle_status_18fa0517::BUSV_FDOOR_STATUS_DOOR_CLOSED) {
      return true;
    }
  }
  // vcu off-line
  if (minibus.has_vcu_drive_feedback_18ff7097()) {
    if (!minibus.vcu_drive_feedback_18ff7097().has_vcu_drive_life_signal()) {
      return true;
    }
  }
  // eps off-line
  if (!minibus.has_eps_feedback_18ff83aa()) {
    return true;
  }
  // esp off-line
  if (!minibus.has_vcu_breaksys_cmd_18ff85a7()) {
    return true;
  }
  return false;
}

void MinibusController::SecurityDogThreadFunc() {
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
          << "Too much time consumption in MinibusController looping process:"
          << elapsed.count();
    }
  }
}

bool MinibusController::CheckResponse(const int32_t flags, bool need_wait) {
  int32_t retry_num = 10;
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

void MinibusController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t MinibusController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode MinibusController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void MinibusController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace minibus
}  // namespace canbus
}  // namespace apollo
