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

#include "modules/canbus/vehicle/neolix_edu/neolix_edu_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/canbus/vehicle/neolix_edu/neolix_edu_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace neolix_edu {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}  // namespace

ErrorCode Neolix_eduController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "Neolix_eduController has already been initiated.";
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
  ads_brake_command_46_ = dynamic_cast<Adsbrakecommand46*>(
      message_manager_->GetMutableProtocolDataById(Adsbrakecommand46::ID));
  if (ads_brake_command_46_ == nullptr) {
    AERROR
        << "Adsbrakecommand46 does not exist in the Neolix_eduMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  ads_diagnosis_628_ = dynamic_cast<Adsdiagnosis628*>(
      message_manager_->GetMutableProtocolDataById(Adsdiagnosis628::ID));
  if (ads_diagnosis_628_ == nullptr) {
    AERROR << "Adsdiagnosis628 does not exist in the Neolix_eduMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  ads_drive_command_50_ = dynamic_cast<Adsdrivecommand50*>(
      message_manager_->GetMutableProtocolDataById(Adsdrivecommand50::ID));
  if (ads_drive_command_50_ == nullptr) {
    AERROR
        << "Adsdrivecommand50 does not exist in the Neolix_eduMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  ads_eps_command_56_ = dynamic_cast<Adsepscommand56*>(
      message_manager_->GetMutableProtocolDataById(Adsepscommand56::ID));
  if (ads_eps_command_56_ == nullptr) {
    AERROR << "Adsepscommand56 does not exist in the Neolix_eduMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  ads_light_horn_command_310_ = dynamic_cast<Adslighthorncommand310*>(
      message_manager_->GetMutableProtocolDataById(Adslighthorncommand310::ID));
  if (ads_light_horn_command_310_ == nullptr) {
    AERROR << "Adslighthorncommand310 does not exist in the "
              "Neolix_eduMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Adsbrakecommand46::ID, ads_brake_command_46_, false);
  can_sender_->AddMessage(Adsdiagnosis628::ID, ads_diagnosis_628_, false);
  can_sender_->AddMessage(Adsdrivecommand50::ID, ads_drive_command_50_, false);
  can_sender_->AddMessage(Adsepscommand56::ID, ads_eps_command_56_, false);
  can_sender_->AddMessage(Adslighthorncommand310::ID,
                          ads_light_horn_command_310_, false);

  // need sleep to ensure all messages received
  AINFO << "Neolix_eduController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

Neolix_eduController::~Neolix_eduController() {}

bool Neolix_eduController::Start() {
  if (!is_initialized_) {
    AERROR << "Neolix_eduController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void Neolix_eduController::Stop() {
  if (!is_initialized_) {
    AERROR << "Neolix_eduController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "Neolix_eduController stopped.";
  }
}

Chassis Neolix_eduController::chassis() {
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

  // 3 speed_mps
  if (chassis_detail.neolix_edu().has_aeb_frontwheelspeed_353() &&
      chassis_detail.neolix_edu().has_aeb_rearwheelspeed_354()) {
    auto wheelspeed = chassis_.mutable_wheel_speed();
    wheelspeed->set_wheel_spd_fl(
        chassis_detail.neolix_edu().aeb_frontwheelspeed_353().wheelspeed_fl());
    wheelspeed->set_wheel_spd_fr(
        chassis_detail.neolix_edu().aeb_frontwheelspeed_353().wheelspeed_fr());
    wheelspeed->set_wheel_spd_rl(
        chassis_detail.neolix_edu().aeb_rearwheelspeed_354().wheelspeed_rl());
    wheelspeed->set_wheel_spd_rr(
        chassis_detail.neolix_edu().aeb_rearwheelspeed_354().wheelspeed_rr());
    chassis_.set_speed_mps(
        (chassis_detail.neolix_edu().aeb_frontwheelspeed_353().wheelspeed_fl() +
         chassis_detail.neolix_edu().aeb_frontwheelspeed_353().wheelspeed_fr() +
         chassis_detail.neolix_edu().aeb_rearwheelspeed_354().wheelspeed_rl() +
         chassis_detail.neolix_edu().aeb_rearwheelspeed_354().wheelspeed_rr()) /
        4 / 3.6);
  } else {
    chassis_.set_speed_mps(0);
  }
  // 4 SOC
  if (chassis_detail.neolix_edu().has_vcu_vehicle_status_report_101() &&
      chassis_detail.neolix_edu()
          .vcu_vehicle_status_report_101()
          .has_vcu_display_soc()) {
    chassis_.set_battery_soc_percentage(chassis_detail.neolix_edu()
                                            .vcu_vehicle_status_report_101()
                                            .vcu_display_soc());
  } else {
    chassis_.set_battery_soc_percentage(0);
  }
  // 5 steering
  if (chassis_detail.neolix_edu().has_vcu_eps_report_57() &&
      chassis_detail.neolix_edu().vcu_eps_report_57().has_vcu_real_angle()) {
    chassis_.set_steering_percentage(static_cast<float>(
        chassis_detail.neolix_edu().vcu_eps_report_57().vcu_real_angle() * 100 /
        vehicle_params_.max_steer_angle() * M_PI / 180));
  } else {
    chassis_.set_steering_percentage(0);
  }

  // 6 throttle
  if (chassis_detail.neolix_edu().has_vcu_drive_report_52() &&
      chassis_detail.neolix_edu().vcu_drive_report_52().has_vcu_real_torque()) {
    chassis_.set_throttle_percentage(
        chassis_detail.neolix_edu().vcu_drive_report_52().vcu_real_torque() *
        2);
  } else {
    chassis_.set_throttle_percentage(0);
  }

  // 7 brake
  if (chassis_detail.neolix_edu().has_vcu_brake_report_47() &&
      chassis_detail.neolix_edu().vcu_brake_report_47().has_vcu_real_brake()) {
    chassis_.set_brake_percentage(
        chassis_detail.neolix_edu().vcu_brake_report_47().vcu_real_brake());
  } else {
    chassis_.set_brake_percentage(0);
  }
  // 8 gear
  if (chassis_detail.neolix_edu().has_vcu_drive_report_52() &&
      chassis_detail.neolix_edu().vcu_drive_report_52().has_vcu_real_shift()) {
    chassis_.set_gear_location(
        (apollo::canbus::Chassis_GearPosition)chassis_detail.neolix_edu()
            .vcu_drive_report_52()
            .vcu_real_shift());
  }
  // 9 epb
  if (chassis_detail.neolix_edu().has_vcu_brake_report_47() &&
      chassis_detail.neolix_edu()
          .vcu_brake_report_47()
          .has_vcu_real_parking_status()) {
    if (chassis_detail.neolix_edu()
            .vcu_brake_report_47()
            .vcu_real_parking_status() == 1) {
      chassis_.set_parking_brake(true);
    } else {
      chassis_.set_parking_brake(false);
    }
  } else {
    chassis_.set_parking_brake(false);
  }

  if (chassis_error_mask_) {
    chassis_.set_chassis_error_mask(chassis_error_mask_);
  }

  // give engage_advice based on error_code and canbus feedback
  if (!chassis_error_mask_ && !chassis_.parking_brake() &&
      (chassis_.throttle_percentage() == 0.0)) {
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

void Neolix_eduController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode Neolix_eduController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  // set enable
  ads_brake_command_46_->set_drive_enable(true);
  ads_drive_command_50_->set_drive_enable(true);
  ads_eps_command_56_->set_drive_enable(true);

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

ErrorCode Neolix_eduController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode Neolix_eduController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode.";
    return ErrorCode::OK;
  }
  AFATAL << "SpeedOnlyMode is not supported in Neolix_edu!";
  return ErrorCode::OK;
}

ErrorCode Neolix_eduController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  AFATAL << "SpeedOnlyMode is not supported in Neolix_edu!";
  return ErrorCode::OK;
}

// NEUTRAL, REVERSE, DRIVE
void Neolix_eduController::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }
  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      ads_drive_command_50_->set_auto_shift_command(
          Ads_drive_command_50::AUTO_SHIFT_COMMAND_N);
      ads_brake_command_46_->set_auto_parking_command(false);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      ads_drive_command_50_->set_auto_shift_command(
          Ads_drive_command_50::AUTO_SHIFT_COMMAND_R);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      ads_drive_command_50_->set_auto_shift_command(
          Ads_drive_command_50::AUTO_SHIFT_COMMAND_D);
      break;
    }
    case Chassis::GEAR_PARKING: {
      ads_brake_command_46_->set_auto_parking_command(true);
      break;
    }
    default:
      break;
  }
}

// brake with pedal
// pedal:0.00~99.99, unit:
void Neolix_eduController::Brake(double pedal) {
  // TODO(All) :  Update brake value based on mode
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set brake pedal.";
    return;
  }
  ads_brake_command_46_->set_auto_brake_command(pedal);
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void Neolix_eduController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  ads_drive_command_50_->set_auto_drive_torque(pedal / 2);
}

// confirm the car is driven by acceleration command or throttle/brake pedal
// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void Neolix_eduController::Acceleration(double acc) {
  // None
}

// neolix_edu default, -380 ~ 380, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void Neolix_eduController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  ads_eps_command_56_->set_auto_target_angle(real_angle);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void Neolix_eduController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  ads_eps_command_56_->set_auto_target_angle(real_angle);
}

void Neolix_eduController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    ads_brake_command_46_->set_auto_parking_command(true);
  } else {
    ads_brake_command_46_->set_auto_parking_command(false);
  }
}

void Neolix_eduController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void Neolix_eduController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void Neolix_eduController::SetTurningSignal(const ControlCommand& command) {
  // None
}

void Neolix_eduController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool Neolix_eduController::CheckChassisError() {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
   */
  return false;
}

void Neolix_eduController::SecurityDogThreadFunc() {
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
      AERROR << "Too much time consumption in Neolix_eduController looping "
                "process:"
             << elapsed.count();
    }
  }
}

bool Neolix_eduController::CheckResponse(const int32_t flags, bool need_wait) {
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

void Neolix_eduController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t Neolix_eduController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode Neolix_eduController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void Neolix_eduController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace neolix_edu
}  // namespace canbus
}  // namespace apollo
