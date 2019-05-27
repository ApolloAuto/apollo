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

#include "modules/canbus/vehicle/lincoln/lincoln_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/lincoln/lincoln_message_manager.h"
#include "modules/canbus/vehicle/lincoln/protocol/brake_60.h"
#include "modules/canbus/vehicle/lincoln/protocol/gear_66.h"
#include "modules/canbus/vehicle/lincoln/protocol/steering_64.h"
#include "modules/canbus/vehicle/lincoln/protocol/throttle_62.h"
#include "modules/canbus/vehicle/lincoln/protocol/turnsignal_68.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/kv_db/kv_db.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace lincoln {

using ::apollo::drivers::canbus::ProtocolData;
using common::ErrorCode;
using control::ControlCommand;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;

}  // namespace

ErrorCode LincolnController::Init(
    const VehicleParameter &params,
    CanSender<::apollo::canbus::ChassisDetail> *const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail> *const message_manager) {
  if (is_initialized_) {
    AINFO << "LincolnController has already been initiated.";
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
  brake_60_ = dynamic_cast<Brake60 *>(
      message_manager_->GetMutableProtocolDataById(Brake60::ID));
  if (brake_60_ == nullptr) {
    AERROR << "Brake60 does not exist in the LincolnMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  throttle_62_ = dynamic_cast<Throttle62 *>(
      message_manager_->GetMutableProtocolDataById(Throttle62::ID));
  if (throttle_62_ == nullptr) {
    AERROR << "Throttle62 does not exist in the LincolnMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  steering_64_ = dynamic_cast<Steering64 *>(
      message_manager_->GetMutableProtocolDataById(Steering64::ID));
  if (steering_64_ == nullptr) {
    AERROR << "Steering64 does not exist in the LincolnMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  gear_66_ = dynamic_cast<Gear66 *>(
      message_manager_->GetMutableProtocolDataById(Gear66::ID));
  if (gear_66_ == nullptr) {
    AERROR << "Gear66 does not exist in the LincolnMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }
  turnsignal_68_ = dynamic_cast<Turnsignal68 *>(
      message_manager_->GetMutableProtocolDataById(Turnsignal68::ID));
  if (turnsignal_68_ == nullptr) {
    AERROR << "Turnsignal68 does not exist in the LincolnMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Brake60::ID, brake_60_, false);
  can_sender_->AddMessage(Throttle62::ID, throttle_62_, false);
  can_sender_->AddMessage(Steering64::ID, steering_64_, false);
  can_sender_->AddMessage(Gear66::ID, gear_66_, false);
  can_sender_->AddMessage(Turnsignal68::ID, turnsignal_68_, false);

  // Need to sleep to ensure all messages received
  AINFO << "Controller is initialized.";

  gear_tmp_ = Chassis::GEAR_INVALID;
  is_initialized_ = true;
  return ErrorCode::OK;
}

bool LincolnController::Start() {
  if (!is_initialized_) {
    AERROR << "LincolnController has NOT been initiated.";
    return false;
  }
  const auto &update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void LincolnController::Stop() {
  if (!is_initialized_) {
    AERROR << "LincolnController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "LincolnController stopped.";
  }
}

Chassis LincolnController::chassis() {
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
  // 4
  if (chassis_detail.has_ems() && chassis_detail.ems().has_engine_rpm()) {
    chassis_.set_engine_rpm(
        static_cast<float>(chassis_detail.ems().engine_rpm()));
  } else {
    chassis_.set_engine_rpm(0);
  }
  // 5
  if (chassis_detail.has_vehicle_spd() &&
      chassis_detail.vehicle_spd().has_vehicle_spd()) {
    chassis_.set_speed_mps(
        static_cast<float>(chassis_detail.vehicle_spd().vehicle_spd()));
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rr_valid(
        chassis_detail.vehicle_spd().is_wheel_spd_rr_valid());
    chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
        chassis_detail.vehicle_spd().wheel_direction_rr());
    chassis_.mutable_wheel_speed()->set_wheel_spd_rr(
        chassis_detail.vehicle_spd().wheel_spd_rr());

    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rl_valid(
        chassis_detail.vehicle_spd().is_wheel_spd_rl_valid());
    chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
        chassis_detail.vehicle_spd().wheel_direction_rl());
    chassis_.mutable_wheel_speed()->set_wheel_spd_rl(
        chassis_detail.vehicle_spd().wheel_spd_rl());

    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fr_valid(
        chassis_detail.vehicle_spd().is_wheel_spd_fr_valid());
    chassis_.mutable_wheel_speed()->set_wheel_direction_fr(
        chassis_detail.vehicle_spd().wheel_direction_fr());
    chassis_.mutable_wheel_speed()->set_wheel_spd_fr(
        chassis_detail.vehicle_spd().wheel_spd_fr());

    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fl_valid(
        chassis_detail.vehicle_spd().is_wheel_spd_fl_valid());
    chassis_.mutable_wheel_speed()->set_wheel_direction_fl(
        chassis_detail.vehicle_spd().wheel_direction_fl());
    chassis_.mutable_wheel_speed()->set_wheel_spd_fl(
        chassis_detail.vehicle_spd().wheel_spd_fl());

  } else {
    chassis_.set_speed_mps(0);
  }
  // 6
  if (chassis_detail.has_basic() && chassis_detail.basic().has_odo_meter()) {
    // odo_meter is in km
    chassis_.set_odometer_m(
        static_cast<float>(chassis_detail.basic().odo_meter() * 1000.0));
  } else {
    chassis_.set_odometer_m(0);
  }

  // 7
  // lincoln only has fuel percentage
  // to avoid confusing, just don't set
  chassis_.set_fuel_range_m(0);
  // 8
  if (chassis_detail.has_gas() && chassis_detail.gas().has_throttle_output()) {
    chassis_.set_throttle_percentage(
        static_cast<float>(chassis_detail.gas().throttle_output()));
  } else {
    chassis_.set_throttle_percentage(0);
  }
  // 9
  if (chassis_detail.has_brake() && chassis_detail.brake().has_brake_output()) {
    chassis_.set_brake_percentage(
        static_cast<float>(chassis_detail.brake().brake_output()));
  } else {
    chassis_.set_brake_percentage(0);
  }
  // 23, previously 10
  if (chassis_detail.has_gear() && chassis_detail.gear().has_gear_state()) {
    chassis_.set_gear_location(chassis_detail.gear().gear_state());
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }
  // 11
  if (chassis_detail.has_eps() && chassis_detail.eps().has_steering_angle()) {
    chassis_.set_steering_percentage(
        static_cast<float>(chassis_detail.eps().steering_angle() * 100.0 /
                           vehicle_params_.max_steer_angle() * M_PI / 180.0));
  } else {
    chassis_.set_steering_percentage(0);
  }
  // 12
  if (chassis_detail.has_eps() && chassis_detail.eps().has_epas_torque()) {
    chassis_.set_steering_torque_nm(
        static_cast<float>(chassis_detail.eps().epas_torque()));
  } else {
    chassis_.set_steering_torque_nm(0);
  }
  // 13
  if (chassis_detail.has_eps() &&
      chassis_detail.epb().has_parking_brake_status()) {
    chassis_.set_parking_brake(chassis_detail.epb().parking_brake_status() ==
                               Epb::PBRAKE_ON);
  } else {
    chassis_.set_parking_brake(false);
  }

  // 14, 15
  if (chassis_detail.has_light() &&
      chassis_detail.light().has_lincoln_lamp_type()) {
    chassis_.mutable_signal()->set_high_beam(
        chassis_detail.light().lincoln_lamp_type() == Light::BEAM_HIGH);
  } else {
    chassis_.mutable_signal()->set_high_beam(false);
  }

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
  // 18
  if (chassis_detail.has_light() && chassis_detail.light().has_is_horn_on() &&
      chassis_detail.light().is_horn_on()) {
    chassis_.mutable_signal()->set_horn(true);
  } else {
    chassis_.mutable_signal()->set_horn(false);
  }

  // 19, lincoln wiper is too complicated
  // 24
  if (chassis_detail.has_eps() && chassis_detail.eps().has_timestamp_65()) {
    chassis_.set_steering_timestamp(chassis_detail.eps().timestamp_65());
  }
  // 26
  if (chassis_error_mask_) {
    chassis_.set_chassis_error_mask(chassis_error_mask_);
  }

  // 6d, 6e, 6f, if gps valid is available, assume all gps related field
  // available
  if (chassis_detail.basic().has_gps_valid()) {
    chassis_.mutable_chassis_gps()->set_latitude(
        chassis_detail.basic().latitude());
    chassis_.mutable_chassis_gps()->set_longitude(
        chassis_detail.basic().longitude());
    chassis_.mutable_chassis_gps()->set_gps_valid(
        chassis_detail.basic().gps_valid());
    chassis_.mutable_chassis_gps()->set_year(chassis_detail.basic().year());
    chassis_.mutable_chassis_gps()->set_month(chassis_detail.basic().month());
    chassis_.mutable_chassis_gps()->set_day(chassis_detail.basic().day());
    chassis_.mutable_chassis_gps()->set_hours(chassis_detail.basic().hours());
    chassis_.mutable_chassis_gps()->set_minutes(
        chassis_detail.basic().minutes());
    chassis_.mutable_chassis_gps()->set_seconds(
        chassis_detail.basic().seconds());
    chassis_.mutable_chassis_gps()->set_compass_direction(
        chassis_detail.basic().compass_direction());
    chassis_.mutable_chassis_gps()->set_pdop(chassis_detail.basic().pdop());
    chassis_.mutable_chassis_gps()->set_is_gps_fault(
        chassis_detail.basic().is_gps_fault());
    chassis_.mutable_chassis_gps()->set_is_inferred(
        chassis_detail.basic().is_inferred());
    chassis_.mutable_chassis_gps()->set_altitude(
        chassis_detail.basic().altitude());
    chassis_.mutable_chassis_gps()->set_heading(
        chassis_detail.basic().heading());
    chassis_.mutable_chassis_gps()->set_hdop(chassis_detail.basic().hdop());
    chassis_.mutable_chassis_gps()->set_vdop(chassis_detail.basic().vdop());
    chassis_.mutable_chassis_gps()->set_quality(
        chassis_detail.basic().quality());
    chassis_.mutable_chassis_gps()->set_num_satellites(
        chassis_detail.basic().num_satellites());
    chassis_.mutable_chassis_gps()->set_gps_speed(
        chassis_detail.basic().gps_speed());
  } else {
    chassis_.mutable_chassis_gps()->set_gps_valid(false);
  }

  // VIN number will be written into KVDB once.
  if (chassis_detail.vehicle_id().has_vin()) {
    chassis_.mutable_vehicle_id()->set_vin(chassis_detail.vehicle_id().vin());
    if (!received_vin_) {
      apollo::common::KVDB::Put("apollo:canbus:vin",
                                chassis_detail.vehicle_id().vin());
      received_vin_ = true;
    }
  }

  if (chassis_detail.has_surround()) {
    chassis_.mutable_surround()->CopyFrom(chassis_detail.surround());
  }
  // Give engage_advice based on error_code and canbus feedback
  if (chassis_error_mask_ || chassis_.throttle_percentage() == 0.0 ||
      chassis_.brake_percentage() == 0.0) {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    chassis_.mutable_engage_advice()->set_reason("Chassis error!");
  } else if (chassis_.parking_brake() || CheckSafetyError(chassis_detail)) {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::DISALLOW_ENGAGE);
    chassis_.mutable_engage_advice()->set_reason(
        "Vehicle is not in a safe state to engage!");
  } else {
    chassis_.mutable_engage_advice()->set_advice(
        apollo::common::EngageAdvice::READY_TO_ENGAGE);
  }
  return chassis_;
}

void LincolnController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
  if (chassis_error_code() == Chassis::NO_ERROR) {
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
  }
}

ErrorCode LincolnController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "Already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  brake_60_->set_enable();
  throttle_62_->set_enable();
  steering_64_->set_enable();

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    CheckChassisError();
    Emergency();
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
  AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
  return ErrorCode::OK;
}

ErrorCode LincolnController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode LincolnController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode";
    return ErrorCode::OK;
  }
  brake_60_->set_disable();
  throttle_62_->set_disable();
  steering_64_->set_enable();

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    CheckChassisError();
    Emergency();
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_STEER_ONLY);
  AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
  return ErrorCode::OK;
}

ErrorCode LincolnController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  brake_60_->set_enable();
  throttle_62_->set_enable();
  steering_64_->set_disable();

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true)) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    CheckChassisError();
    Emergency();
    return ErrorCode::CANBUS_ERROR;
  }
  set_driving_mode(Chassis::AUTO_SPEED_ONLY);
  AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
  return ErrorCode::OK;
}

// NEUTRAL, REVERSE, DRIVE
void LincolnController::Gear(Chassis::GearPosition ref_gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }

  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  const Chassis::GearPosition current_gear_position =
      chassis_detail.gear().gear_state();
  if (ref_gear_position == current_gear_position) {
    return;
  }

  // Need to request neutral gear first if current gear location is not neutral
  // or none
  if (ref_gear_position != current_gear_position &&
      current_gear_position != Chassis::GEAR_NEUTRAL &&
      current_gear_position != Chassis::GEAR_NONE) {
    gear_tmp_ = Chassis::GEAR_NEUTRAL;
  } else if (current_gear_position == Chassis::GEAR_NEUTRAL) {
    gear_tmp_ = ref_gear_position;
  }

  switch (gear_tmp_) {
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
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:%
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void LincolnController::Brake(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  brake_60_->set_pedal(pedal);
}

// drive with old acceleration
// gas:0.00~99.99 unit:%
void LincolnController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  throttle_62_->set_pedal(pedal);
}

// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void LincolnController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  // None
}

// lincoln default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:%, left:-, right:+
void LincolnController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  // reverse sign
  steering_64_->set_steering_angle(real_angle)->set_steering_angle_speed(200);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:%, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void LincolnController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;
  const double real_angle_spd =
      ProtocolData<::apollo::canbus::ChassisDetail>::BoundedValue(
          vehicle_params_.min_steer_angle_rate() / M_PI * 180,
          vehicle_params_.max_steer_angle_rate() / M_PI * 180,
          vehicle_params_.max_steer_angle_rate() / M_PI * 180 * angle_spd /
              100.0);
  steering_64_->set_steering_angle(real_angle)
      ->set_steering_angle_speed(real_angle_spd);
}

void LincolnController::SetEpbBreak(const ControlCommand &command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void LincolnController::SetBeam(const ControlCommand &command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void LincolnController::SetHorn(const ControlCommand &command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void LincolnController::SetTurningSignal(const ControlCommand &command) {
  // Set Turn Signal
  auto signal = command.signal().turn_signal();
  if (signal == common::VehicleSignal::TURN_LEFT) {
    turnsignal_68_->set_turn_left();
  } else if (signal == common::VehicleSignal::TURN_RIGHT) {
    turnsignal_68_->set_turn_right();
  } else {
    turnsignal_68_->set_turn_none();
  }
}

void LincolnController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool LincolnController::CheckChassisError() {
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  int32_t error_cnt = 0;
  int32_t chassis_error_mask = 0;

  // Steer fault
  bool steer_fault = chassis_detail.eps().watchdog_fault() |
                     chassis_detail.eps().channel_1_fault() |
                     chassis_detail.eps().channel_2_fault() |
                     chassis_detail.eps().calibration_fault() |
                     chassis_detail.eps().connector_fault();

  chassis_error_mask |=
      ((chassis_detail.eps().watchdog_fault()) << (++error_cnt));
  chassis_error_mask |=
      ((chassis_detail.eps().channel_1_fault()) << (++error_cnt));
  chassis_error_mask |=
      ((chassis_detail.eps().channel_2_fault()) << (++error_cnt));
  chassis_error_mask |=
      ((chassis_detail.eps().calibration_fault()) << (++error_cnt));
  chassis_error_mask |=
      ((chassis_detail.eps().connector_fault()) << (++error_cnt));

  if (!chassis_detail.has_brake()) {
    AERROR_EVERY(100) << "ChassisDetail has NO brake."
                      << chassis_detail.DebugString();
    return false;
  }
  // Brake fault
  bool brake_fault = chassis_detail.brake().watchdog_fault() |
                     chassis_detail.brake().channel_1_fault() |
                     chassis_detail.brake().channel_2_fault() |
                     chassis_detail.brake().boo_fault() |
                     chassis_detail.brake().connector_fault();

  chassis_error_mask |=
      ((chassis_detail.brake().watchdog_fault()) << (++error_cnt));
  chassis_error_mask |=
      ((chassis_detail.brake().channel_1_fault()) << (++error_cnt));
  chassis_error_mask |=
      ((chassis_detail.brake().channel_2_fault()) << (++error_cnt));
  chassis_error_mask |= ((chassis_detail.brake().boo_fault()) << (++error_cnt));
  chassis_error_mask |=
      ((chassis_detail.brake().connector_fault()) << (++error_cnt));

  if (!chassis_detail.has_gas()) {
    AERROR_EVERY(100) << "ChassisDetail has NO gas."
                      << chassis_detail.DebugString();
    return false;
  }
  // Throttle fault
  bool throttle_fault = chassis_detail.gas().watchdog_fault() |
                        chassis_detail.gas().channel_1_fault() |
                        chassis_detail.gas().channel_2_fault() |
                        chassis_detail.gas().connector_fault();

  chassis_error_mask |=
      ((chassis_detail.gas().watchdog_fault()) << (++error_cnt));
  chassis_error_mask |=
      ((chassis_detail.gas().channel_1_fault()) << (++error_cnt));
  chassis_error_mask |=
      ((chassis_detail.gas().channel_2_fault()) << (++error_cnt));
  chassis_error_mask |=
      ((chassis_detail.gas().connector_fault()) << (++error_cnt));

  if (!chassis_detail.has_gear()) {
    AERROR_EVERY(100) << "ChassisDetail has NO gear."
                      << chassis_detail.DebugString();
    return false;
  }
  // Gear fault
  bool gear_fault = chassis_detail.gear().canbus_fault();

  chassis_error_mask |=
      ((chassis_detail.gear().canbus_fault()) << (++error_cnt));

  set_chassis_error_mask(chassis_error_mask);

  if (steer_fault) {
    set_chassis_error_code(Chassis::CHASSIS_ERROR_ON_STEER);
    AERROR_EVERY(100) << "Steering fault detected: "
                      << chassis_detail.eps().watchdog_fault() << ", "
                      << chassis_detail.eps().channel_1_fault() << ", "
                      << chassis_detail.eps().channel_2_fault() << ", "
                      << chassis_detail.eps().calibration_fault() << ", "
                      << chassis_detail.eps().connector_fault();
  }

  if (brake_fault) {
    set_chassis_error_code(Chassis::CHASSIS_ERROR_ON_BRAKE);
    AERROR_EVERY(100) << "Brake fault detected: "
                      << chassis_detail.brake().watchdog_fault() << ", "
                      << chassis_detail.brake().channel_1_fault() << ", "
                      << chassis_detail.brake().channel_2_fault() << ", "
                      << chassis_detail.brake().boo_fault() << ", "
                      << chassis_detail.brake().connector_fault();
  }

  if (throttle_fault) {
    set_chassis_error_code(Chassis::CHASSIS_ERROR_ON_THROTTLE);
    AERROR_EVERY(100) << "Throttle fault detected: "
                      << chassis_detail.gas().watchdog_fault() << ", "
                      << chassis_detail.gas().channel_1_fault() << ", "
                      << chassis_detail.gas().channel_2_fault() << ", "
                      << chassis_detail.gas().connector_fault();
  }

  if (gear_fault) {
    set_chassis_error_code(Chassis::CHASSIS_ERROR_ON_GEAR);
    AERROR_EVERY(100) << "Gear fault detected: "
                      << chassis_detail.gear().canbus_fault();
  }

  if (steer_fault || brake_fault || throttle_fault) {
    return true;
  }

  return false;
}

void LincolnController::SecurityDogThreadFunc() {
  if (can_sender_ == nullptr) {
    AERROR << "Failed to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start =
      common::time::AsInt64<common::time::micros>(common::time::Clock::Now());

  int32_t speed_ctrl_fail = 0;
  int32_t steer_ctrl_fail = 0;

  while (can_sender_->IsRunning()) {
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. Steer control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false)) {
      ++steer_ctrl_fail;
      if (steer_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      steer_ctrl_fail = 0;
    }

    // 2. Speed control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        !CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false)) {
      ++speed_ctrl_fail;
      if (speed_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      speed_ctrl_fail = 0;
    }
    if (CheckChassisError()) {
      emergency_mode = true;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      Emergency();
    }
    int64_t end =
        common::time::AsInt64<common::time::micros>(common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
      start = common::time::AsInt64<common::time::micros>(
          common::time::Clock::Now());
    } else {
      AERROR_EVERY(100)
          << "Too much time consumption in LincolnController looping process:"
          << elapsed.count();
      start = end;
    }
  }
}

bool LincolnController::CheckResponse(const int32_t flags, bool need_wait) {
  // for Lincoln, CheckResponse commonly takes 300ms. We leave a 100ms buffer
  // for it.
  int32_t retry_num = 20;
  ChassisDetail chassis_detail;
  bool is_eps_online = false;
  bool is_vcu_online = false;
  bool is_esp_online = false;

  do {
    if (message_manager_->GetSensorData(&chassis_detail) != ErrorCode::OK) {
      AERROR_EVERY(100) << "Get chassis detail failed.";
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
    }
    AINFO << "Need to check response again.";
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

void LincolnController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t LincolnController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode LincolnController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void LincolnController::set_chassis_error_code(
    const Chassis::ErrorCode &error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

bool LincolnController::CheckSafetyError(
    const ::apollo::canbus::ChassisDetail &chassis_detail) {
  bool safety_error =
      chassis_detail.safety().is_passenger_door_open() ||
      chassis_detail.safety().is_rearleft_door_open() ||
      chassis_detail.safety().is_rearright_door_open() ||
      chassis_detail.safety().is_hood_open() ||
      chassis_detail.safety().is_trunk_open() ||
      (chassis_detail.safety().is_passenger_detected() &&
       (!chassis_detail.safety().is_passenger_airbag_enabled() ||
        !chassis_detail.safety().is_passenger_buckled()));
  ADEBUG << "Vehicle safety error status is : " << safety_error;
  return safety_error;
}

}  // namespace lincoln
}  // namespace canbus
}  // namespace apollo
