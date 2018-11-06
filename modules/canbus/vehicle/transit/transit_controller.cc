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

#include "modules/canbus/vehicle/transit/transit_controller.h"

#include "modules/common/proto/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "modules/canbus/vehicle/transit/transit_message_manager.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace transit {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}  // namespace

ErrorCode TransitController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "TransitController has already been initiated.";
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
  adc_auxiliarycontrol_110_ = dynamic_cast<Adcauxiliarycontrol110*>(
      message_manager_->GetMutableProtocolDataById(Adcauxiliarycontrol110::ID));
  if (adc_auxiliarycontrol_110_ == nullptr) {
    AERROR << "Adcauxiliarycontrol110 does not exist in the "
              "TransitMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  adc_motioncontrol1_10_ = dynamic_cast<Adcmotioncontrol110*>(
      message_manager_->GetMutableProtocolDataById(Adcmotioncontrol110::ID));
  if (adc_motioncontrol1_10_ == nullptr) {
    AERROR
        << "Adcmotioncontrol110 does not exist in the TransitMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  adc_motioncontrollimits1_12_ = dynamic_cast<Adcmotioncontrollimits112*>(
      message_manager_->GetMutableProtocolDataById(
          Adcmotioncontrollimits112::ID));
  if (adc_motioncontrollimits1_12_ == nullptr) {
    AERROR << "Adcmotioncontrollimits112 does not exist in the "
              "TransitMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  llc_diag_brakecontrol_721_ = dynamic_cast<Llcdiagbrakecontrol721*>(
      message_manager_->GetMutableProtocolDataById(Llcdiagbrakecontrol721::ID));
  if (llc_diag_brakecontrol_721_ == nullptr) {
    AERROR << "Llcdiagbrakecontrol721 does not exist in the "
              "TransitMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  llc_diag_steeringcontrol_722_ = dynamic_cast<Llcdiagsteeringcontrol722*>(
      message_manager_->GetMutableProtocolDataById(
          Llcdiagsteeringcontrol722::ID));
  if (llc_diag_steeringcontrol_722_ == nullptr) {
    AERROR << "Llcdiagsteeringcontrol722 does not exist in the "
              "TransitMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Adcauxiliarycontrol110::ID, adc_auxiliarycontrol_110_,
                          false);
  can_sender_->AddMessage(Adcmotioncontrol110::ID, adc_motioncontrol1_10_,
                          false);
  can_sender_->AddMessage(Adcmotioncontrollimits112::ID,
                          adc_motioncontrollimits1_12_, false);
  can_sender_->AddMessage(Llcdiagbrakecontrol721::ID,
                          llc_diag_brakecontrol_721_, false);
  can_sender_->AddMessage(Llcdiagsteeringcontrol722::ID,
                          llc_diag_steeringcontrol_722_, false);

  // need sleep to ensure all messages received
  AINFO << "TransitController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

TransitController::~TransitController() {}

bool TransitController::Start() {
  if (!is_initialized_) {
    AERROR << "TransitController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void TransitController::Stop() {
  if (!is_initialized_) {
    AERROR << "TransitController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "TransitController stopped.";
  }
}

Chassis TransitController::chassis() {
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
    chassis_.set_engine_rpm(chassis_detail.ems().engine_rpm());
  } else {
    chassis_.set_engine_rpm(0);
  }
  // 5
  if (chassis_detail.has_vehicle_spd() &&
      chassis_detail.vehicle_spd().has_vehicle_spd()) {
    chassis_.set_speed_mps(chassis_detail.vehicle_spd().vehicle_spd());
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
    chassis_.set_odometer_m(chassis_detail.basic().odo_meter() * 1000);
  } else {
    chassis_.set_odometer_m(0);
  }

  // 7
  // lincoln only has fuel percentage
  // to avoid confusing, just don't set
  chassis_.set_fuel_range_m(0);
  // 8
  if (chassis_detail.has_gas() && chassis_detail.gas().has_throttle_output()) {
    chassis_.set_throttle_percentage(chassis_detail.gas().throttle_output());
  } else {
    chassis_.set_throttle_percentage(0);
  }
  // 9
  if (chassis_detail.has_brake() && chassis_detail.brake().has_brake_output()) {
    chassis_.set_brake_percentage(chassis_detail.brake().brake_output());
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
    chassis_.set_steering_percentage(chassis_detail.eps().steering_angle() *
                                     100.0 / vehicle_params_.max_steer_angle() *
                                     M_PI / 180.0);
  } else {
    chassis_.set_steering_percentage(0);
  }
  // 12
  if (chassis_detail.has_eps() && chassis_detail.eps().has_epas_torque()) {
    chassis_.set_steering_torque_nm(chassis_detail.eps().epas_torque());
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

  // 24
  if (chassis_detail.has_eps() && chassis_detail.eps().has_timestamp_65()) {
    chassis_.set_steering_timestamp(chassis_detail.eps().timestamp_65());
  }
  // 26
  if (chassis_error_mask_) {
    chassis_.set_chassis_error_mask(chassis_error_mask_);
  }

  // 6d, 6e, 6f, if gps valid is availiable, assume all gps related field
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

  // vin number will be written into KVDB once.
  if (chassis_detail.license().has_vin()) {
    chassis_.mutable_license()->set_vin(chassis_detail.license().vin());
    if (!received_vin_) {
      apollo::common::KVDB::Put("apollo:canbus:vin",
                                chassis_detail.license().vin());
      received_vin_ = true;
    }
  }

  if (chassis_detail.has_surround()) {
    chassis_.mutable_surround()->CopyFrom(chassis_detail.surround());
  }
  // give engage_advice based on error_code and canbus feedback
  if (chassis_error_mask_ || (chassis_.throttle_percentage() == 0.0) ||
      (chassis_.brake_percentage() == 0.0)) {
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

void TransitController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode TransitController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  adc_motioncontrol1_10_->
    set_adc_cmd_autonomyrequest(
    Adc_motioncontrol1_10::ADC_CMD_AUTONOMYREQUEST_AUTONOMY_REQUESTED);
  adc_motioncontrol1_10_->
    set_adc_cmd_steeringcontrolmode(
    Adc_motioncontrol1_10::ADC_CMD_STEERINGCONTROLMODE_ANGLE);
  adc_motioncontrol1_10_->
    set_adc_cmd_longitudinalcontrolmode(
    Adc_motioncontrol1_10::ADC_CMD_LONGITUDINALCONTROLMODE_DIRECT_THROTTLE_BRAKE
    );
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

ErrorCode TransitController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode TransitController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode";
    return ErrorCode::OK;
  }
  adc_motioncontrol1_10_->
    set_adc_cmd_autonomyrequest(
    Adc_motioncontrol1_10::ADC_CMD_AUTONOMYREQUEST_AUTONOMY_REQUESTED);
  adc_motioncontrol1_10_->
    set_adc_cmd_steeringcontrolmode(
    Adc_motioncontrol1_10::ADC_CMD_STEERINGCONTROLMODE_ANGLE);
  adc_motioncontrol1_10_->
    set_adc_cmd_longitudinalcontrolmode(
    Adc_motioncontrol1_10::ADC_CMD_LONGITUDINALCONTROLMODE_NONE);
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

ErrorCode TransitController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  adc_motioncontrol1_10_->
    set_adc_cmd_autonomyrequest(
    Adc_motioncontrol1_10::ADC_CMD_AUTONOMYREQUEST_AUTONOMY_REQUESTED);
  adc_motioncontrol1_10_->
    set_adc_cmd_steeringcontrolmode(
    Adc_motioncontrol1_10::ADC_CMD_STEERINGCONTROLMODE_NONE);
  adc_motioncontrol1_10_->
    set_adc_cmd_longitudinalcontrolmode(
    Adc_motioncontrol1_10::ADC_CMD_LONGITUDINALCONTROLMODE_DIRECT_THROTTLE_BRAKE
    );
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
void TransitController::Gear(Chassis::GearPosition gear_position) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "this drive mode no need to set gear.";
    return;
  }
  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      adc_motioncontrol1_10_.set_adc_cmd_gear(
        Adc_motioncontrol1_10::ADC_CMD_GEAR_N_NEUTRAL);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      adc_motioncontrol1_10_.set_adc_cmd_gear(
        Adc_motioncontrol1_10::ADC_CMD_GEAR_R_REVERSE);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      adc_motioncontrol1_10_.set_adc_cmd_gear(
        Adc_motioncontrol1_10::ADC_CMD_GEAR_D_DRIVE);
      break;
    }
    case Chassis::GEAR_PARKING: {
      adc_motioncontrol1_10_.set_adc_cmd_gear(
        Adc_motioncontrol1_10::ADC_CMD_GEAR_P_PARKING);
      break;
    }
    case Chassis::GEAR_LOW: {
      adc_motioncontrol1_10_.set_adc_cmd_gear(
        Adc_motioncontrol1_10::ADC_CMD_GEAR_D_DRIVE);
      break;
    }
    case Chassis::GEAR_NONE: {
      adc_motioncontrol1_10_.set_adc_cmd_gear(
        Adc_motioncontrol1_10::ADC_CMD_GEAR_N_NEUTRAL);
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      adc_motioncontrol1_10_.set_adc_cmd_gear(
        Adc_motioncontrol1_10::ADC_CMD_GEAR_N_NEUTRAL);
      break;
    }
    default: {
      adc_motioncontrol1_10_.set_adc_cmd_gear(
        Adc_motioncontrol1_10::ADC_CMD_GEAR_N_NEUTRAL);
      break;
    }
  }
  return;
}

// brake with new acceleration
// acceleration:0.00~99.99, unit:
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void TransitController::Brake(double pedal) {
  // double real_value = params_.max_acc() * acceleration / 100;
  // TODO(QiL) :  Update brake value based on mode
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  adc_motioncontrol1_10_.set_adc_cmd_brakepressure(pedal);
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void TransitController::Throttle(double pedal) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  adc_motioncontrol1_10_.set_adc_cmd_throttleposition(pedal);
  adc_motioncontrollimits1_12_.set_adc_cmd_steeringrate(200);
}

// transit default, -470 ~ 470, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void TransitController::Steer(double angle) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle = params_.max_steer_angle() * angle / 100.0;
  const double real_angle_spd = ProtocolData::BoundedValue(
      params_.min_steer_angle_spd(), params_.max_steer_angle_spd(),
      params_.max_steer_angle_spd() * angle_spd / 100.0);
  adc_motioncontrol1_10_.set_adc_cmd_steerwheelangle(real_angle);
  adc_motioncontrollimits1_12_.set_adc_cmd_steeringrate(real_angle_spd);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void TransitController::Steer(double angle, double angle_spd) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle = params_.max_steer_angle() * angle / 100.0;
  adc_motioncontrol1_10_.set_adc_cmd_steerwheelangle(real_angle);
}

void TransitController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    adc_motioncontrol1_10_.set_adc_cmd_parkingbrake(true);
  } else {
    adc_motioncontrol1_10_.set_adc_cmd_parkingbrake(false);
  }
}

void TransitController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    adc_auxiliarycontrol_110_->set_adc_cmd_highbeam(true);
  } else if (command.signal().low_beam()) {
    adc_auxiliarycontrol_110_->set_adc_cmd_lowbeaml(true);
  } else {
    adc_auxiliarycontrol_110_->set_adc_cmd_highbeam(false);
    adc_auxiliarycontrol_110_->set_adc_cmd_lowbeaml(false);
  }
}

void TransitController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    adc_auxiliarycontrol_110_->set_adc_cmd_horn(true);
  } else {
    adc_auxiliarycontrol_110_->set_adc_cmd_horn(false);
  }
}

void TransitController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  auto signal = command.signal().turn_signal();
  if (signal == Signal::TURN_LEFT) {
    adc_auxiliarycontrol_110_->set_adc_cmd_turnsignal(
      Adc_auxiliarycontrol_110::ADC_CMD_TURNSIGNAL_LEFT);
  } else if (signal == Signal::TURN_RIGHT) {
    adc_auxiliarycontrol_110_->set_adc_cmd_turnsignal(
      Adc_auxiliarycontrol_110::ADC_CMD_TURNSIGNAL_RIGHT);
  } else {
    adc_auxiliarycontrol_110_->set_adc_cmd_turnsignal(
      Adc_auxiliarycontrol_110::ADC_CMD_TURNSIGNAL_NONE);
  }
}

void TransitController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool TransitController::CheckChassisError() {
  // steer fault
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);

  int32_t error_cnt = 0;
  int32_t chassis_error_mask = 0;
  if (!chassis_detail.has_eps()) {
    AERROR_EVERY(100) << "ChassisDetail has NO eps."
                      << chassis_detail.DebugString();
    return false;
  }
  bool steer_fault = chassis_detail.eps().watchdog_fault() |
                     chassis_detail.eps().channel_1_fault() |
                     chassis_detail.eps().channel_2_fault() |
                     chassis_detail.eps().calibration_fault() |
                     chassis_detail.eps().connector_fault();

  chassis_error_mask |=
      ((chassis_detail.eps().watchdog_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.eps().channel_1_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.eps().channel_2_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.eps().calibration_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.eps().connector_fault()) << (error_cnt++));

  if (!chassis_detail.has_brake()) {
    AERROR_EVERY(100) << "ChassisDetail has NO brake."
                      << chassis_detail.DebugString();
    return false;
  }
  // brake fault
  bool brake_fault = chassis_detail.brake().watchdog_fault() |
                     chassis_detail.brake().channel_1_fault() |
                     chassis_detail.brake().channel_2_fault() |
                     chassis_detail.brake().boo_fault() |
                     chassis_detail.brake().connector_fault();

  chassis_error_mask |=
      ((chassis_detail.brake().watchdog_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.brake().channel_1_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.brake().channel_2_fault()) << (error_cnt++));
  chassis_error_mask |= ((chassis_detail.brake().boo_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.brake().connector_fault()) << (error_cnt++));

  if (!chassis_detail.has_gas()) {
    AERROR_EVERY(100) << "ChassisDetail has NO gas."
                      << chassis_detail.DebugString();
    return false;
  }
  // throttle fault
  bool throttle_fault = chassis_detail.gas().watchdog_fault() |
                        chassis_detail.gas().channel_1_fault() |
                        chassis_detail.gas().channel_2_fault() |
                        chassis_detail.gas().connector_fault();

  chassis_error_mask |=
      ((chassis_detail.gas().watchdog_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.gas().channel_1_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.gas().channel_2_fault()) << (error_cnt++));
  chassis_error_mask |=
      ((chassis_detail.gas().connector_fault()) << (error_cnt++));

  if (!chassis_detail.has_gear()) {
    AERROR_EVERY(100) << "ChassisDetail has NO gear."
                      << chassis_detail.DebugString();
    return false;
  }
  // gear fault
  bool gear_fault = chassis_detail.gear().canbus_fault();

  chassis_error_mask |=
      ((chassis_detail.gear().canbus_fault()) << (error_cnt++));

  set_chassis_error_mask(chassis_error_mask);

  if (steer_fault) {
    AERROR_EVERY(100) << "Steering fault detected: "
                      << chassis_detail.eps().watchdog_fault() << ", "
                      << chassis_detail.eps().channel_1_fault() << ", "
                      << chassis_detail.eps().channel_2_fault() << ", "
                      << chassis_detail.eps().calibration_fault() << ", "
                      << chassis_detail.eps().connector_fault();
  }

  if (brake_fault) {
    AERROR_EVERY(100) << "Brake fault detected: "
                      << chassis_detail.brake().watchdog_fault() << ", "
                      << chassis_detail.brake().channel_1_fault() << ", "
                      << chassis_detail.brake().channel_2_fault() << ", "
                      << chassis_detail.brake().boo_fault() << ", "
                      << chassis_detail.brake().connector_fault();
  }

  if (throttle_fault) {
    AERROR_EVERY(100) << "Throttle fault detected: "
                      << chassis_detail.gas().watchdog_fault() << ", "
                      << chassis_detail.gas().channel_1_fault() << ", "
                      << chassis_detail.gas().channel_2_fault() << ", "
                      << chassis_detail.gas().connector_fault();
  }

  if (gear_fault) {
    AERROR_EVERY(100) << "Gear fault detected: "
                      << chassis_detail.gear().canbus_fault();
  }

  if (steer_fault || brake_fault || throttle_fault) {
    return true;
  }

  return false;
}

void TransitController::SecurityDogThreadFunc() {
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
      AERROR
          << "Too much time consumption in TransitController looping process:"
          << elapsed.count();
    }
  }
}

bool TransitController::CheckResponse(const int32_t flags, bool need_wait) {
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

void TransitController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t TransitController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode TransitController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void TransitController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace transit
}  // namespace canbus
}  // namespace apollo
