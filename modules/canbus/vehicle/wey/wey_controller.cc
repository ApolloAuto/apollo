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

#include "modules/canbus/vehicle/wey/wey_controller.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/canbus/vehicle/wey/wey_message_manager.h"
#include "modules/common/proto/vehicle_signal.pb.h"
#include "modules/common/time/time.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace wey {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
double angle_init = 0;
}  // namespace

ErrorCode WeyController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
    MessageManager<::apollo::canbus::ChassisDetail>* const message_manager) {
  if (is_initialized_) {
    AINFO << "WeyController has already been initialized.";
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

  // sender part
  ads1_111_ = dynamic_cast<Ads1111*>(
      message_manager_->GetMutableProtocolDataById(Ads1111::ID));
  if (ads1_111_ == nullptr) {
    AERROR << "Ads1111 does not exist in the WeyMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  ads3_38e_ = dynamic_cast<Ads338e*>(
      message_manager_->GetMutableProtocolDataById(Ads338e::ID));
  if (ads3_38e_ == nullptr) {
    AERROR << "Ads338e does not exist in the WeyMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  ads_eps_113_ = dynamic_cast<Adseps113*>(
      message_manager_->GetMutableProtocolDataById(Adseps113::ID));
  if (ads_eps_113_ == nullptr) {
    AERROR << "Adseps113 does not exist in the WeyMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  ads_req_vin_390_ = dynamic_cast<Adsreqvin390*>(
      message_manager_->GetMutableProtocolDataById(Adsreqvin390::ID));
  if (ads_req_vin_390_ == nullptr) {
    AERROR << "Adsreqvin390 does not exist in the WeyMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  ads_shifter_115_ = dynamic_cast<Adsshifter115*>(
      message_manager_->GetMutableProtocolDataById(Adsshifter115::ID));
  if (ads_shifter_115_ == nullptr) {
    AERROR << "Adsshifter115 does not exist in the WeyMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Ads1111::ID, ads1_111_, false);
  can_sender_->AddMessage(Ads338e::ID, ads3_38e_, false);
  can_sender_->AddMessage(Adseps113::ID, ads_eps_113_, false);
  can_sender_->AddMessage(Adsreqvin390::ID, ads_req_vin_390_, false);
  can_sender_->AddMessage(Adsshifter115::ID, ads_shifter_115_, false);

  // Need to sleep to ensure all messages received
  AINFO << "WeyController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

WeyController::~WeyController() {}

bool WeyController::Start() {
  if (!is_initialized_) {
    AERROR << "WeyController has NOT been initialized.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void WeyController::Stop() {
  if (!is_initialized_) {
    AERROR << "WeyController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "WeyController stopped.";
  }
}

Chassis WeyController::chassis() {
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
  // if there is not wey, no chassis detail can be retrieved and return
  if (!chassis_detail.has_wey()) {
    AERROR << "NO WEY chassis information!";
    return chassis_;
  }
  Wey wey = chassis_detail.wey();

  // 4 engine_rpm
  if (wey.has_fbs3_237() && wey.fbs3_237().has_engspd()) {
    chassis_.set_engine_rpm(static_cast<float>(wey.fbs3_237().engspd()));
  } else {
    chassis_.set_engine_rpm(0);
  }

  // 5 6
  if (wey.has_fbs2_240() && wey.fbs2_240().has_vehiclespd() &&
      wey.has_fbs1_243() && wey.has_status_310()) {
    Fbs2_240 fbs2_240 = wey.fbs2_240();
    Fbs1_243 fbs1_243 = wey.fbs1_243();
    Status_310 status_310 = wey.status_310();
    // speed_mps
    chassis_.set_speed_mps(static_cast<float>(fbs2_240.vehiclespd()));
    // rr
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rr_valid(
        static_cast<bool>(status_310.rrwheelspdvalid()));
    if (fbs2_240.rrwheeldirection() == Fbs2_240::RRWHEELDIRECTION_FORWARD) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
          WheelSpeed::FORWARD);
    } else if (fbs2_240.rrwheeldirection() ==
               Fbs2_240::RRWHEELDIRECTION_BACKWARD) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
          WheelSpeed::BACKWARD);
    } else if (fbs2_240.rrwheeldirection() == Fbs2_240::RRWHEELDIRECTION_STOP) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
          WheelSpeed::STANDSTILL);
    } else {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rr(
          WheelSpeed::INVALID);
    }
    chassis_.mutable_wheel_speed()->set_wheel_spd_rr(fbs2_240.rrwheelspd());
    // rl
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_rl_valid(
        static_cast<bool>(status_310.rlwheelspdvalid()));
    if (fbs2_240.rlwheeldrivedirection() ==
        Fbs2_240::RLWHEELDRIVEDIRECTION_FORWARD) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
          WheelSpeed::FORWARD);
    } else if (fbs2_240.rlwheeldrivedirection() ==
               Fbs2_240::RLWHEELDRIVEDIRECTION_BACKWARD) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
          WheelSpeed::BACKWARD);
    } else if (fbs2_240.rlwheeldrivedirection() ==
               Fbs2_240::RLWHEELDRIVEDIRECTION_STOP) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
          WheelSpeed::STANDSTILL);
    } else {
      chassis_.mutable_wheel_speed()->set_wheel_direction_rl(
          WheelSpeed::INVALID);
    }
    chassis_.mutable_wheel_speed()->set_wheel_spd_rl(fbs2_240.rlwheelspd());
    // fr
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fr_valid(
        static_cast<bool>(status_310.frwheelspdvalid()));
    if (fbs1_243.frwheeldirection() == Fbs1_243::FRWHEELDIRECTION_FORWARD) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fr(
          WheelSpeed::FORWARD);
    } else if (fbs1_243.frwheeldirection() ==
               Fbs1_243::FRWHEELDIRECTION_BACKWARD) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fr(
          WheelSpeed::BACKWARD);
    } else if (fbs1_243.frwheeldirection() == Fbs1_243::FRWHEELDIRECTION_STOP) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fr(
          WheelSpeed::STANDSTILL);
    } else {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fr(
          WheelSpeed::INVALID);
    }
    chassis_.mutable_wheel_speed()->set_wheel_spd_fr(fbs2_240.frwheelspd());
    // fl
    chassis_.mutable_wheel_speed()->set_is_wheel_spd_fl_valid(
        static_cast<bool>(status_310.flwheelspdvalid()));
    if (fbs2_240.flwheeldirection() == Fbs2_240::FLWHEELDIRECTION_FORWARD) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fl(
          WheelSpeed::FORWARD);
    } else if (fbs2_240.flwheeldirection() ==
               Fbs2_240::FLWHEELDIRECTION_BACKWARD) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fl(
          WheelSpeed::BACKWARD);
    } else if (fbs2_240.flwheeldirection() == Fbs2_240::FLWHEELDIRECTION_STOP) {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fl(
          WheelSpeed::STANDSTILL);
    } else {
      chassis_.mutable_wheel_speed()->set_wheel_direction_fl(
          WheelSpeed::INVALID);
    }
    chassis_.mutable_wheel_speed()->set_wheel_spd_fl(fbs1_243.flwheelspd());
  } else {
    chassis_.set_speed_mps(0);
  }
  // 7
  chassis_.set_fuel_range_m(0);
  // 8
  if (wey.has_fbs3_237() && wey.fbs3_237().has_accpedalpos()) {
    chassis_.set_throttle_percentage(
        static_cast<float>(wey.fbs3_237().accpedalpos()));
  } else {
    chassis_.set_throttle_percentage(0);
  }
  // 23, previously 10 gear position
  if (wey.has_fbs3_237() && wey.fbs3_237().has_currentgear()) {
    switch (wey.fbs3_237().currentgear()) {
      case Fbs3_237::CURRENTGEAR_D: {
        chassis_.set_gear_location(Chassis::GEAR_DRIVE);
      } break;
      case Fbs3_237::CURRENTGEAR_N: {
        chassis_.set_gear_location(Chassis::GEAR_NEUTRAL);
      } break;
      case Fbs3_237::CURRENTGEAR_R: {
        chassis_.set_gear_location(Chassis::GEAR_REVERSE);
      } break;
      case Fbs3_237::CURRENTGEAR_P: {
        chassis_.set_gear_location(Chassis::GEAR_PARKING);
      } break;
      default:
        chassis_.set_gear_location(Chassis::GEAR_INVALID);
        break;
    }
  } else {
    chassis_.set_gear_location(Chassis::GEAR_NONE);
  }
  // 11
  if (wey.has_fbs4_235() && wey.fbs4_235().has_steerwheelangle() &&
      wey.has_status_310() && wey.status_310().has_steerwheelanglesign()) {
    if (wey.status_310().steerwheelanglesign() ==
        Status_310::STEERWHEELANGLESIGN_LEFT_POSITIVE) {
      chassis_.set_steering_percentage(
          static_cast<float>(wey.fbs4_235().steerwheelangle() * 100.0 /
                             vehicle_params_.max_steer_angle() * M_PI / 180));
      angle_init = wey.fbs4_235().steerwheelangle();
    } else if (wey.status_310().steerwheelanglesign() ==
               Status_310::STEERWHEELANGLESIGN_RIGHT_NEGATIVE) {
      chassis_.set_steering_percentage(
          static_cast<float>(wey.fbs4_235().steerwheelangle() * (-1) * 100.0 /
                             vehicle_params_.max_steer_angle() * M_PI / 180));
      angle_init = wey.fbs4_235().steerwheelangle() * (-1);
    } else {
      chassis_.set_steering_percentage(0);
    }
  } else {
    chassis_.set_steering_percentage(0);
  }

  // 12
  if (wey.has_fbs3_237() && wey.fbs3_237().has_epsdrvinputtrqvalue()) {
    chassis_.set_steering_torque_nm(
        static_cast<float>(wey.fbs3_237().epsdrvinputtrqvalue()));
  } else {
    chassis_.set_steering_torque_nm(0);
  }
  // 13
  if (wey.has_status_310() && wey.status_310().has_epbsts()) {
    chassis_.set_parking_brake(wey.status_310().epbsts() ==
                               Status_310::EPBSTS_CLOSED);
  } else {
    chassis_.set_parking_brake(false);
  }
  // 14, 15
  if (wey.has_status_310() && wey.status_310().has_lowbeamsts() &&
      wey.status_310().lowbeamsts() == Status_310::LOWBEAMSTS_ON) {
    chassis_.mutable_signal()->set_low_beam(true);
  } else {
    chassis_.mutable_signal()->set_low_beam(false);
  }
  // 16, 17
  if (wey.has_status_310()) {
    if (wey.status_310().has_leftturnlampsts() &&
        wey.status_310().leftturnlampsts() == Status_310::LEFTTURNLAMPSTS_ON) {
      chassis_.mutable_signal()->set_turn_signal(
          common::VehicleSignal::TURN_LEFT);
    } else if (wey.status_310().has_rightturnlampsts() &&
               wey.status_310().rightturnlampsts() ==
                   Status_310::RIGHTTURNLAMPSTS_ON) {
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
  if (wey.has_vin_resp1_391() && wey.has_vin_resp2_392() &&
      wey.has_vin_resp3_393()) {
    Vin_resp1_391 vin_resp1_391 = wey.vin_resp1_391();
    Vin_resp2_392 vin_resp2_392 = wey.vin_resp2_392();
    Vin_resp3_393 vin_resp3_393 = wey.vin_resp3_393();
    if (vin_resp1_391.has_vin00() && vin_resp1_391.has_vin01() &&
        vin_resp1_391.has_vin02() && vin_resp1_391.has_vin03() &&
        vin_resp1_391.has_vin04() && vin_resp1_391.has_vin05() &&
        vin_resp1_391.has_vin06() && vin_resp1_391.has_vin07() &&
        vin_resp2_392.has_vin08() && vin_resp2_392.has_vin09() &&
        vin_resp2_392.has_vin10() && vin_resp2_392.has_vin11() &&
        vin_resp2_392.has_vin12() && vin_resp2_392.has_vin13() &&
        vin_resp2_392.has_vin14() && vin_resp2_392.has_vin15() &&
        vin_resp3_393.has_vin16()) {
      int n[17];
      n[0] = vin_resp1_391.vin07();
      n[1] = vin_resp1_391.vin06();
      n[2] = vin_resp1_391.vin05();
      n[3] = vin_resp1_391.vin04();
      n[4] = vin_resp1_391.vin03();
      n[5] = vin_resp1_391.vin02();
      n[6] = vin_resp1_391.vin01();
      n[7] = vin_resp1_391.vin00();
      n[8] = vin_resp2_392.vin15();
      n[9] = vin_resp2_392.vin14();
      n[10] = vin_resp2_392.vin13();
      n[11] = vin_resp2_392.vin12();
      n[12] = vin_resp2_392.vin11();
      n[13] = vin_resp2_392.vin10();
      n[14] = vin_resp2_392.vin09();
      n[15] = vin_resp2_392.vin08();
      n[16] = vin_resp3_393.vin16();
      char ch[17];
      memset(&ch, '\0', sizeof(ch));
      for (int i = 0; i < 17; i++) {
        ch[i] = static_cast<char>(n[i]);
      }
      chassis_.mutable_vehicle_id()->set_vin(ch);
    }
  }

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

void WeyController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode WeyController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "Already in COMPLETE_AUTO_DRIVE mode.";
    return ErrorCode::OK;
  }
  ads1_111_->set_ads_mode(Ads1_111::ADS_MODE_ACTIVE_MODE);
  ads_eps_113_->set_ads_epsmode(Ads_eps_113::ADS_EPSMODE_ACTIVE);
  // unlock the limited steering angle for first cmd within [-10,10] deg
  ads_eps_113_->set_ads_reqepstargetangle(angle_init);
  ads_shifter_115_->set_ads_shiftmode(Ads_shifter_115::ADS_SHIFTMODE_VALID);
  ads_req_vin_390_->set_req_vin_signal(Ads_req_vin_390::REQ_VIN_SIGNAL_REQUEST);
  // BCM enable control for horn/ beam/ turnlight
  // notice : if BCM enable, the beam manual control is invalid. If you use the
  // car at night, please watch out or disable this function before auto_drive
  ads3_38e_->set_ads_bcmworkstsvalid(Ads3_38e::ADS_BCMWORKSTSVALID_VALID);
  ads3_38e_->set_ads_bcm_worksts(Ads3_38e::ADS_BCM_WORKSTS_ACTIVE);
  ads3_38e_->set_ads_reqcontrolbcm(Ads3_38e::ADS_REQCONTROLBCM_REQUEST);
  ads3_38e_->set_dippedbeamon(Ads3_38e::DIPPEDBEAMON_TURN_ON);

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

ErrorCode WeyController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode WeyController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode";
    return ErrorCode::OK;
  }

  ads1_111_->set_ads_mode(Ads1_111::ADS_MODE_OFF_MODE);
  ads_eps_113_->set_ads_epsmode(Ads_eps_113::ADS_EPSMODE_ACTIVE);
  ads_eps_113_->set_ads_reqepstargetangle(angle_init);
  ads_shifter_115_->set_ads_shiftmode(Ads_shifter_115::ADS_SHIFTMODE_INVALID);
  ads3_38e_->set_ads_bcmworkstsvalid(Ads3_38e::ADS_BCMWORKSTSVALID_INVALID);
  ads3_38e_->set_ads_bcm_worksts(Ads3_38e::ADS_BCM_WORKSTS_DISABLE);
  ads3_38e_->set_ads_reqcontrolbcm(Ads3_38e::ADS_REQCONTROLBCM_NO_REQUEST);

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

ErrorCode WeyController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  ads1_111_->set_ads_mode(Ads1_111::ADS_MODE_ACTIVE_MODE);
  ads_eps_113_->set_ads_epsmode(Ads_eps_113::ADS_EPSMODE_DISABLE);
  ads_shifter_115_->set_ads_shiftmode(Ads_shifter_115::ADS_SHIFTMODE_VALID);
  ads3_38e_->set_ads_bcmworkstsvalid(Ads3_38e::ADS_BCMWORKSTSVALID_INVALID);
  ads3_38e_->set_ads_bcm_worksts(Ads3_38e::ADS_BCM_WORKSTS_DISABLE);
  ads3_38e_->set_ads_reqcontrolbcm(Ads3_38e::ADS_REQCONTROLBCM_NO_REQUEST);

  can_sender_->Update();
  if (!CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true)) {
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
void WeyController::Gear(Chassis::GearPosition gear_position) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "This drive mode no need to set gear.";
    return;
  }

  switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      ads_shifter_115_->set_ads_targetgear(Ads_shifter_115::ADS_TARGETGEAR_N);
      break;
    }
    case Chassis::GEAR_REVERSE: {
      ads_shifter_115_->set_ads_targetgear(Ads_shifter_115::ADS_TARGETGEAR_R);
      break;
    }
    case Chassis::GEAR_DRIVE: {
      ads_shifter_115_->set_ads_targetgear(Ads_shifter_115::ADS_TARGETGEAR_D);
      break;
    }
    case Chassis::GEAR_PARKING: {
      ads_shifter_115_->set_ads_targetgear(Ads_shifter_115::ADS_TARGETGEAR_P);
      break;
    }
    case Chassis::GEAR_LOW: {
      ads_shifter_115_->set_ads_targetgear(Ads_shifter_115::ADS_TARGETGEAR_N);
      break;
    }
    case Chassis::GEAR_NONE: {
      ads_shifter_115_->set_ads_targetgear(Ads_shifter_115::ADS_TARGETGEAR_N);
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      ads_shifter_115_->set_ads_targetgear(Ads_shifter_115::ADS_TARGETGEAR_N);
      break;
    }
    default: {
      ads_shifter_115_->set_ads_targetgear(Ads_shifter_115::ADS_TARGETGEAR_N);
      break;
    }
  }
}

// brake with pedal
// acceleration:-7.0 ~ 5.0, unit:m/s^2
void WeyController::Brake(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  // None
}

// drive with pedal
// acceleration:-7.0 ~ 5.0, unit:m/s^2
void WeyController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  // None
}

// wey use the acc to control the car acceleration and deceleration
// drive with acceleration/deceleration
// acc:-7.0 ~ 5.0, unit:m/s^2
void WeyController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  if (acc >= 0) {
    ads1_111_->set_ads_dectostop(Ads1_111::ADS_DECTOSTOP_NO_DEMAND);
    ads1_111_->set_ads_driveoff_req(Ads1_111::ADS_DRIVEOFF_REQ_DEMAND);
  } else {
    ads1_111_->set_ads_dectostop(Ads1_111::ADS_DECTOSTOP_DEMAND);
    ads1_111_->set_ads_driveoff_req(Ads1_111::ADS_DRIVEOFF_REQ_NO_DEMAND);
  }
  ads1_111_->set_ads_taracce(acc);
}

// wey default, -500 ~ 500, left:+, right:-
// angle:-99.99~0.00~99.99, unit:, left:+, right:-
void WeyController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle = 500 * angle / 100;
  ads_eps_113_->set_ads_reqepstargetangle(real_angle);
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// wey has no angle_spd
void WeyController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle = 500 * angle / 100;
  ads_eps_113_->set_ads_reqepstargetangle(real_angle);
}

void WeyController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void WeyController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    ads3_38e_->set_highbeamton(Ads3_38e::HIGHBEAMTON_TURN_ON);
  } else if (command.signal().low_beam()) {
    ads3_38e_->set_dippedbeamon(Ads3_38e::DIPPEDBEAMON_TURN_ON);
  } else {
    ads3_38e_->set_highbeamton(Ads3_38e::HIGHBEAMTON_TURN_OFF);
    ads3_38e_->set_dippedbeamon(Ads3_38e::DIPPEDBEAMON_TURN_OFF);
  }
}

void WeyController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    ads3_38e_->set_hornon(Ads3_38e::HORNON_TURN_ON);
  } else {
    ads3_38e_->set_hornon(Ads3_38e::HORNON_TURN_OFF);
  }
}

void WeyController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  auto signal = command.signal().turn_signal();
  if (signal == common::VehicleSignal::TURN_LEFT) {
    ads3_38e_->set_turnllighton(Ads3_38e::TURNLLIGHTON_TURN_LEFT_ON);
  } else if (signal == common::VehicleSignal::TURN_RIGHT) {
    ads3_38e_->set_turnllighton(Ads3_38e::TURNLLIGHTON_TURN_RIGHT_ON);
  } else {
    ads3_38e_->set_turnllighton(Ads3_38e::TURNLLIGHTON_TURN_OFF);
  }
}

void WeyController::ResetProtocol() { message_manager_->ResetSendMessages(); }

bool WeyController::CheckChassisError() {
  ChassisDetail chassis_detail;
  message_manager_->GetSensorData(&chassis_detail);
  if (!chassis_detail.has_wey()) {
    AERROR_EVERY(100) << "ChassisDetail has NO wey vehicle info."
                      << chassis_detail.DebugString();
    return false;
  }
  Wey wey = chassis_detail.wey();
  // check steer error
  if (wey.has_fail_241()) {
    if (wey.fail_241().epsfail() == Fail_241::EPSFAIL_FAULT) {
      return true;
    }
  }
  // check ems error
  if (wey.has_fail_241()) {
    if (wey.fail_241().engfail() == Fail_241::ENGFAIL_FAIL) {
      return true;
    }
  }
  // check braking error
  if (wey.has_fail_241()) {
    if (wey.fail_241().espfail() == Fail_241::ESPFAIL_FAILURE) {
      return true;
    }
  }
  // check gear error question
  if (wey.has_fail_241()) {
    if (wey.fail_241().shiftfail() ==
        Fail_241::SHIFTFAIL_TRANSMISSION_P_ENGAGEMENT_FAULT) {
      return true;
    }
  }
  // check parking error
  if (wey.has_fail_241()) {
    if (wey.fail_241().epbfail() == Fail_241::EPBFAIL_ERROR) {
      return true;
    }
  }
  return false;
}

void WeyController::SecurityDogThreadFunc() {
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

    // 1. Horizontal control check
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

    // 2. Vertical control check
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
      AERROR << "Too much time consumption in WeyController looping process:"
             << elapsed.count();
    }
  }
}

bool WeyController::CheckResponse(const int32_t flags, bool need_wait) {
  ChassisDetail chassis_detail;
  bool is_eps_online = false;
  bool is_vcu_online = false;
  bool is_esp_online = false;
  int retry_num = 20;

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
    if (need_wait) {
      --retry_num;
      std::this_thread::sleep_for(
          std::chrono::duration<double, std::milli>(20));
    }
    if (check_ok) {
      return true;
    }
    AINFO << "Need to check response again.";
  } while (need_wait && retry_num);

  AINFO << "check_response fail: is_eps_online: " << is_eps_online
        << ", is_vcu_online: " << is_vcu_online
        << ", is_esp_online: " << is_esp_online;
  return false;
}

void WeyController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t WeyController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode WeyController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void WeyController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace wey
}  // namespace canbus
}  // namespace apollo
