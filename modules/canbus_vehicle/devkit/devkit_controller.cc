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

#include "modules/canbus_vehicle/devkit/devkit_controller.h"

#include <string>

#include "modules/common_msgs/basic_msgs/vehicle_signal.pb.h"

#include "cyber/common/log.h"
#include "cyber/time/time.h"
#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/canbus_vehicle/devkit/devkit_message_manager.h"
#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace devkit {

using ::apollo::common::ErrorCode;
using ::apollo::common::VehicleSignal;
using ::apollo::control::ControlCommand;
using ::apollo::drivers::canbus::ProtocolData;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
bool emergency_brake = false;
}  // namespace

void DevkitController::AddSendMessage() {
  can_sender_->AddMessage(Throttlecommand100::ID, throttle_command_100_, false);
  can_sender_->AddMessage(Brakecommand101::ID, brake_command_101_, false);
  can_sender_->AddMessage(Gearcommand103::ID, gear_command_103_, false);
  can_sender_->AddMessage(Parkcommand104::ID, park_command_104_, false);
  can_sender_->AddMessage(Steeringcommand102::ID, steering_command_102_, false);
  can_sender_->AddMessage(Vehiclemodecommand105::ID, vehicle_mode_command_105_,
                          false);
}

ErrorCode DevkitController::Init(
    const VehicleParameter& params,
    CanSender<::apollo::canbus::Devkit>* const can_sender,
    MessageManager<::apollo::canbus::Devkit>* const message_manager) {
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

  vehicle_mode_command_105_ = dynamic_cast<Vehiclemodecommand105*>(
      message_manager_->GetMutableProtocolDataById(Vehiclemodecommand105::ID));
  if (vehicle_mode_command_105_ == nullptr) {
    AERROR
        << "Vehiclemodecommand105 does not exist in the DevkitMessageManager!";
    return ErrorCode::CANBUS_ERROR;
  }

  AddSendMessage();

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

  Devkit chassis_detail = GetNewRecvChassisDetail();

  // 1, 2
  // if (driving_mode() == Chassis::EMERGENCY_MODE) {
  //   set_chassis_error_code(Chassis::NO_ERROR);
  // }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);
  // 4 engine rpm ch has no engine rpm
  // chassis_.set_engine_rpm(0);
  // 5 wheel spd
  if (chassis_detail.has_wheelspeed_report_506()) {
    if (chassis_detail.wheelspeed_report_506().has_rr()) {
      chassis_.mutable_wheel_speed()->set_wheel_spd_rr(
          chassis_detail.wheelspeed_report_506().rr());
    }
    if (chassis_detail.wheelspeed_report_506().has_rl()) {
      chassis_.mutable_wheel_speed()->set_wheel_spd_rl(
          chassis_detail.wheelspeed_report_506().rl());
    }
    if (chassis_detail.wheelspeed_report_506().has_fr()) {
      chassis_.mutable_wheel_speed()->set_wheel_spd_fr(
          chassis_detail.wheelspeed_report_506().fr());
    }
    if (chassis_detail.wheelspeed_report_506().has_fl()) {
      chassis_.mutable_wheel_speed()->set_wheel_spd_fl(
          chassis_detail.wheelspeed_report_506().fl());
    }
  }
  // 6 speed_mps
  if (chassis_detail.has_vcu_report_505() &&
      chassis_detail.vcu_report_505().has_speed()) {
    chassis_.set_speed_mps(
        std::abs(static_cast<float>(chassis_detail.vcu_report_505().speed())));
  } else {
    chassis_.set_speed_mps(0);
  }
  // 7 no odometer
  // chassis_.set_odometer_m(0);
  // 8 no fuel. do not set;
  // chassis_.set_fuel_range_m(0);
  // 9 throttle
  if (chassis_detail.has_throttle_report_500() &&
      chassis_detail.throttle_report_500().has_throttle_pedal_actual()) {
    chassis_.set_throttle_percentage(static_cast<float>(
        chassis_detail.throttle_report_500().throttle_pedal_actual()));
  } else {
    chassis_.set_throttle_percentage(0);
  }
  // throttle sender cmd
  if (chassis_detail.has_throttle_command_100() &&
      chassis_detail.throttle_command_100().has_throttle_pedal_target()) {
    chassis_.set_throttle_percentage_cmd(static_cast<float>(
        chassis_detail.throttle_command_100().throttle_pedal_target()));
  } else {
    chassis_.set_throttle_percentage_cmd(0);
  }
  // 10 brake
  if (chassis_detail.has_brake_report_501() &&
      chassis_detail.brake_report_501().has_brake_pedal_actual()) {
    chassis_.set_brake_percentage(static_cast<float>(
        chassis_detail.brake_report_501().brake_pedal_actual()));
  } else {
    chassis_.set_brake_percentage(0);
  }
  // brake sender cmd
  if (chassis_detail.has_brake_command_101() &&
      chassis_detail.brake_command_101().has_brake_pedal_target()) {
    chassis_.set_brake_percentage_cmd(static_cast<float>(
        chassis_detail.brake_command_101().brake_pedal_target()));
  } else {
    chassis_.set_brake_percentage_cmd(0);
  }
  // 23, previously 11 gear
  if (chassis_detail.has_gear_report_503() &&
      chassis_detail.gear_report_503().has_gear_actual()) {
    Chassis::GearPosition gear_pos = Chassis::GEAR_INVALID;

    if (chassis_detail.gear_report_503().gear_actual() ==
        Gear_report_503::GEAR_ACTUAL_INVALID) {
      gear_pos = Chassis::GEAR_INVALID;
    }
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
  // 12 steering
  if (chassis_detail.has_steering_report_502() &&
      chassis_detail.steering_report_502().has_steer_angle_actual()) {
    chassis_.set_steering_percentage(static_cast<float>(
        chassis_detail.steering_report_502().steer_angle_actual() * 100.0 /
        vehicle_params_.max_steer_angle() * M_PI / 180));
  } else {
    chassis_.set_steering_percentage(0);
  }
  // steering sender cmd
  if (chassis_detail.has_steering_command_102() &&
      chassis_detail.steering_command_102().has_steer_angle_target()) {
    chassis_.set_steering_percentage_cmd(static_cast<float>(
        chassis_detail.steering_command_102().steer_angle_target()));
  } else {
    chassis_.set_steering_percentage_cmd(0);
  }
  // 13 parking brake
  if (chassis_detail.has_park_report_504() &&
      chassis_detail.park_report_504().has_parking_actual()) {
    if (chassis_detail.park_report_504().parking_actual() ==
        Park_report_504::PARKING_ACTUAL_PARKING_TRIGGER) {
      chassis_.set_parking_brake(true);
    } else {
      chassis_.set_parking_brake(false);
    }
  } else {
    chassis_.set_parking_brake(false);
  }
  // 14 battery soc
  if (chassis_detail.has_bms_report_512() &&
      chassis_detail.bms_report_512().has_battery_soc_percentage()) {
    chassis_.set_battery_soc_percentage(
        chassis_detail.bms_report_512().battery_soc_percentage());
    // 15 give engage_advice based on battery low soc warn
    if (chassis_detail.bms_report_512().is_battery_soc_low()) {
      chassis_.mutable_engage_advice()->set_advice(
          apollo::common::EngageAdvice::DISALLOW_ENGAGE);
      chassis_.mutable_engage_advice()->set_reason(
          "Battery soc percentage is lower than 15%, please charge it "
          "quickly!");
    }
  } else {
    chassis_.set_battery_soc_percentage(0);
  }
  // 16 sonor list
  // to do(ALL):check your vehicle type, confirm your sonar position because of
  // every vhechle has different sonars assembly.
  // 08 09 10 11
  if (chassis_detail.has_ultr_sensor_1_507()) {
    chassis_.mutable_surround()->set_sonar08(
        chassis_detail.ultr_sensor_1_507().uiuss8_tof_direct());
    chassis_.mutable_surround()->set_sonar09(
        chassis_detail.ultr_sensor_1_507().uiuss9_tof_direct());
    chassis_.mutable_surround()->set_sonar10(
        chassis_detail.ultr_sensor_1_507().uiuss10_tof_direct());
    chassis_.mutable_surround()->set_sonar11(
        chassis_detail.ultr_sensor_1_507().uiuss11_tof_direct());
  } else {
    chassis_.mutable_surround()->set_sonar08(0);
    chassis_.mutable_surround()->set_sonar09(0);
    chassis_.mutable_surround()->set_sonar10(0);
    chassis_.mutable_surround()->set_sonar11(0);
  }
  // 2 3 4 5
  if (chassis_detail.has_ultr_sensor_3_509()) {
    chassis_.mutable_surround()->set_sonar02(
        chassis_detail.ultr_sensor_3_509().uiuss2_tof_direct());
    chassis_.mutable_surround()->set_sonar03(
        chassis_detail.ultr_sensor_3_509().uiuss3_tof_direct());
    chassis_.mutable_surround()->set_sonar04(
        chassis_detail.ultr_sensor_3_509().uiuss4_tof_direct());
    chassis_.mutable_surround()->set_sonar05(
        chassis_detail.ultr_sensor_3_509().uiuss5_tof_direct());
  } else {
    chassis_.mutable_surround()->set_sonar02(0);
    chassis_.mutable_surround()->set_sonar03(0);
    chassis_.mutable_surround()->set_sonar04(0);
    chassis_.mutable_surround()->set_sonar05(0);
  }
  // 0 1 6 7
  if (chassis_detail.has_ultr_sensor_5_511()) {
    chassis_.mutable_surround()->set_sonar00(
        chassis_detail.ultr_sensor_5_511().uiuss0_tof_direct());
    chassis_.mutable_surround()->set_sonar01(
        chassis_detail.ultr_sensor_5_511().uiuss1_tof_direct());
    chassis_.mutable_surround()->set_sonar06(
        chassis_detail.ultr_sensor_5_511().uiuss6_tof_direct());
    chassis_.mutable_surround()->set_sonar07(
        chassis_detail.ultr_sensor_5_511().uiuss7_tof_direct());
  } else {
    chassis_.mutable_surround()->set_sonar00(0);
    chassis_.mutable_surround()->set_sonar01(0);
    chassis_.mutable_surround()->set_sonar06(0);
    chassis_.mutable_surround()->set_sonar07(0);
  }
  // 17 set vin
  // vin set 17 bits, like LSBN1234567890123 is prased as
  // vin17(L),vin16(S),vin15(B),......,vin03(1),vin02(2),vin01(3)
  std::string vin = "";
  if (chassis_detail.has_vin_resp1_514()) {
    Vin_resp1_514 vin_resp1_514 = chassis_detail.vin_resp1_514();
    vin += vin_resp1_514.vin00();
    vin += vin_resp1_514.vin01();
    vin += vin_resp1_514.vin02();
    vin += vin_resp1_514.vin03();
    vin += vin_resp1_514.vin04();
    vin += vin_resp1_514.vin05();
    vin += vin_resp1_514.vin06();
    vin += vin_resp1_514.vin07();
  }
  if (chassis_detail.has_vin_resp2_515()) {
    Vin_resp2_515 vin_resp2_515 = chassis_detail.vin_resp2_515();
    vin += vin_resp2_515.vin08();
    vin += vin_resp2_515.vin09();
    vin += vin_resp2_515.vin10();
    vin += vin_resp2_515.vin11();
    vin += vin_resp2_515.vin12();
    vin += vin_resp2_515.vin13();
    vin += vin_resp2_515.vin14();
    vin += vin_resp2_515.vin15();
  }
  if (chassis_detail.has_vin_resp3_516()) {
    Vin_resp3_516 vin_resp3_516 = chassis_detail.vin_resp3_516();
    vin += vin_resp3_516.vin16();
  }
  std::reverse(vin.begin(), vin.end());
  chassis_.mutable_vehicle_id()->set_vin(vin);
  // 18 front bumper event
  if (chassis_detail.has_vcu_report_505() &&
      chassis_detail.vcu_report_505().has_frontcrash_state()) {
    if (chassis_detail.vcu_report_505().frontcrash_state() ==
        Vcu_report_505::FRONTCRASH_STATE_CRASH_EVENT) {
      chassis_.set_front_bumper_event(Chassis::BUMPER_PRESSED);
    } else {
      chassis_.set_front_bumper_event(Chassis::BUMPER_NORMAL);
    }
  } else {
    chassis_.set_front_bumper_event(Chassis::BUMPER_INVALID);
  }
  // 19 back bumper event
  if (chassis_detail.has_vcu_report_505() &&
      chassis_detail.vcu_report_505().has_backcrash_state()) {
    if (chassis_detail.vcu_report_505().backcrash_state() ==
        Vcu_report_505::BACKCRASH_STATE_CRASH_EVENT) {
      chassis_.set_back_bumper_event(Chassis::BUMPER_PRESSED);
    } else {
      chassis_.set_back_bumper_event(Chassis::BUMPER_NORMAL);
    }
  } else {
    chassis_.set_back_bumper_event(Chassis::BUMPER_INVALID);
  }

  // 20 add checkresponse signal
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
        "devkit chassis detail is lost! Please check the communication error.");
    set_chassis_error_code(Chassis::CHASSIS_CAN_LOST);
    set_driving_mode(Chassis::EMERGENCY_MODE);
  }

  return chassis_;
}

bool DevkitController::VerifyID() {
  if (!CheckVin()) {
    AERROR << "Failed to get the vin.";
    GetVin();
    return false;
  } else {
    ResetVin();
    return true;
  }
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
    AINFO << "Set AEB enable";
  }

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
  if (!emergency_brake) {
    brake_command_101_->set_brake_pedal_target(pedal);
  }
  // brake_command_101_->set_brake_pedal_target(pedal);
}

// drive with pedal
// pedal:0.0~99.9 unit:%s
void DevkitController::Throttle(double pedal) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set throttle pedal.";
    return;
  }
  if (!emergency_brake) {
    throttle_command_100_->set_throttle_pedal_target(pedal);
  }
  // throttle_command_100_->set_throttle_pedal_target(pedal);
}

// confirm the car is driven by acceleration command instead of
// throttle/brake pedal drive with acceleration/deceleration acc:-7.0 ~ 5.0,
// unit:m/s^2
void DevkitController::Acceleration(double acc) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }
  // None
}

// confirm the car is driven by speed command
// speed:-xx.0~xx.0, unit:m/s
void DevkitController::Speed(double speed) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_SPEED_ONLY) {
    AINFO << "The current drive mode does not need to set speed.";
    return;
  }
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  // TODO(ALL): CHECK YOUR VEHICLE WHETHER SUPPORT THIS DRIVE MODE
  */
}

// devkit default, left:+, right:-
// need to be compatible with control module, so reverse
// steering with default angle speed, 25-250 (default:250)
// angle:-99.99~0.00~99.99, unit:, left:+, right:-
void DevkitController::Steer(double angle) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;

  if (!emergency_brake) {
    steering_command_102_->set_steer_angle_target(real_angle)
        ->set_steer_angle_spd_target(250);
  }
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:+, right:-
// angle_spd:25~250(default:250), unit:deg/s
void DevkitController::Steer(double angle, double angle_spd) {
  if (driving_mode() != Chassis::COMPLETE_AUTO_DRIVE &&
      driving_mode() != Chassis::AUTO_STEER_ONLY) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }
  const double real_angle =
      vehicle_params_.max_steer_angle() / M_PI * 180 * angle / 100.0;

  if (!emergency_brake) {
    steering_command_102_->set_steer_angle_target(real_angle)
        ->set_steer_angle_spd_target(250);
  }
}

void DevkitController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    park_command_104_->set_park_target(
        Park_command_104::PARK_TARGET_PARKING_TRIGGER);
  } else {
    park_command_104_->set_park_target(Park_command_104::PARK_TARGET_RELEASE);
  }
}

ErrorCode DevkitController::HandleCustomOperation(
    const external_command::ChassisCommand& command) {
  return ErrorCode::OK;
}

void DevkitController::SetBeam(const VehicleSignal& vehicle_signal) {
  if (vehicle_signal.high_beam()) {
    // None
  } else if (vehicle_signal.low_beam()) {
    // None
  }
}

void DevkitController::SetHorn(const VehicleSignal& vehicle_signal) {
  if (vehicle_signal.horn()) {
    // None
  } else {
    // None
  }
}

void DevkitController::SetTurningSignal(const VehicleSignal& vehicle_signal) {
  // Set Turn Signal
  auto signal = vehicle_signal.turn_signal();
  if (signal == common::VehicleSignal::TURN_LEFT) {
    vehicle_mode_command_105_->set_turn_light_ctrl(
        Vehicle_mode_command_105::TURN_LIGHT_CTRL_LEFT_TURNLAMP_ON);
  } else if (signal == common::VehicleSignal::TURN_RIGHT) {
    vehicle_mode_command_105_->set_turn_light_ctrl(
        Vehicle_mode_command_105::TURN_LIGHT_CTRL_RIGHT_TURNLAMP_ON);
  } else if (signal == common::VehicleSignal::TURN_HAZARD_WARNING) {
    vehicle_mode_command_105_->set_turn_light_ctrl(
        Vehicle_mode_command_105::TURN_LIGHT_CTRL_HAZARD_WARNING_LAMPSTS_ON);
  } else {
    vehicle_mode_command_105_->set_turn_light_ctrl(
        Vehicle_mode_command_105::TURN_LIGHT_CTRL_TURNLAMP_OFF);
  }
}

bool DevkitController::CheckVin() {
  if (chassis_.vehicle_id().vin().size() == 17) {
    AINFO << "Vin check success! Vehicel vin is "
          << chassis_.vehicle_id().vin();
    return true;
  } else {
    AINFO << "Vin check failed! Current vin size is "
          << chassis_.vehicle_id().vin().size();
    return false;
  }
}

void DevkitController::GetVin() {
  vehicle_mode_command_105_->set_vin_req(
      Vehicle_mode_command_105::VIN_REQ_VIN_REQ_ENABLE);
  AINFO << "Get vin";
  can_sender_->Update();
}

void DevkitController::ResetVin() {
  vehicle_mode_command_105_->set_vin_req(
      Vehicle_mode_command_105::VIN_REQ_VIN_REQ_DISABLE);
  AINFO << "Reset vin";
  can_sender_->Update();
}

void DevkitController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool DevkitController::CheckChassisError() {
  if (is_chassis_communication_error_) {
    AERROR_EVERY(100) << "ChassisDetail has no devkit vehicle info.";
    return false;
  }

  Devkit chassis_detail = GetNewRecvChassisDetail();
  // steer fault
  if (chassis_detail.has_steering_report_502()) {
    if (Steering_report_502::STEER_FLT1_STEER_SYSTEM_HARDWARE_FAULT ==
        chassis_detail.steering_report_502().steer_flt1()) {
      AERROR_EVERY(100) << "Chassis has steer system fault.";
      return true;
    }
  }
  // drive fault
  if (chassis_detail.has_throttle_report_500()) {
    if (Throttle_report_500::THROTTLE_FLT1_DRIVE_SYSTEM_HARDWARE_FAULT ==
        chassis_detail.throttle_report_500().throttle_flt1()) {
      AERROR_EVERY(100) << "Chassis has drive system fault.";
      return true;
    }
  }
  // brake fault
  if (chassis_detail.has_brake_report_501()) {
    if (Brake_report_501::BRAKE_FLT1_BRAKE_SYSTEM_HARDWARE_FAULT ==
        chassis_detail.brake_report_501().brake_flt1()) {
      AERROR_EVERY(100) << "Chassis has brake system fault.";
      return true;
    }
  }
  // battery soc low
  if (chassis_detail.has_bms_report_512()) {
    if (chassis_detail.bms_report_512().is_battery_soc_low()) {
      AERROR_EVERY(100) << "Chassis battery has low soc, please charge.";
      return true;
    }
  }
  // battery over emperature fault
  if (chassis_detail.has_bms_report_512()) {
    if (Bms_report_512::BATTERY_FLT_OVER_TEMP_FAULT ==
        chassis_detail.bms_report_512().battery_flt_over_temp()) {
      AERROR_EVERY(100) << "Chassis battery has over temperature fault.";
      return true;
    }
  }
  // battery low temperature fault
  if (chassis_detail.has_bms_report_512()) {
    if (Bms_report_512::BATTERY_FLT_LOW_TEMP_FAULT ==
        chassis_detail.bms_report_512().battery_flt_low_temp()) {
      AERROR_EVERY(100) << "Chassis battery has below low temperature fault.";
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
    emergency_brake = false;

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
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
      if (chassis_.speed_mps() > 0.3) {
        emergency_brake = true;
      }
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      if (emergency_brake) {
        throttle_command_100_->set_throttle_pedal_target(0);
        brake_command_101_->set_brake_pedal_target(40);
        steering_command_102_->set_steer_angle_target(0);
        std::this_thread::sleep_for(
            std::chrono::duration<double, std::milli>(3000));
      }
      message_manager_->ResetSendMessages();
      can_sender_->Update();
      emergency_brake = false;
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
      AERROR << "Too much time consumption in DevkitController looping "
                "process:"
             << elapsed.count();
    }
  }
}

bool DevkitController::CheckResponse(const int32_t flags, bool need_wait) {
  int32_t retry_num = 20;
  Devkit chassis_detail;
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
