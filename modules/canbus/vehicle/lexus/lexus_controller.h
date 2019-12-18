/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <memory>
#include <thread>

#include "gtest/gtest_prod.h"

#include "modules/canbus/vehicle/vehicle_controller.h"

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/canbus/proto/chassis_detail.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/common/proto/error_code.pb.h"
#include "modules/control/proto/control_cmd.pb.h"

#include "modules/canbus/vehicle/lexus/protocol/accel_cmd_100.h"
#include "modules/canbus/vehicle/lexus/protocol/brake_cmd_104.h"
#include "modules/canbus/vehicle/lexus/protocol/cruise_control_buttons_cmd_108.h"
#include "modules/canbus/vehicle/lexus/protocol/dash_controls_right_rpt_210.h"
#include "modules/canbus/vehicle/lexus/protocol/hazard_lights_cmd_114.h"
#include "modules/canbus/vehicle/lexus/protocol/headlight_cmd_118.h"
#include "modules/canbus/vehicle/lexus/protocol/horn_cmd_11c.h"
#include "modules/canbus/vehicle/lexus/protocol/parking_brake_cmd_124.h"
#include "modules/canbus/vehicle/lexus/protocol/shift_cmd_128.h"
#include "modules/canbus/vehicle/lexus/protocol/steering_cmd_12c.h"
#include "modules/canbus/vehicle/lexus/protocol/turn_cmd_130.h"
#include "modules/canbus/vehicle/lexus/protocol/wiper_cmd_134.h"

namespace apollo {
namespace canbus {
namespace lexus {

class LexusController final : public VehicleController {
 public:
  virtual ~LexusController();

  ::apollo::common::ErrorCode Init(
      const VehicleParameter& params,
      CanSender<::apollo::canbus::ChassisDetail>* const can_sender,
      MessageManager<::apollo::canbus::ChassisDetail>* const message_manager)
      override;

  bool Start() override;

  /**
   * @brief stop the vehicle controller.
   */
  void Stop() override;

  /**
   * @brief calculate and return the chassis.
   * @returns a copy of chassis. Use copy here to avoid multi-thread issues.
   */
  Chassis chassis() override;

  FRIEND_TEST(LexusControllerTest, SetDrivingMode);
  FRIEND_TEST(LexusControllerTest, Status);
  FRIEND_TEST(LexusControllerTest, UpdateDrivingMode);

 private:
  // main logical function for operation the car enter or exit the auto driving
  void Emergency() override;
  ::apollo::common::ErrorCode EnableAutoMode() override;
  ::apollo::common::ErrorCode DisableAutoMode() override;
  ::apollo::common::ErrorCode EnableSteeringOnlyMode() override;
  ::apollo::common::ErrorCode EnableSpeedOnlyMode() override;

  // NEUTRAL, REVERSE, DRIVE
  void Gear(Chassis::GearPosition state) override;

  // brake with new acceleration
  // acceleration:0.00~99.99, unit:
  // acceleration_spd: 60 ~ 100, suggest: 90
  void Brake(double acceleration) override;

  // drive with old acceleration
  // gas:0.00~99.99 unit:
  void Throttle(double throttle) override;

  // drive with acceleration/deceleration
  // acc:-7.0~5.0 unit:m/s^2
  void Acceleration(double acc) override;

  // steering with old angle speed
  // angle:-99.99~0.00~99.99, unit:, left:+, right:-
  void Steer(double angle) override;

  // steering with new angle speed
  // angle:-99.99~0.00~99.99, unit:, left:+, right:-
  // angle_spd:0.00~99.99, unit:deg/s
  void Steer(double angle, double angle_spd) override;

  // set Electrical Park Brake
  void SetEpbBreak(const ::apollo::control::ControlCommand& command) override;
  void SetBeam(const ::apollo::control::ControlCommand& command) override;
  void SetHorn(const ::apollo::control::ControlCommand& command) override;
  void SetTurningSignal(
      const ::apollo::control::ControlCommand& command) override;

  void ResetProtocol();
  bool CheckChassisError();

 private:
  void SecurityDogThreadFunc();
  virtual bool CheckResponse(const int32_t flags, bool need_wait);
  void set_chassis_error_mask(const int32_t mask);
  int32_t chassis_error_mask();
  Chassis::ErrorCode chassis_error_code();
  void set_chassis_error_code(const Chassis::ErrorCode& error_code);

 private:
  // control protocol
  Accelcmd100* accel_cmd_100_ = nullptr;
  Brakecmd104* brake_cmd_104_ = nullptr;
  Cruisecontrolbuttonscmd108* cruise_control_buttons_cmd_108_ = nullptr;
  Dashcontrolsrightrpt210* dash_controls_right_rpt_210_ = nullptr;
  Hazardlightscmd114* hazard_lights_cmd_114_ = nullptr;
  Headlightcmd118* headlight_cmd_118_ = nullptr;
  Horncmd11c* horn_cmd_11c_ = nullptr;
  Parkingbrakecmd124* parking_brake_cmd_124_ = nullptr;
  Shiftcmd128* shift_cmd_128_ = nullptr;
  Steeringcmd12c* steering_cmd_12c_ = nullptr;
  Turncmd130* turn_cmd_130_ = nullptr;
  Wipercmd134* wiper_cmd_134_ = nullptr;

  Chassis chassis_;
  std::unique_ptr<std::thread> thread_;
  bool is_chassis_error_ = false;

  std::mutex chassis_error_code_mutex_;
  Chassis::ErrorCode chassis_error_code_ = Chassis::NO_ERROR;

  std::mutex chassis_mask_mutex_;
  int32_t chassis_error_mask_ = 0;
};

}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
