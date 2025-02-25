// Copyright 2025 daohu527@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#pragma once

#include <memory>
#include <thread>

#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/canbus/proto/vehicle_parameter.pb.h"
#include "modules/common_msgs/basic_msgs/error_code.pb.h"
#include "modules/common_msgs/control_msgs/control_cmd.pb.h"

#include "modules/canbus/vehicle/vehicle_controller.h"
#include "modules/canbus_vehicle/ros/protocol/acc_fb.h"
#include "modules/canbus_vehicle/ros/protocol/ang_vel_fb.h"
#include "modules/canbus_vehicle/ros/protocol/twist_cmd.h"
#include "modules/canbus_vehicle/ros/protocol/twist_fb.h"

namespace apollo {
namespace canbus {
namespace ros {

class RosController final
    : public VehicleController<::apollo::canbus::Ros> {
 public:
  RosController() {}

  virtual ~RosController();

  ::apollo::common::ErrorCode Init(
      const VehicleParameter& params,
      CanSender<::apollo::canbus::Ros>* const can_sender,
      MessageManager<::apollo::canbus::Ros>* const message_manager)
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

  /**
   * for test
   */
  FRIEND_TEST(RosControllerTest, SetDrivingMode);
  FRIEND_TEST(RosControllerTest, Status);
  FRIEND_TEST(RosControllerTest, UpdateDrivingMode);

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

  void SetSpeed(double speed) override;

  void SetAngularSpeed(double angular_speed) override;

  // set Electrical Park Brake
  void SetEpbBreak(const control::ControlCommand& command) override;
  common::ErrorCode HandleCustomOperation(
    const external_command::ChassisCommand& command) override;
  void SetBeam(const common::VehicleSignal& signal) override;
  void SetHorn(const common::VehicleSignal& signal) override;
  void SetTurningSignal(const common::VehicleSignal& signal) override;

  bool VerifyID() override;
  void ResetProtocol();
  bool CheckChassisError();

 private:
  void SecurityDogThreadFunc();
  Chassis::ErrorCode chassis_error_code();
  void set_chassis_error_code(const Chassis::ErrorCode& error_code);

 private:
  // control protocol
  TwistCmd* twist_cmd_ = nullptr;

  Chassis chassis_;
  std::unique_ptr<std::thread> thread_;

  std::mutex chassis_error_code_mutex_;
  Chassis::ErrorCode chassis_error_code_ = Chassis::NO_ERROR;
};

}  // namespace ros
}  // namespace canbus
}  // namespace apollo
