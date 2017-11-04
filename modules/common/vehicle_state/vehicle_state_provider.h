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
/**
 * @file vehicle_state.h
 *
 * @brief Declaration of the class VehicleStateProvider.
 */
#ifndef MODULES_COMMON_VEHICLE_STATE_VEHICLE_STATE_H_
#define MODULES_COMMON_VEHICLE_STATE_VEHICLE_STATE_H_

#include <memory>
#include <string>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "modules/common/macro.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/status/status.h"

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
namespace apollo {
namespace common {

/**
 * @class VehicleStateProvider
 * @brief The class of vehicle state.
 *        It includes basic information and computation
 *        about the state of the vehicle.
 */
class VehicleStateProvider {
 public:
  /**
   * @brief Constructor by information of localization and chassis.
   * @param localization Localization information of the vehicle.
   * @param chassis Chassis information of the vehicle.
   */
  Status Update(const localization::LocalizationEstimate& localization,
                const canbus::Chassis& chassis);

  /**
   * @brief Update VehicleStateProvider instance by protobuf files.
   * @param localization_file the localization protobuf file.
   * @param chassis_file The chassis protobuf file
   */
  void Update(const std::string& localization_file,
              const std::string& chassis_file);

  double timestamp() const { return timestamp_; }

  const localization::Pose& pose() const { return pose_; }

  /**
   * @brief Default destructor.
   */
  virtual ~VehicleStateProvider() = default;

  /**
   * @brief Get the x-coordinate of vehicle position.
   * @return The x-coordinate of vehicle position.
   */
  double x() const;

  /**
   * @brief Get the y-coordinate of vehicle position.
   * @return The y-coordinate of vehicle position.
   */
  double y() const;

  /**
   * @brief Get the z coordinate of vehicle position.
   * @return The z coordinate of vehicle position.
   */
  double z() const;

  double kappa() const;

  /**
   * @brief Get the vehicle roll angle.
   * @return The euler roll angle.
   */
  double roll() const;

  /**
   * @brief Get the vehicle pitch angle.
   * @return The euler pitch angle.
   */
  double pitch() const;

  /**
   * @brief Get the vehicle yaw angle.
   *  As of now, use the heading instead of yaw angle.
   *  Heading angle with East as zero, yaw angle has North as zero
   * @return The euler yaw angle.
   */
  double yaw() const;

  /**
   * @brief Get the heading of vehicle position, which is the angle
   *        between the vehicle's heading direction and the x-axis.
   * @return The angle between the vehicle's heading direction
   *         and the x-axis.
   */
  double heading() const;

  /**
   * @brief Get the vehicle's linear velocity.
   * @return The vehicle's linear velocity.
   */
  double linear_velocity() const;

  /**
   * @brief Get the vehicle's angular velocity.
   * @return The vehicle's angular velocity.
   */
  double angular_velocity() const;

  /**
   * @brief Get the vehicle's linear acceleration.
   * @return The vehicle's linear acceleration.
   */
  double linear_acceleration() const;

  /**
   * @brief Get the vehicle's gear position.
   * @return The vehicle's gear position.
   */
  double gear() const;

  /**
   * @brief Set the x-coordinate of vehicle position.
   * @param x The x-coordinate of vehicle position.
   */
  void set_x(const double x);

  /**
   * @brief Set the y-coordinate of vehicle position.
   * @param y The y-coordinate of vehicle position.
   */
  void set_y(const double y);

  /**
   * @brief Set the z coordinate of vehicle position.
   * @param z The z coordinate of vehicle position.
   */
  void set_z(const double z);

  /**
   * @brief Set the vehicle roll angle.
   * @param pitch The vehicle roll angle.
   */
  void set_roll(const double roll);

  /**
   * @brief Set the vehicle pitch angle.
   * @param pitch The vehicle pitch angle.
   */
  void set_pitch(const double pitch);

  /**
   * @brief Set the vehicle yaw angle.
   * @param pitch The vehicle yaw angle.
   */
  void set_yaw(const double yaw);

  /**
   * @brief Set the heading of vehicle position, which is the angle
   *        between the vehicle's heading direction and the x-axis.
   * @param heading The angle between the vehicle's heading direction
   *         and the x-axis.
   */
  void set_heading(const double heading);

  void set_kappa(const double kappa) { kappa_ = kappa; }

  /**
   * @brief Set the vehicle's linear velocity.
   * @param linear_velocity The value to set the vehicle's linear velocity.
   */
  void set_linear_velocity(const double linear_velocity);

  /**
   * @brief Set the vehicle's angular velocity.
   * @param angular_velocity The vehicle's angular velocity.
   */
  void set_angular_velocity(const double angular_velocity);

  /**
   * @brief Set the vehicle's gear position.
   * @param gear_position The vehicle's gear position.
   */
  void set_gear(const canbus::Chassis::GearPosition gear_position);

  /**
   * @brief Estimate future position from current position and heading,
   *        along a period of time, by constant linear velocity,
   *        linear acceleration, angular velocity.
   * @param t The length of time period.
   * @return The estimated future position in time t.
   */
  math::Vec2d EstimateFuturePosition(const double t) const;

  /**
   * @brief Compute the position of center of mass(COM) of the vehicle,
   *        given the distance from rear wheels to the center of mass.
   * @param rear_to_com_distance Distance from rear wheels to
   *        the vehicle's center of mass.
   * @return The position of the vehicle's center of mass.
   */
  math::Vec2d ComputeCOMPosition(const double rear_to_com_distance) const;

  /**
   * @brief Compute the bouding box of the vehicle.
   * @return the bounding box of the vehicle represented by Box2d.
   */
  const math::Box2d& AdcBoundingBox() const;

 private:
  DECLARE_SINGLETON(VehicleStateProvider);

  void InitAdcBoundingBox();

  bool ConstructExceptLinearVelocity(
      const localization::LocalizationEstimate& localization);

  double x_ = 0.0;
  double y_ = 0.0;
  double z_ = 0.0;
  double roll_ = 0.0;
  double pitch_ = 0.0;
  double yaw_ = 0.0;
  double heading_ = 0.0;
  // TODO(all): check the setting of kappa_
  double kappa_ = 0.0;
  double linear_v_ = 0.0;
  double angular_v_ = 0.0;
  double timestamp_ = 0.0;
  double linear_a_y_ = 0.0;
  canbus::Chassis::GearPosition gear_;
  localization::Pose pose_;
  std::unique_ptr<math::Box2d> adc_bounding_box_ = nullptr;
};

}  // namespace common
}  // namespace apollo

#endif  // MODULES_COMMON_VEHICLE_STATE_VEHICLE_STATE_H_
