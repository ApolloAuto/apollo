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
 * @brief Declaration of the class VehicleState.
 */
#ifndef MODULES_COMMON_VEHICLE_STATE_VEHICLE_STATE_H_
#define MODULES_COMMON_VEHICLE_STATE_VEHICLE_STATE_H_

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "Eigen/Core"

/**
 * @namespace apollo::common::vehicle_state
 * @brief apollo::common::vehicle_state
 */
namespace apollo {
namespace common {
namespace vehicle_state {

/**
 * @class VehicleState
 * @brief The class of vehicle state.
 *        It includes basic information and computation
 *        about the state of the vehicle.
 */
class VehicleState {
 public:
  /**
   * @brief Empty constructor.
   */
  VehicleState() = default;

  /**
   * @brief Constructor only by information of localization.
   * @param localization Localization information of the vehicle.
   */
  explicit VehicleState(const localization::LocalizationEstimate &localization);

  /**
   * @brief Constructor by information of localization and chassis.
   * @param localization Localization information of the vehicle.
   * @param chassis Chassis information of the vehicle.
   */
  VehicleState(const localization::LocalizationEstimate *localization,
               const canbus::Chassis *chassis);

  /**
   * @brief Default destructor.
   */
  virtual ~VehicleState() = default;

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
   * @brief Set the heading of vehicle position, which is the angle
   *        between the vehicle's heading direction and the x-axis.
   * @param heading The angle between the vehicle's heading direction
   *         and the x-axis.
   */
  void set_heading(const double heading);

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
   * @brief Estimate future position from current position and heading,
   *        along a period of time, by constant linear velocity,
   *        linear acceleration, angular velocity.
   * @param t The length of time period.
   * @return The estimated future position in time t.
   */
  Eigen::Vector2d EstimateFuturePosition(const double t) const;

  /**
 * @brief Compute the position of center of mass(COM) of the vehicle,
 *        given the distance from rear wheels to the center of mass.
 * @param rear_to_com_distance Distance from rear wheels to
 *        the vehicle's center of mass.
 * @return The position of the vehicle's center of mass.
 */
  Eigen::Vector2d ComputeCOMPosition(const double rear_to_com_distance) const;

 private:
  void ConstructExceptLinearVelocity(
      const localization::LocalizationEstimate *localization);

  double x_ = 0.0;

  double y_ = 0.0;

  double z_ = 0.0;

  double heading_ = 0.0;

  double linear_v_ = 0.0;

  double angular_v_ = 0.0;

  double linear_a_ = 0.0;

  const localization::LocalizationEstimate *localization_ptr_ = nullptr;
};

}  // namespace vehicle_state
}  // namespace common
}  // namespace apollo

#endif /* MODULES_COMMON_VEHICLE_STATE_VEHICLE_STATE_H_ */
