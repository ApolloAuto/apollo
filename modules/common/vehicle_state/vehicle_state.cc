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

#include "modules/common/vehicle_state/vehicle_state.h"
#include <cmath>
#include "modules/common/math/quaternion.h"
#include "modules/localization/common/localization_gflags.h"
#include "modules/common/log.h"

namespace apollo {
namespace common {
namespace vehicle_state {

VehicleState::VehicleState(
    const localization::LocalizationEstimate &localization) {
  ConstructExceptLinearVelocity(&localization);
  if (localization.has_pose() && localization.pose().has_linear_velocity()) {
    linear_v_ = localization.pose().linear_velocity().y();
  }
}

VehicleState::VehicleState(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis) {
  ConstructExceptLinearVelocity(localization);
  if (chassis != nullptr && chassis->has_speed_mps()) {
    linear_v_ = chassis->speed_mps();
  }
}

void VehicleState::ConstructExceptLinearVelocity(
    const localization::LocalizationEstimate *localization) {
  if (localization == nullptr || !localization->has_pose()) {
    AERROR << "Invalid localization input.";
    return;
  }
  localization_ptr_ = localization;
  if (localization->pose().has_position()) {
    x_ = localization->pose().position().x();
    y_ = localization->pose().position().y();
    z_ = localization->pose().position().z();
  }

  if (localization->pose().has_heading()) {
    heading_ = localization->pose().heading();
  } else {
    const auto &orientation = localization->pose().orientation();
    heading_ = ::apollo::common::math::QuaternionToHeading(
        orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());
  }

  if (FLAGS_enable_map_reference_unify) {
    angular_v_ = localization->pose().angular_velocity_vrf().z();
    linear_a_ = localization->pose().linear_acceleration_vrf().y();
  } else {
    angular_v_ = localization->pose().angular_velocity().z();
    linear_a_ = localization->pose().linear_acceleration().y();
  }
}

double VehicleState::x() const { return x_; }

double VehicleState::y() const { return y_; }

double VehicleState::z() const { return z_; }

double VehicleState::heading() const { return heading_; }

double VehicleState::linear_velocity() const { return linear_v_; }

double VehicleState::angular_velocity() const { return angular_v_; }

double VehicleState::linear_acceleration() const { return linear_a_; }

void VehicleState::set_x(const double x) { x_ = x; }

void VehicleState::set_y(const double y) { y_ = y; }

void VehicleState::set_z(const double z) { z_ = z; }

void VehicleState::set_heading(const double heading) { heading_ = heading; }

void VehicleState::set_linear_velocity(const double linear_velocity) {
  linear_v_ = linear_velocity;
}

void VehicleState::set_angular_velocity(const double angular_velocity) {
  angular_v_ = angular_velocity;
}

Eigen::Vector2d VehicleState::EstimateFuturePosition(const double t) const {
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  // Predict distance travel vector
  if (std::fabs(angular_v_) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = linear_v_ * t;
  } else {
    vec_distance[0] =
        -linear_v_ / angular_v_ * (1.0 - std::cos(angular_v_ * t));
    vec_distance[1] = std::sin(angular_v_ * t) * linear_v_ / angular_v_;
  }

  // If we have rotation information, take it into consideration.
  if (localization_ptr_ != nullptr && localization_ptr_->has_pose() &&
      localization_ptr_->pose().has_orientation()) {
    const auto &orientation = localization_ptr_->pose().orientation();
    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                         orientation.qy(), orientation.qz());
    Eigen::Vector3d pos_vec(x_, y_, z_);
    auto future_pos_3d = quaternion.toRotationMatrix() * vec_distance + pos_vec;
    return Eigen::Vector2d(future_pos_3d[0], future_pos_3d[1]);
  }

  // If no valid rotation information provided from localization,
  // return the estimated future position without rotation.
  return Eigen::Vector2d(vec_distance[0] + x_, vec_distance[1] + y_);
}

Eigen::Vector2d VehicleState::ComputeCOMPosition(
    const double rear_to_com_distance) const {
  // set length as distance between rear wheel and center of mass.
  Eigen::Vector3d v(0.0, rear_to_com_distance, 0.0);
  Eigen::Vector3d pos_vec(x_, y_, z_);
  // Initialize the COM position without rotation
  Eigen::Vector3d com_pos_3d = v + pos_vec;

  // If we have rotation information, take it into consideration.
  if (localization_ptr_ != nullptr && localization_ptr_->has_pose() &&
      localization_ptr_->pose().has_orientation()) {
    const auto &orientation = localization_ptr_->pose().orientation();
    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                         orientation.qy(), orientation.qz());
    // Update the COM position with rotation
    com_pos_3d = quaternion.toRotationMatrix() * v + pos_vec;
  }

  return Eigen::Vector2d(com_pos_3d[0], com_pos_3d[1]);
}

}  // namespace vehicle_state
}  // namespace common
}  // namespace apollo
