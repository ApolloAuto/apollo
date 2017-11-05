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

#include "modules/common/vehicle_state/vehicle_state_provider.h"

#include <cmath>

#include "Eigen/Core"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/euler_angles_zxy.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/string_util.h"
#include "modules/localization/common/localization_gflags.h"

namespace apollo {
namespace common {

VehicleStateProvider::VehicleStateProvider() {}

Status VehicleStateProvider::Update(
    const localization::LocalizationEstimate &localization,
    const canbus::Chassis &chassis) {
  if (!ConstructExceptLinearVelocity(localization)) {
    std::string msg = util::StrCat(
        "Fail to update because ConstructExceptLinearVelocity error.",
        "localization:\n", localization.DebugString());
    return Status(ErrorCode::LOCALIZATION_ERROR, msg);
  }
  if (chassis.has_header() && chassis.header().has_timestamp_sec()) {
    timestamp_ = chassis.header().timestamp_sec();
  }
  if (chassis.has_speed_mps()) {
    linear_v_ = chassis.speed_mps();
  }

  if (chassis.has_gear_location()) {
    gear_ = chassis.gear_location();
  } else {
    gear_ = canbus::Chassis::GEAR_NONE;
  }

  InitAdcBoundingBox();

  return Status::OK();
}

void VehicleStateProvider::InitAdcBoundingBox() {
  const auto &param =
      VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  math::Vec2d position(x_, y_);
  math::Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
  math::Vec2d center(position + vec_to_center.rotate(heading_));
  adc_bounding_box_ = std::unique_ptr<math::Box2d>(
      new math::Box2d(center, heading_, param.length(), param.width()));
}

const math::Box2d &VehicleStateProvider::AdcBoundingBox() const {
  CHECK(adc_bounding_box_) << "ADC bounding box not initialized";
  return *adc_bounding_box_;
}

bool VehicleStateProvider::ConstructExceptLinearVelocity(
    const localization::LocalizationEstimate &localization) {
  if (!localization.has_pose()) {
    AERROR << "Invalid localization input.";
    return false;
  }
  pose_ = localization.pose();
  if (localization.pose().has_position()) {
    x_ = localization.pose().position().x();
    y_ = localization.pose().position().y();
    z_ = localization.pose().position().z();
  }

  const auto &orientation = localization.pose().orientation();

  if (localization.pose().has_heading()) {
    heading_ = localization.pose().heading();
  } else {
    heading_ = math::QuaternionToHeading(orientation.qw(), orientation.qx(),
                                         orientation.qy(), orientation.qz());
  }

  if (FLAGS_enable_map_reference_unify) {
    if (!localization.pose().has_angular_velocity_vrf()) {
      AERROR << "localization.pose().has_angular_velocity_vrf() must be true "
                "when FLAGS_enable_map_reference_unify is true.";
      return false;
    }
    angular_v_ = localization.pose().angular_velocity_vrf().z();

    if (!localization.pose().has_linear_acceleration_vrf()) {
      AERROR << "localization.pose().has_linear_acceleration_vrf() must be "
                "true when FLAGS_enable_map_reference_unify is true.";
      return false;
    }
    linear_a_y_ = localization.pose().linear_acceleration_vrf().y();
  } else {
    CHECK(localization.pose().has_angular_velocity());
    angular_v_ = localization.pose().angular_velocity().z();
    CHECK(localization.pose().has_linear_acceleration());
    linear_a_y_ = localization.pose().linear_acceleration().y();
  }

  if (!(linear_v_ > 0.0)) {
    kappa_ = 0.0;
  } else {
    kappa_ = angular_v_ / linear_v_;
  }

  if (localization.pose().has_euler_angles()) {
    roll_ = localization.pose().euler_angles().x();
    pitch_ = localization.pose().euler_angles().y();
    yaw_ = localization.pose().euler_angles().z();
  } else {
    math::EulerAnglesZXYd euler_angle(orientation.qw(), orientation.qx(),
                                      orientation.qy(), orientation.qz());
    roll_ = euler_angle.roll();
    pitch_ = euler_angle.pitch();
    yaw_ = euler_angle.yaw();
  }

  return true;
}

double VehicleStateProvider::x() const { return x_; }

double VehicleStateProvider::y() const { return y_; }

double VehicleStateProvider::z() const { return z_; }

double VehicleStateProvider::roll() const { return roll_; }

double VehicleStateProvider::pitch() const { return pitch_; }

double VehicleStateProvider::yaw() const { return yaw_; }

double VehicleStateProvider::heading() const { return heading_; }

double VehicleStateProvider::kappa() const { return kappa_; }

double VehicleStateProvider::linear_velocity() const { return linear_v_; }

double VehicleStateProvider::angular_velocity() const { return angular_v_; }

double VehicleStateProvider::linear_acceleration() const { return linear_a_y_; }

double VehicleStateProvider::gear() const { return gear_; }

void VehicleStateProvider::set_x(const double x) { x_ = x; }

void VehicleStateProvider::set_y(const double y) { y_ = y; }

void VehicleStateProvider::set_z(const double z) { z_ = z; }

void VehicleStateProvider::set_roll(const double roll) { roll_ = roll; }

void VehicleStateProvider::set_pitch(const double pitch) { pitch_ = pitch; }

// As of now, use heading instead of yaw angle
void VehicleStateProvider::set_yaw(const double yaw) { yaw_ = yaw; }

void VehicleStateProvider::set_heading(const double heading) {
  heading_ = heading;
}

void VehicleStateProvider::set_linear_velocity(const double linear_velocity) {
  linear_v_ = linear_velocity;
}

void VehicleStateProvider::set_angular_velocity(const double angular_velocity) {
  angular_v_ = angular_velocity;
}

void VehicleStateProvider::set_gear(
    const canbus::Chassis::GearPosition gear_position) {
  gear_ = gear_position;
}

math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const {
  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  double v = linear_v_;
  if (gear_ == canbus::Chassis::GEAR_REVERSE) {
    v = -linear_v_;
  }
  // Predict distance travel vector
  if (std::fabs(angular_v_) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    vec_distance[0] = -v / angular_v_ * (1.0 - std::cos(angular_v_ * t));
    vec_distance[1] = std::sin(angular_v_ * t) * v / angular_v_;
  }

  // If we have rotation information, take it into consideration.
  if (pose_.has_orientation()) {
    const auto &orientation = pose_.orientation();
    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                         orientation.qy(), orientation.qz());
    Eigen::Vector3d pos_vec(x_, y_, z_);
    auto future_pos_3d = quaternion.toRotationMatrix() * vec_distance + pos_vec;
    return math::Vec2d(future_pos_3d[0], future_pos_3d[1]);
  }

  // If no valid rotation information provided from localization,
  // return the estimated future position without rotation.
  return math::Vec2d(vec_distance[0] + x_, vec_distance[1] + y_);
}

math::Vec2d VehicleStateProvider::ComputeCOMPosition(
    const double rear_to_com_distance) const {
  // set length as distance between rear wheel and center of mass.
  Eigen::Vector3d v(0.0, rear_to_com_distance, 0.0);
  Eigen::Vector3d pos_vec(x_, y_, z_);
  // Initialize the COM position without rotation
  Eigen::Vector3d com_pos_3d = v + pos_vec;

  // If we have rotation information, take it into consideration.
  if (pose_.has_orientation()) {
    const auto &orientation = pose_.orientation();
    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                         orientation.qy(), orientation.qz());
    // Update the COM position with rotation
    com_pos_3d = quaternion.toRotationMatrix() * v + pos_vec;
  }
  return math::Vec2d(com_pos_3d[0], com_pos_3d[1]);
}

}  // namespace common
}  // namespace apollo
