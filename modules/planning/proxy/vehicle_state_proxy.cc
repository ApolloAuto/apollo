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
 * @file vehicle_state_proxy.cc
 **/

#include "modules/planning/proxy/vehicle_state_proxy.h"

#include <cmath>

#include "Eigen/Dense"

#include "modules/common/log.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

void VehicleStateProxy::set_vehicle_chassis(
    const ::apollo::common::config::VehicleConfig &config,
    const ::apollo::localization::LocalizationEstimate &localization_estimate,
    const ::apollo::canbus::Chassis &chassis) {
  _localization_estimate.CopyFrom(localization_estimate);
  _chassis.CopyFrom(chassis);
  init(config);
}

const ::apollo::localization::LocalizationEstimate &
VehicleStateProxy::localization_estimate() {
  return _localization_estimate;
}

const ::apollo::canbus::Chassis &VehicleStateProxy::vehicle_chassis() const {
  return _chassis;
}

const ::apollo::canbus::Chassis::GearPosition VehicleStateProxy::gear() const {
  return _chassis.gear_location();
}

double VehicleStateProxy::linear_velocity() const { return _linear_velocity; }

double VehicleStateProxy::heading() const { return _heading; }

double VehicleStateProxy::linear_acceleration() const {
  return _linear_acceleration;
}

double VehicleStateProxy::curvature() const { return _curvature; }

double VehicleStateProxy::timestamp() const {
  return _chassis.header().timestamp_sec();
}

// Proxy should be used as convenient helper with a pure reflection of sensor
// input.
// How to decide a planning starting point should be moved to other planning
// related places.

bool VehicleStateProxy::need_planner_reinit(
    const Frame *const prev_frame) const {
  // judge if the replanning is needed here
  /*
  if (prev_frame == nullptr) {
    return false;
  }
  const auto& prev_trajectory =
      prev_frame->planning_data().computed_trajectory();
  if (prev_trajectory.num_of_points() == 0) {
    AWARN << "Projected trajectory at time [" << prev_trajectory.header_time()
          << "] size is zero! Use origin car status instead.";
    return true;
  }
  const int prev_trajectory_size = prev_trajectory.num_of_points();
  const double veh_rel_time = timestamp() - prev_trajectory.header_time();
  if (Double::compare(prev_trajectory.start_point().relative_time(),
                      veh_rel_time) > 0 ||
      Double::compare(prev_trajectory.end_point().relative_time(),
                      veh_rel_time) < 0) {
    AWARN << "Projected trajectory at time [" << prev_trajectory.header_time()
          << "] is out of range ["
          << prev_trajectory.start_point().relative_time() << ","
          << prev_trajectory.end_point().relative_time()
          << "], current veh rel time " << veh_rel_time;
    return true;
  }
  for (int i = 1; i < prev_trajectory_size; ++i) {
    const double next_time =
        prev_trajectory.trajectory_point_at(i).relative_time();
    if (next_time >= veh_rel_time) {
      // TODO here add the check if replanning is needed
      return false;
    }
  }
  */
  return true;
}

const ::apollo::common::TrajectoryPoint VehicleStateProxy::init_point(
    const Frame *const prev_frame) const {
  ::apollo::common::TrajectoryPoint init_point;
  /*
   init_point.mutable_path_point()->set_x(
       _localization_estimate.pose().position().x());
   init_point.mutable_path_point()->set_y(
       _localization_estimate.pose().position().y());
   init_point.set_v(linear_velocity());
   init_point.set_a(linear_acceleration());
   init_point.mutable_path_point()->set_kappa(curvature());
   init_point.mutable_path_point()->set_theta(heading());
   init_point.set_relative_time(0.0);

   if (prev_frame == nullptr || need_planner_reinit(prev_frame)) {
     return init_point;
   }

   const auto& prev_trajectory =
       prev_frame->planning_data().computed_trajectory();

   if (prev_trajectory.num_of_points() == 0) {
     AWARN << "Projected trajectory at time [" << prev_trajectory.header_time()
           << "] size is zero! Use original chassis instea.";
     return init_point;
   }

   const int prev_trajectory_size = prev_trajectory.num_of_points();
   const double veh_rel_time =
       timestamp() - prev_trajectory.header_time() + FLAGS_forward_predict_time;

   if (prev_trajectory.start_point().relative_time() > veh_rel_time ||
       prev_trajectory.end_point().relative_time() < veh_rel_time) {
     AWARN << "Projected trajectory at time [" << prev_trajectory.header_time()
           << "] is out of range ["
           << prev_trajectory.start_point().relative_time() << ", "
           << prev_trajectory.end_point().relative_time()
           << "], current veh_rel_time is " << veh_rel_time;
     return init_point;
   }

   for (int i = 1; i < prev_trajectory_size; ++i) {
     const double next_time =
         prev_trajectory.trajectory_point_at(i).relative_time();
     if (next_time >= veh_rel_time) {
       // update status and return;
       init_point = prev_trajectory.trajectory_point_at(i);
       init_point.set_relative_time(next_time + prev_trajectory.header_time() -
                                    timestamp());
       return init_point;
     }
   }
   */
  return init_point;
}

void VehicleStateProxy::init(
    const ::apollo::common::config::VehicleConfig &config) {
  _config = config;
  // TODO(@lianglia_apollo):
  // if the car is driving uphill or downhill, the velocity in the z axis
  // is not 0.
  const auto &velocity3d = _localization_estimate.pose().linear_velocity();
  Eigen::Vector2d velocity2d(velocity3d.x(), velocity3d.y());
  _linear_velocity = velocity2d.norm();

  const auto &acceleration3d =
      _localization_estimate.pose().linear_acceleration();
  if (std::isnan(acceleration3d.x()) || std::isnan(acceleration3d.y())) {
    _linear_acceleration = 0.0;
  } else {
    // TODO(@lianglia_apollo): verify the acceleration in the z axis.
    Eigen::Vector2d acceleration2d(acceleration3d.x(), acceleration3d.y());
    if (_linear_velocity > 0.0) {
      _linear_acceleration = acceleration2d.dot(velocity2d) / _linear_velocity;
    } else {
      _linear_acceleration = acceleration2d.norm();
    }
  }

  const auto &quaternion = _localization_estimate.pose().orientation();
  _heading = std::atan2(2.0 * (quaternion.qw() * quaternion.qz() +
                            quaternion.qx() * quaternion.qy()),
                        1.0 -
                            2.0 * (quaternion.qy() * quaternion.qy() +
                                quaternion.qz() * quaternion.qz())) +
      M_PI / 2.0;

  CHECK(_config.has_vehicle_param());
  CHECK(_config.vehicle_param().has_steer_ratio());
  CHECK(_config.vehicle_param().has_max_steer_angle());
  CHECK(_config.vehicle_param().has_wheel_base());

  if (std::isnan(_chassis.steering_percentage())) {
    AERROR << "Missing chassis steering percentage info!";
    _curvature = 0.0;
  } else {
    _curvature = std::tan(_chassis.steering_percentage() *
        _config.vehicle_param().max_steer_angle() /
        _config.vehicle_param().steer_ratio()) /
        _config.vehicle_param().wheel_base();
  }
}

}  // namespace planning
}  // namespace apollo
