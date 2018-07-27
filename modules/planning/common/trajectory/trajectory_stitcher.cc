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
 * @file
 **/

#include "modules/planning/common/trajectory/trajectory_stitcher.h"

#include <algorithm>
#include <list>
#include <utility>

#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::common::util::DistanceXY;

std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const VehicleState& vehicle_state) {
  TrajectoryPoint init_point;
  init_point.mutable_path_point()->set_s(0.0);
  init_point.mutable_path_point()->set_x(vehicle_state.x());
  init_point.mutable_path_point()->set_y(vehicle_state.y());
  init_point.mutable_path_point()->set_z(vehicle_state.z());
  init_point.mutable_path_point()->set_theta(vehicle_state.heading());
  init_point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  init_point.set_v(vehicle_state.linear_velocity());
  init_point.set_a(vehicle_state.linear_acceleration());
  init_point.set_relative_time(0.0);

  return std::vector<TrajectoryPoint>(1, init_point);
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  float cos_theta =
      common::math::cos(common::math::Angle16::from_rad(theta_diff));
  float sin_theta =
      -common::math::sin(common::math::Angle16::from_rad(theta_diff));

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->trajectory_points().begin(),
                prev_trajectory->trajectory_points().end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](common::TrajectoryPoint& p) {
                  auto x = p.path_point().x();
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new = common::math::WrapAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

// Planning from current vehicle state:
// if 1. the auto-driving mode is off or
//    2. we don't have the trajectory from last planning cycle or
//    3. the position deviation from actual and target is too high
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time,
    const PublishableTrajectory* prev_trajectory, bool* is_replan) {
  *is_replan = true;
  if (!FLAGS_enable_trajectory_stitcher) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  if (!prev_trajectory) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  std::size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  std::size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  if (time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  auto time_matched_point =
      prev_trajectory->TrajectoryPointAt(time_matched_index);

  if (!time_matched_point.has_path_point()) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  std::size_t position_matched_index = prev_trajectory->QueryNearestPoint(
      {vehicle_state.x(), vehicle_state.y()});

  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(position_matched_index));

  auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
  auto lat_diff = frenet_sd.second;

  ADEBUG << "Control lateral diff: " << lat_diff
         << ", longitudinal diff: " << lon_diff;

  if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold ||
      std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {
    AERROR << "the distance between matched point and actual position is too "
              "large. Replan is triggered. lat_diff = "
           << lat_diff << ", lon_diff = " << lon_diff;
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  double forward_rel_time =
      prev_trajectory->TrajectoryPointAt(time_matched_index).relative_time() +
      planning_cycle_time;

  std::size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  ADEBUG << "Position matched index: " << position_matched_index;
  ADEBUG << "Time matched index: " << time_matched_index;

  auto matched_index = std::min(time_matched_index, position_matched_index);
  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->trajectory_points().begin() +
          std::max(0, static_cast<int>(matched_index - 1)),
      prev_trajectory->trajectory_points().begin() + forward_time_index + 1);

  const double zero_s = time_matched_point.path_point().s();

  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      return ComputeReinitStitchingTrajectory(vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  *is_replan = false;
  return stitching_trajectory;
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) {
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  Vec2d n(common::math::cos(
              common::math::Angle16::from_rad(p.path_point().theta())),
          common::math::sin(
              common::math::Angle16::from_rad(p.path_point().theta())));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

}  // namespace planning
}  // namespace apollo
