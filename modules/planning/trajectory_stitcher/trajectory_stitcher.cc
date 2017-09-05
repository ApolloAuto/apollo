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
 * @file trajectory_stitcher.cc
 **/

#include "modules/planning/trajectory_stitcher/trajectory_stitcher.h"

#include <algorithm>

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

using VehicleState = apollo::common::VehicleState;

namespace {

std::vector<common::TrajectoryPoint> ComputeReinitStitchingTrajectory() {
  common::TrajectoryPoint init_point;
  init_point.mutable_path_point()->set_x(VehicleState::instance()->x());
  init_point.mutable_path_point()->set_y(VehicleState::instance()->y());
  init_point.mutable_path_point()->set_z(VehicleState::instance()->z());
  init_point.mutable_path_point()->set_theta(
      VehicleState::instance()->heading());
  init_point.mutable_path_point()->set_kappa(VehicleState::instance()->kappa());

  init_point.set_v(VehicleState::instance()->linear_velocity());
  init_point.set_a(VehicleState::instance()->linear_acceleration());
  init_point.set_relative_time(0.0);

  DCHECK(!std::isnan(init_point.path_point().x()));
  DCHECK(!std::isnan(init_point.path_point().y()));
  DCHECK(!std::isnan(init_point.path_point().z()));
  DCHECK(!std::isnan(init_point.path_point().theta()));
  DCHECK(!std::isnan(init_point.path_point().kappa()));
  DCHECK(!std::isnan(init_point.v()));
  DCHECK(!std::isnan(init_point.a()));

  return std::vector<common::TrajectoryPoint>(1, init_point);
}
}

// Planning from current vehicle state:
// if 1. the auto-driving mode is off or
//    2. we don't have the trajectory from last planning cycle or
//    3. the position deviation from actual and target is too high
std::vector<common::TrajectoryPoint>
TrajectoryStitcher::ComputeStitchingTrajectory(
    const bool is_on_auto_mode, const double current_timestamp,
    const double planning_cycle_time,
    const PublishableTrajectory& prev_trajectory) {
  if (!is_on_auto_mode) {
    return ComputeReinitStitchingTrajectory();
  }

  std::size_t prev_trajectory_size = prev_trajectory.NumOfPoints();

  if (prev_trajectory_size == 0) {
    AWARN << "Projected trajectory at time [" << prev_trajectory.header_time()
          << "] size is zero! Previous planning not exist or failed. Use "
             "origin car status instead.";
    return ComputeReinitStitchingTrajectory();
  }

  const double veh_rel_time = current_timestamp - prev_trajectory.header_time();

  std::size_t matched_index = prev_trajectory.QueryNearestPoint(veh_rel_time);

  if (matched_index == prev_trajectory_size) {
    AWARN << "The previous trajectory is not long enough, something is wrong";
    return ComputeReinitStitchingTrajectory();
  }

  if (matched_index == 0 &&
      veh_rel_time < prev_trajectory.StartPoint().relative_time()) {
    AWARN << "the previous trajectory doesn't cover current time";
    return ComputeReinitStitchingTrajectory();
  }

  auto matched_point = prev_trajectory.TrajectoryPointAt(matched_index);
  const double position_diff =
      common::math::Vec2d(
          matched_point.path_point().x() - VehicleState::instance()->x(),
          matched_point.path_point().y() - VehicleState::instance()->y())
          .Length();

  if (position_diff > FLAGS_replan_distance_threshold) {
    AWARN << "the distance between matched point and actual position is too "
             "large";
    return ComputeReinitStitchingTrajectory();
  }

  double forward_rel_time = veh_rel_time + planning_cycle_time;
  std::size_t forward_index =
      prev_trajectory.QueryNearestPoint(forward_rel_time);

  std::vector<common::TrajectoryPoint> stitching_trajectory(
      prev_trajectory.trajectory_points().begin() + matched_index,
      prev_trajectory.trajectory_points().begin() + forward_index + 1);

  double zero_time =
      prev_trajectory.TrajectoryPointAt(matched_index).relative_time();

  std::for_each(stitching_trajectory.begin(), stitching_trajectory.end(),
                [&zero_time](common::TrajectoryPoint& tp) {
                  tp.set_relative_time(tp.relative_time() - zero_time);
                });

  double zero_s =
      prev_trajectory.TrajectoryPointAt(forward_index).path_point().s();
  std::for_each(stitching_trajectory.begin(), stitching_trajectory.end(),
                [&zero_s](common::TrajectoryPoint& tp) {
                  double s = tp.path_point().s();
                  tp.mutable_path_point()->set_s(s - zero_s);
                });

  return stitching_trajectory;
}

}  // namespace planning
}  // namespace apollo
