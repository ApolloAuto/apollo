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

#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
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

void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double current_time, PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }
  std::size_t prev_trajectory_size = prev_trajectory->NumOfPoints();
  if (prev_trajectory_size <= 1) {
    return;
  }
  const double time_diff = current_time - prev_trajectory->header_time();
  auto matched_point = prev_trajectory->Evaluate(time_diff);
  if (!matched_point.has_path_point()) {
    return;
  }
  const double cos_theta = std::cos(-matched_point.path_point().theta());
  const double sin_theta = std::sin(-matched_point.path_point().theta());
  std::vector<TrajectoryPoint> transformed_points;
  for (const auto& old_point : prev_trajectory->trajectory_points()) {
    TrajectoryPoint point = old_point;
    Eigen::Vector3d before_rotate(
        old_point.path_point().x() - matched_point.path_point().x(),
        old_point.path_point().y() - matched_point.path_point().y(),
        old_point.path_point().z() - matched_point.path_point().z());
    const double after_rotate_x =
        before_rotate.x() * cos_theta - before_rotate.y() * sin_theta;
    const double after_rotate_y =
        before_rotate.x() * sin_theta + before_rotate.y() * cos_theta;
    point.mutable_path_point()->set_x(after_rotate_x);
    point.mutable_path_point()->set_y(after_rotate_y);
    point.mutable_path_point()->set_z(before_rotate.z());
    point.mutable_path_point()->set_theta(common::math::WrapAngle(
        old_point.path_point().theta() - matched_point.path_point().theta()));
    transformed_points.emplace_back(point);
  }
  prev_trajectory->SetTrajectoryPoints(transformed_points);
}

// only used in navigation mode
std::vector<TrajectoryPoint> TrajectoryStitcher::CalculateInitPoint(
    const VehicleState& vehicle_state, const ReferenceLine& reference_line,
    bool* is_replan) {
  CHECK_NOTNULL(is_replan);
  *is_replan = false;

  Vec2d adc_pose(vehicle_state.x(), vehicle_state.y());
  auto ref_point = reference_line.GetNearestReferencePoint(adc_pose);
  double distance = DistanceXY(ref_point, adc_pose);
  constexpr double kEpsilon = 0.01;
  if (distance - kEpsilon > FLAGS_replan_lateral_distance_threshold) {
    Vec2d shift_direction = adc_pose - ref_point;
    shift_direction.Normalize();
    ref_point +=
        shift_direction * (distance - FLAGS_replan_lateral_distance_threshold);
    *is_replan = true;
    AWARN << "Replan is triggered. distance = " << distance;
  }
  std::vector<TrajectoryPoint> trajectory_points;
  trajectory_points.emplace_back();
  auto& init_point = trajectory_points.back();
  init_point.mutable_path_point()->CopyFrom(ref_point.ToPathPoint(0.0));
  init_point.set_v(vehicle_state.linear_velocity());
  init_point.set_a(vehicle_state.linear_acceleration());
  init_point.set_relative_time(0.0);
  return trajectory_points;
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

  std::size_t matched_index = prev_trajectory->QueryNearestPoint(veh_rel_time);

  if (matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  if (matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  auto matched_point = prev_trajectory->Evaluate(veh_rel_time);

  if (!matched_point.has_path_point()) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  auto nearest_point_index = prev_trajectory->QueryNearestPoint(
      Vec2d(vehicle_state.x(), vehicle_state.y()));
  auto nearest_point = prev_trajectory->TrajectoryPointAt(nearest_point_index);

  const double lat_diff =
      std::hypot(nearest_point.path_point().x() - vehicle_state.x(),
                 nearest_point.path_point().y() - vehicle_state.y());
  const double lon_diff = std::fabs(nearest_point.path_point().s() -
                                    matched_point.path_point().s());
  ADEBUG << "Control lateral diff: " << lat_diff
         << ", longitudinal diff: " << lon_diff;

  if (lat_diff > FLAGS_replan_lateral_distance_threshold ||
      lon_diff > FLAGS_replan_longitudinal_distance_threshold) {
    AERROR << "the distance between matched point and actual position is too "
              "large. Replan is triggered. lat_diff = "
           << lat_diff << ", lon_diff = " << lon_diff;
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  double forward_rel_time =
      prev_trajectory->TrajectoryPointAt(matched_index).relative_time() +
      planning_cycle_time;

  std::size_t forward_index =
      prev_trajectory->QueryNearestPoint(forward_rel_time);

  ADEBUG << "matched_index: " << matched_index;
  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->trajectory_points().begin() +
          std::max(0, static_cast<int>(matched_index - 1)),
      prev_trajectory->trajectory_points().begin() + forward_index + 1);

  const double zero_s = matched_point.path_point().s();

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

}  // namespace planning
}  // namespace apollo
