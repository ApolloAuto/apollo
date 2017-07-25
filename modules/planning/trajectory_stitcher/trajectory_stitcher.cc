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

using VehicleState = apollo::common::vehicle_state::VehicleState;

std::vector<TrajectoryPoint> TrajectoryStitcher::compute_stitching_trajectory(
    const VehicleState& vehicle_state, const Frame* const prev_frame) {
  auto compute_reinit_stitching_trajectory =
      [](const VehicleState& vehicle_state) {
        TrajectoryPoint init_point;
        init_point.mutable_path_point()->set_x(vehicle_state.x());
        init_point.mutable_path_point()->set_y(vehicle_state.y());
        init_point.set_v(vehicle_state.linear_velocity());
        init_point.set_a(vehicle_state.linear_acceleration());
        init_point.mutable_path_point()->set_theta(vehicle_state.heading());
        init_point.mutable_path_point()->set_kappa(vehicle_state.kappa());

        // TODO: the time is not correct, it should be
        // FLAGS_forward_predict_time.
        init_point.set_relative_time(0.0);

        // TODO: the init point should be some future point, not current vehicle
        // state
        // TODO: need vehicle bicycle model to compute the overhead trajectory.
        return std::vector<TrajectoryPoint>(1, init_point);
      };

  // no planning history
  if (prev_frame == nullptr) {
    return compute_reinit_stitching_trajectory(vehicle_state);
  }
  const auto& prev_trajectory =
      prev_frame->planning_data().computed_trajectory();
  std::size_t prev_trajectory_size = prev_trajectory.num_of_points();

  // previous planning is failed
  if (prev_trajectory_size == 0) {
    AWARN << "Projected trajectory at time [" << prev_trajectory.header_time()
          << "] size is zero! Use origin car status instead.";
    return compute_reinit_stitching_trajectory(vehicle_state);
  }

  const double veh_rel_time =
      vehicle_state.timestamp() - prev_trajectory.header_time();
  std::size_t matched_index = prev_trajectory.query_nearest_point(veh_rel_time);

  // the previous trajectory is not long enough; something is seriously wrong.
  if (matched_index == prev_trajectory_size) {
    return compute_reinit_stitching_trajectory(vehicle_state);
  }

  // the previous trajectory doesn't cover current time;
  if (matched_index == 0 &&
      veh_rel_time < prev_trajectory.start_point().relative_time()) {
    return compute_reinit_stitching_trajectory(vehicle_state);
  }

  auto matched_point = prev_trajectory.trajectory_point_at(matched_index);
  double position_diff =
      Eigen::Vector2d(matched_point.path_point().x() - vehicle_state.x(),
                      matched_point.path_point().y() - vehicle_state.y())
          .norm();

  // the distance between matched point and actual position is too large
  if (position_diff > FLAGS_replan_distance_threshold) {
    return compute_reinit_stitching_trajectory(vehicle_state);
  }

  double forward_rel_time = veh_rel_time + FLAGS_forward_predict_time;
  std::size_t forward_index =
      prev_trajectory.query_nearest_point(forward_rel_time);

  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory.trajectory_points().begin() + matched_index,
      prev_trajectory.trajectory_points().begin() + forward_index + 1);

  double zero_time =
      prev_trajectory.trajectory_point_at(matched_index).relative_time();
  std::for_each(stitching_trajectory.begin(), stitching_trajectory.end(),
                [&zero_time](TrajectoryPoint& tp) {
                  tp.set_relative_time(tp.relative_time() - zero_time);
                });

  return stitching_trajectory;
}

}  // namespace planning
}  // namespace apollo
