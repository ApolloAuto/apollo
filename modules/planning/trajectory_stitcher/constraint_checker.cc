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
 * @file constraint_checker.cc
 **/

#include "modules/planning/trajectory_stitcher/constraint_checker.h"

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {
template <typename T>
bool within_range(const T v, const T lower, const T upper) {
  if (v < lower || v > upper) {
    return false;
  }
  return true;
}
}

bool ConstraintChecker::ValidTrajectory(
    const DiscretizedTrajectory& trajectory) {
  for (const auto& p : trajectory.trajectory_points()) {
    double t = p.relative_time();
    double lon_v = p.v();
    if (!within_range(lon_v, FLAGS_speed_lower_bound,
                      FLAGS_speed_upper_bound)) {
      AWARN << "Velocity at relative time " << t
            << " exceeds bound, value: " << lon_v << ", bound ["
            << FLAGS_speed_lower_bound << ", " << FLAGS_speed_upper_bound
            << "].";
      return false;
    }

    double lon_a = p.a();
    if (!within_range(lon_a, FLAGS_longitudinal_acceleration_lower_bound,
                      FLAGS_longitudinal_acceleration_upper_bound)) {
      AWARN << "Longitudinal acceleration at relative time " << t
            << " exceeds bound, value: " << lon_a << ", bound ["
            << FLAGS_longitudinal_acceleration_lower_bound << ", "
            << FLAGS_longitudinal_acceleration_upper_bound << "].";
      return false;
    }

    double lat_a = lon_v * lon_v * p.path_point().kappa();
    if (!within_range(lat_a, -FLAGS_lateral_acceleration_bound,
                      FLAGS_lateral_acceleration_bound)) {
      AWARN << "Lateral acceleration at relative time " << t
            << " exceeds bound, value: " << lat_a << ", bound ["
            << -FLAGS_lateral_acceleration_bound << ", "
            << FLAGS_lateral_acceleration_bound << "].";
      return false;
    }

    double kappa = p.path_point().kappa();
    if (!within_range(kappa, -FLAGS_kappa_bound, FLAGS_kappa_bound)) {
      AWARN << "Kappa at relative time " << t
            << " exceeds bound, value: " << kappa << ", bound ["
            << -FLAGS_kappa_bound << ", " << FLAGS_kappa_bound << "].";
      return false;
    }
  }

  for (std::size_t i = 1; i < trajectory.NumOfPoints(); ++i) {
    const auto& p0 = trajectory.TrajectoryPointAt(i - 1);
    const auto& p1 = trajectory.TrajectoryPointAt(i);
    double t = p0.relative_time();

    double dt = p1.relative_time() - p0.relative_time();
    double d_lon_a = p1.a() - p0.a();
    double lon_jerk = d_lon_a / dt;
    if (!within_range(d_lon_a / dt, FLAGS_longitudinal_jerk_lower_bound,
                      FLAGS_longitudinal_jerk_upper_bound)) {
      AWARN << "Longitudinal jerk at relative time " << t
            << " exceeds bound, value: " << lon_jerk << ", bound ["
            << FLAGS_longitudinal_jerk_lower_bound << ", "
            << FLAGS_longitudinal_jerk_upper_bound << "].";
      return false;
    }

    double d_lat_a = p1.v() * p1.v() * p1.path_point().kappa() -
                     p0.v() * p0.v() * p0.path_point().kappa();
    double lat_jerk = d_lat_a / dt;
    if (!within_range(lat_jerk, -FLAGS_lateral_jerk_bound,
                      FLAGS_lateral_jerk_bound)) {
      AWARN << "Lateral jerk at relative time " << t
            << " exceeds bound, value: " << lat_jerk << ", bound ["
            << -FLAGS_lateral_jerk_bound << ", " << FLAGS_lateral_jerk_bound
            << "].";
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
