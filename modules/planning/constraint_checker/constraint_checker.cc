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

#include "modules/planning/constraint_checker/constraint_checker.h"

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {
template <typename T>
bool WithinRange(const T v, const T lower, const T upper) {
  return lower <= v && v <= upper;
}
}

bool ConstraintChecker::ValidTrajectory(
    const DiscretizedTrajectory& trajectory) {
  for (const auto& p : trajectory.trajectory_points()) {
    double t = p.relative_time();
    double lon_v = p.v();
    if (!WithinRange(lon_v, FLAGS_speed_lower_bound, FLAGS_speed_upper_bound)) {
      AERROR << "Velocity at relative time " << t
             << " exceeds bound, value: " << lon_v << ", bound ["
             << FLAGS_speed_lower_bound << ", " << FLAGS_speed_upper_bound
             << "].";
      return false;
    }

    double lon_a = p.a();
    if (!WithinRange(lon_a, FLAGS_longitudinal_acceleration_lower_bound,
                     FLAGS_longitudinal_acceleration_upper_bound)) {
      AERROR << "Longitudinal acceleration at relative time " << t
             << " exceeds bound, value: " << lon_a << ", bound ["
             << FLAGS_longitudinal_acceleration_lower_bound << ", "
             << FLAGS_longitudinal_acceleration_upper_bound << "].";
      return false;
    }

    double kappa = p.path_point().kappa();
    if (!WithinRange(kappa, -FLAGS_kappa_bound, FLAGS_kappa_bound)) {
      AERROR << "Kappa at relative time " << t
             << " exceeds bound, value: " << kappa << ", bound ["
             << -FLAGS_kappa_bound << ", " << FLAGS_kappa_bound << "].";
      return false;
    }
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
