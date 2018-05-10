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

#include "modules/planning/constraint_checker/constraint_checker1d.h"

#include "modules/common/log.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

namespace {

bool fuzzy_within(const double v, const double lower, const double upper,
                  const double e = 1.0e-4) {
  if (v < lower - e || v > upper + e) {
    return false;
  }
  return true;
}
}  // namespace

bool ConstraintChecker1d::IsValidLongitudinalTrajectory(
    const Curve1d& lon_trajectory) {
  double t = 0.0;
  while (t < lon_trajectory.ParamLength()) {
    double v = lon_trajectory.Evaluate(1, t);  // evalute_v
    if (!fuzzy_within(v, FLAGS_speed_lower_bound, FLAGS_speed_upper_bound)) {
      return false;
    }

    double a = lon_trajectory.Evaluate(2, t);  // evaluat_a
    if (!fuzzy_within(a, FLAGS_longitudinal_acceleration_lower_bound,
                      FLAGS_longitudinal_acceleration_upper_bound)) {
      return false;
    }

    double j = lon_trajectory.Evaluate(3, t);
    if (!fuzzy_within(j, FLAGS_longitudinal_jerk_lower_bound,
                      FLAGS_longitudinal_jerk_upper_bound)) {
      return false;
    }
    t += FLAGS_trajectory_time_resolution;
  }
  return true;
}

bool ConstraintChecker1d::IsValidLateralTrajectory(
    const Curve1d& lat_trajectory, const Curve1d& lon_trajectory) {
  double t = 0.0;
  while (t < lon_trajectory.ParamLength()) {
    double s = lon_trajectory.Evaluate(0, t);
    double dd_ds = lat_trajectory.Evaluate(1, s);
    double ds_dt = lon_trajectory.Evaluate(1, t);

    double d2d_ds2 = lat_trajectory.Evaluate(2, s);
    double d2s_dt2 = lon_trajectory.Evaluate(2, t);

    double a = 0.0;
    if (s < lat_trajectory.ParamLength()) {
      a = d2d_ds2 * ds_dt * ds_dt + dd_ds * d2s_dt2;
    }

    if (!fuzzy_within(a, -FLAGS_lateral_acceleration_bound,
                      FLAGS_lateral_acceleration_bound)) {
      return false;
    }

    // this is not accurate, just an approximation...
    double j = 0.0;
    if (s < lat_trajectory.ParamLength()) {
      j = lat_trajectory.Evaluate(3, s) * lon_trajectory.Evaluate(3, t);
    }

    if (!fuzzy_within(j, -FLAGS_lateral_jerk_bound, FLAGS_lateral_jerk_bound)) {
      return false;
    }
    t += FLAGS_trajectory_time_resolution;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
