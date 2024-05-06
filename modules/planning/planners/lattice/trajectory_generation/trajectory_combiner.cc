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

#include "modules/planning/planners/lattice/trajectory_generation/trajectory_combiner.h"

#include <algorithm>

#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/math/path_matcher.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::math::PathMatcher;

DiscretizedTrajectory TrajectoryCombiner::Combine(
    const std::vector<PathPoint>& reference_line, const Curve1d& lon_trajectory,
    const Curve1d& lat_trajectory, const double init_relative_time) {
  DiscretizedTrajectory combined_trajectory;

  double s0 = lon_trajectory.Evaluate(0, 0.0);
  double s_ref_max = reference_line.back().s();
  double accumulated_trajectory_s = 0.0;
  PathPoint prev_trajectory_point;

  double last_s = -FLAGS_numerical_epsilon;
  double t_param = 0.0;
  while (t_param < FLAGS_trajectory_time_length) {
    // linear extrapolation is handled internally in LatticeTrajectory1d;
    // no worry about t_param > lon_trajectory.ParamLength() situation
    double s = lon_trajectory.Evaluate(0, t_param);
    if (last_s > 0.0) {
      s = std::max(last_s, s);
    }
    last_s = s;

    double s_dot =
        std::max(FLAGS_numerical_epsilon, lon_trajectory.Evaluate(1, t_param));
    double s_ddot = lon_trajectory.Evaluate(2, t_param);
    if (s > s_ref_max) {
      break;
    }

    double relative_s = s - s0;
    // linear extrapolation is handled internally in LatticeTrajectory1d;
    // no worry about s_param > lat_trajectory.ParamLength() situation
    double d = lat_trajectory.Evaluate(0, relative_s);
    double d_prime = lat_trajectory.Evaluate(1, relative_s);
    double d_pprime = lat_trajectory.Evaluate(2, relative_s);

    PathPoint matched_ref_point = PathMatcher::MatchToPath(reference_line, s);

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double v = 0.0;
    double a = 0.0;

    const double rs = matched_ref_point.s();
    const double rx = matched_ref_point.x();
    const double ry = matched_ref_point.y();
    const double rtheta = matched_ref_point.theta();
    const double rkappa = matched_ref_point.kappa();
    const double rdkappa = matched_ref_point.dkappa();

    std::array<double, 3> s_conditions = {rs, s_dot, s_ddot};
    std::array<double, 3> d_conditions = {d, d_prime, d_pprime};
    CartesianFrenetConverter::frenet_to_cartesian(
        rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, &x, &y,
        &theta, &kappa, &v, &a);

    if (prev_trajectory_point.has_x() && prev_trajectory_point.has_y()) {
      double delta_x = x - prev_trajectory_point.x();
      double delta_y = y - prev_trajectory_point.y();
      double delta_s = std::hypot(delta_x, delta_y);
      accumulated_trajectory_s += delta_s;
    }

    TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->set_x(x);
    trajectory_point.mutable_path_point()->set_y(y);
    trajectory_point.mutable_path_point()->set_s(accumulated_trajectory_s);
    trajectory_point.mutable_path_point()->set_theta(theta);
    trajectory_point.mutable_path_point()->set_kappa(kappa);
    trajectory_point.set_v(v);
    trajectory_point.set_a(a);
    trajectory_point.set_relative_time(t_param + init_relative_time);

    combined_trajectory.AppendTrajectoryPoint(trajectory_point);

    t_param = t_param + FLAGS_trajectory_time_resolution;

    prev_trajectory_point = trajectory_point.path_point();
  }
  return combined_trajectory;
}

}  // namespace planning
}  // namespace apollo
