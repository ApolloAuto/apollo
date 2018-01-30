/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/lattice/trajectory_generator/trajectory_combiner.h"

#include <algorithm>

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/lattice/util/reference_line_matcher.h"
#include "modules/planning/math/frame_conversion/cartesian_frenet_conversion.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;

DiscretizedTrajectory TrajectoryCombiner::Combine(
    const std::vector<PathPoint>& reference_line, const Curve1d& lon_trajectory,
    const Curve1d& lat_trajectory, const double init_relative_time) {
  DiscretizedTrajectory combined_trajectory;

  double s0 = lon_trajectory.Evaluate(0, 0.0);
  double s_ref_max = reference_line.back().s();

  double t_param = 0.0;
  while (t_param < FLAGS_trajectory_time_length) {
    // linear extrapolation is handled internally in LatticeTrajectory1d;
    // no worry about t_param > lon_trajectory.ParamLength() situation
    double s = lon_trajectory.Evaluate(0, t_param);
    double s_dot =
        std::max(FLAGS_lattice_epsilon, lon_trajectory.Evaluate(1, t_param));
    double s_ddot = lon_trajectory.Evaluate(2, t_param);
    if (s > s_ref_max) {
      break;
    }

    double s_param = s - s0;
    // linear extrapolation is handled internally in LatticeTrajectory1d;
    // no worry about s_param > lat_trajectory.ParamLength() situation
    double d = lat_trajectory.Evaluate(0, s_param);
    double d_prime = lat_trajectory.Evaluate(1, s_param);
    double d_pprime = lat_trajectory.Evaluate(2, s_param);

    PathPoint matched_ref_point =
        ReferenceLineMatcher::MatchToReferenceLine(reference_line, s);

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

    TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->set_x(x);
    trajectory_point.mutable_path_point()->set_y(y);
    trajectory_point.mutable_path_point()->set_theta(theta);
    trajectory_point.mutable_path_point()->set_kappa(kappa);
    trajectory_point.set_v(v);
    trajectory_point.set_a(a);
    trajectory_point.set_relative_time(t_param + init_relative_time);

    combined_trajectory.AppendTrajectoryPoint(trajectory_point);

    t_param = t_param + FLAGS_trajectory_time_resolution;
  }
  return combined_trajectory;
}

}  // namespace planning
}  // namespace apollo
