/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/*
 * @file
 */

#include "modules/planning/open_space/trajectory_smoother/iterative_anchoring_smoother.h"

#include "modules/common/math/math_utils.h"
#include "modules/planning/math/discrete_points_math.h"
#include "modules/planning/math/discretized_points_smoothing/fem_pos_deviation_smoother.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::math::Vec2d;
bool IterativeAnchoringSmoother::Smooth(
    const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    DiscretizedTrajectory* discretized_trajectory) {
  if (xWS.cols() < 2) {
    AERROR << "reference points size smaller than two, smoother early "
              "returned";
    return false;
  }
  // Set gear of the trajectory
  gear_ = CheckGear(xWS);
  // Interpolate the traj
  DiscretizedPath warm_start_path;
  size_t xWS_size = xWS.cols();
  double accumulated_s = 0.0;
  Vec2d last_path_point(xWS(0, 0), xWS(1, 0));
  for (size_t i = 0; i < xWS_size; ++i) {
    Vec2d cur_path_point(xWS(0, i), xWS(1, i));
    accumulated_s += cur_path_point.DistanceTo(last_path_point);
    PathPoint path_point;
    path_point.set_x(xWS(0, i));
    path_point.set_y(xWS(1, i));
    path_point.set_theta(xWS(2, i));
    path_point.set_s(accumulated_s);
    warm_start_path.push_back(std::move(path_point));
    last_path_point = cur_path_point;
  }

  DiscretizedPath interpolated_warm_start_path;
  // TODO(Jinyun): move to confs
  const double interpolated_delta_s = 0.1;
  const double path_length = warm_start_path.Length();
  for (double s = 0; s < path_length; s += interpolated_delta_s) {
    interpolated_warm_start_path.push_back(warm_start_path.Evaluate(s));
  }

  // Set initial heading and bounds
  const double initial_heading = xWS(2, 0);
  const double end_heading = xWS(2, xWS_size - 1);
  const size_t interpolated_path_size = interpolated_warm_start_path.size();
  CHECK_GT(interpolated_path_size, 4);
  // Adjust the point position to have heading by finite element difference of
  // the point and the other point equal to the given warm start initial or end
  // heading
  const double first_to_second_s =
      interpolated_warm_start_path[1].s() - interpolated_warm_start_path[0].s();
  Vec2d first_point(interpolated_warm_start_path[0].x(),
                    interpolated_warm_start_path[0].y());
  Vec2d initial_vec(first_to_second_s, 0);
  initial_vec.SelfRotate(initial_heading);
  initial_vec += first_point;
  PathPoint second_path_point;
  second_path_point.set_x(initial_vec.x());
  second_path_point.set_y(initial_vec.y());
  second_path_point.set_theta(interpolated_warm_start_path[1].theta());
  second_path_point.set_s(interpolated_warm_start_path[1].s());
  interpolated_warm_start_path[1] = std::move(second_path_point);

  const double second_last_to_last_s =
      interpolated_warm_start_path[interpolated_path_size - 1].s() -
      interpolated_warm_start_path[interpolated_path_size - 2].s();
  Vec2d last_point(
      interpolated_warm_start_path[interpolated_path_size - 1].x(),
      interpolated_warm_start_path[interpolated_path_size - 1].y());
  Vec2d end_vec(second_last_to_last_s, 0);
  end_vec.SelfRotate(common::math::NormalizeAngle(end_heading + M_PI));
  end_vec += last_point;
  PathPoint second_to_last_path_point;
  second_to_last_path_point.set_x(end_vec.x());
  second_to_last_path_point.set_y(end_vec.y());
  second_to_last_path_point.set_y(
      interpolated_warm_start_path[interpolated_path_size - 2].theta());
  second_to_last_path_point.set_s(
      interpolated_warm_start_path[interpolated_path_size - 2].s());
  interpolated_warm_start_path[interpolated_path_size - 2] =
      std::move(second_to_last_path_point);

  // TODO(Jinyun): move to confs
  const double default_bounds = 0.5;
  std::vector<double> bounds(interpolated_path_size, default_bounds);
  bounds[0] = 0.0;
  bounds[1] = 0.0;
  bounds[interpolated_path_size - 1] = 0.0;
  bounds[interpolated_path_size - 2] = 0.0;

  // Smooth path to have smoothed x, y, phi, kappa and s
  DiscretizedPath smoothed_path_points;
  if (!SmoothPath(interpolated_warm_start_path, bounds, obstacles_vertices_vec,
                  &smoothed_path_points)) {
    return false;
  }

  // Smooth speed to have smoothed v and a
  SpeedData smoothed_speeds;
  if (!SmoothSpeed(init_a, init_v, smoothed_path_points.Length(),
                   &smoothed_speeds)) {
    return false;
  }

  // Combine path and speed
  if (!CombinePathAndSpeed(smoothed_path_points, smoothed_speeds,
                           discretized_trajectory)) {
    return false;
  }
  return true;
}

bool IterativeAnchoringSmoother::SmoothPath(
    const DiscretizedPath& raw_path_points, const std::vector<double>& bounds,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    DiscretizedPath* smoothed_path_points) {
  std::vector<std::pair<double, double>> raw_point2d;
  std::vector<double> flexible_bounds;
  for (const auto& path_point : raw_path_points) {
    raw_point2d.emplace_back(path_point.x(), path_point.y());
  }
  flexible_bounds = bounds;

  FemPosDeviationSmoother fem_pos_smoother;
  // TODO(Jinyun): move to confs
  fem_pos_smoother.set_weight_fem_pos_deviation(1e5);
  fem_pos_smoother.set_weight_path_length(1.0);
  fem_pos_smoother.set_weight_ref_deviation(1.0);
  FemPosDeviationOsqpSettings osqp_settings;
  osqp_settings.max_iter = static_cast<int>(500);
  osqp_settings.time_limit = 0.0;
  osqp_settings.verbose = true;
  osqp_settings.scaled_termination = true;
  osqp_settings.warm_start = true;

  bool is_collision_free = false;
  std::vector<size_t> colliding_point_index;
  std::vector<std::pair<double, double>> smoothed_point2d;
  while (!is_collision_free) {
    AdjustPathBounds(colliding_point_index, &flexible_bounds);
    fem_pos_smoother.set_ref_points(raw_point2d);
    fem_pos_smoother.set_x_bounds_around_refs(flexible_bounds);
    fem_pos_smoother.set_y_bounds_around_refs(flexible_bounds);
    if (!fem_pos_smoother.Smooth(osqp_settings)) {
      AERROR << "Smoothing path fails";
      return false;
    }

    const auto& opt_x = fem_pos_smoother.opt_x();
    const auto& opt_y = fem_pos_smoother.opt_y();

    if (opt_x.size() < 2 || opt_y.size() < 2) {
      AERROR << "Return by fem_pos_smoother is wrong. Size smaller than 2 ";
      return false;
    }

    CHECK_EQ(opt_x.size(), opt_y.size());

    size_t point_size = opt_x.size();
    smoothed_point2d.clear();
    for (size_t i = 0; i < point_size; ++i) {
      smoothed_point2d.emplace_back(opt_x[i], opt_y[i]);
    }

    if (!SetPathProfile(smoothed_point2d, smoothed_path_points)) {
      AERROR << "Set path profile fails";
      return false;
    }

    is_collision_free = CheckCollision(
        *smoothed_path_points, obstacles_vertices_vec, &colliding_point_index);
  }
  return true;
}

bool IterativeAnchoringSmoother::CheckCollision(
    const DiscretizedPath& path_points,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    std::vector<size_t>* colliding_point_index) {
  return true;
}

void IterativeAnchoringSmoother::AdjustPathBounds(
    const std::vector<size_t>& colliding_point_index,
    std::vector<double>* bounds) {
  if (colliding_point_index.empty()) {
    return;
  }
  // TODO(Jinyun): move to confs
  const double decrease_ratio = 0.5;
  for (const auto index : colliding_point_index) {
    bounds->at(index) *= decrease_ratio;
  }
}

bool IterativeAnchoringSmoother::SetPathProfile(
    const std::vector<std::pair<double, double>>& point2d,
    DiscretizedPath* raw_path_points) {
  // Compute path profile
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  if (!DiscretePointsMath::ComputePathProfile(
          point2d, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }
  CHECK_EQ(point2d.size(), headings.size());
  CHECK_EQ(point2d.size(), kappas.size());
  CHECK_EQ(point2d.size(), dkappas.size());
  CHECK_EQ(point2d.size(), accumulated_s.size());

  // Adjust heading, kappa and dkappa direction according to gear
  if (!gear_) {
    std::for_each(headings.begin(), headings.end(), [](double& heading) {
      heading = common::math::NormalizeAngle(heading + M_PI);
    });
    std::for_each(kappas.begin(), kappas.end(),
                  [](double& kappa) { kappa = -kappa; });
    std::for_each(accumulated_s.begin(), accumulated_s.end(),
                  [](double& accumulated_distance) {
                    accumulated_distance = -accumulated_distance;
                  });
  }

  // Load into path point
  size_t points_size = point2d.size();
  raw_path_points->clear();
  for (size_t i = 0; i < points_size; ++i) {
    PathPoint path_point;
    path_point.set_x(point2d[i].first);
    path_point.set_y(point2d[i].second);
    path_point.set_theta(headings[i]);
    path_point.set_s(accumulated_s[i]);
    path_point.set_kappa(kappas[i]);
    path_point.set_dkappa(dkappas[i]);
    raw_path_points->push_back(std::move(path_point));
  }
  return true;
}

bool IterativeAnchoringSmoother::CheckGear(const Eigen::MatrixXd& xWS) {
  return true;
}

bool IterativeAnchoringSmoother::SmoothSpeed(const double init_a,
                                             const double init_v,
                                             const double path_length,
                                             SpeedData* smoothed_speeds) {
  return true;
}

bool IterativeAnchoringSmoother::CombinePathAndSpeed(
    const DiscretizedPath& raw_path_points, const SpeedData& smoothed_speeds,
    DiscretizedTrajectory* discretized_trajectory) {
  return true;
}
}  // namespace planning
}  // namespace apollo
