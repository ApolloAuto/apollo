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

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/planning/math/discrete_points_math.h"
#include "modules/planning/math/discretized_points_smoothing/fem_pos_deviation_smoother.h"
#include "modules/planning/math/piecewise_jerk/piecewise_jerk_speed_problem.h"

namespace apollo {
namespace planning {

using apollo::common::PathPoint;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::LineSegment2d;
using apollo::common::math::NormalizeAngle;
using apollo::common::math::Vec2d;

IterativeAnchoringSmoother::IterativeAnchoringSmoother() {
  const auto& vehicle_param =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  ego_length_ = vehicle_param.length();
  ego_width_ = vehicle_param.width();
  center_shift_distance_ =
      ego_length_ / 2.0 - vehicle_param.back_edge_to_center();
}

bool IterativeAnchoringSmoother::Smooth(
    const Eigen::MatrixXd& xWS, const double init_a, const double init_v,
    const std::vector<std::vector<Vec2d>>& obstacles_vertices_vec,
    DiscretizedTrajectory* discretized_trajectory) {
  if (xWS.cols() < 2) {
    AERROR << "reference points size smaller than two, smoother early "
              "returned";
    return false;
  }
  const auto start_timestamp = std::chrono::system_clock::now();

  // Set gear of the trajectory
  gear_ = CheckGear(xWS);

  // Set obstacle in form of linesegments
  std::vector<std::vector<LineSegment2d>> obstacles_linesegments_vec;
  for (const auto& obstacle_vertices : obstacles_vertices_vec) {
    size_t vertices_num = obstacle_vertices.size();
    std::vector<LineSegment2d> obstacle_linesegments;
    for (size_t i = 0; i + 1 < vertices_num; ++i) {
      LineSegment2d line_segment =
          LineSegment2d(obstacle_vertices[i], obstacle_vertices[i + 1]);
      obstacle_linesegments.emplace_back(line_segment);
    }
    obstacles_linesegments_vec.emplace_back(obstacle_linesegments);
  }
  obstacles_linesegments_vec_ = std::move(obstacles_linesegments_vec);

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
  end_vec.SelfRotate(NormalizeAngle(end_heading + M_PI));
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

  const auto path_smooth_start_timestamp = std::chrono::system_clock::now();

  // Smooth path to have smoothed x, y, phi, kappa and s
  DiscretizedPath smoothed_path_points;
  if (!SmoothPath(interpolated_warm_start_path, bounds,
                  &smoothed_path_points)) {
    return false;
  }

  const auto path_smooth_end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> path_smooth_diff =
      path_smooth_end_timestamp - path_smooth_start_timestamp;
  ADEBUG << "iterative anchoring smoother totoal time: "
         << path_smooth_diff.count() * 1000.0 << " ms.";

  const auto speed_smooth_start_timestamp = std::chrono::system_clock::now();

  // Smooth speed to have smoothed v and a
  SpeedData smoothed_speeds;
  if (!SmoothSpeed(init_a, init_v, smoothed_path_points.Length(),
                   &smoothed_speeds)) {
    return false;
  }

  const auto speed_smooth_end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> speed_smooth_diff =
      speed_smooth_end_timestamp - speed_smooth_start_timestamp;
  ADEBUG << "iterative anchoring smoother totoal time: "
         << speed_smooth_diff.count() * 1000.0 << " ms.";

  // Combine path and speed
  if (!CombinePathAndSpeed(smoothed_path_points, smoothed_speeds,
                           discretized_trajectory)) {
    return false;
  }

  AdjustPathAndSpeedByGear(discretized_trajectory);

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  ADEBUG << "iterative anchoring smoother totoal time: "
         << diff.count() * 1000.0 << " ms.";
  return true;
}

bool IterativeAnchoringSmoother::SmoothPath(
    const DiscretizedPath& raw_path_points, const std::vector<double>& bounds,
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
  size_t counter = 0;
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

    is_collision_free =
        CheckCollisionAvoidance(*smoothed_path_points, &colliding_point_index);
    ADEBUG << "loop iteration number is " << counter;
  }
  return true;
}

bool IterativeAnchoringSmoother::CheckCollisionAvoidance(
    const DiscretizedPath& path_points,
    std::vector<size_t>* colliding_point_index) {
  CHECK_NOTNULL(colliding_point_index);

  colliding_point_index->clear();
  size_t path_points_size = path_points.size();
  for (size_t i = 0; i < path_points_size; ++i) {
    const double heading = gear_
                               ? path_points[i].theta()
                               : NormalizeAngle(path_points[i].theta() + M_PI);
    Box2d ego_box(
        {path_points[i].x() + center_shift_distance_ * std::cos(heading),
         path_points[i].y() + center_shift_distance_ * std::sin(heading)},
        heading, ego_length_, ego_width_);
    for (const auto& obstacle_linesegments : obstacles_linesegments_vec_) {
      for (const LineSegment2d& linesegment : obstacle_linesegments) {
        if (ego_box.HasOverlap(linesegment)) {
          colliding_point_index->push_back(i);
        }
      }
    }
  }

  if (!colliding_point_index->empty()) {
    return false;
  }
  return true;
}

void IterativeAnchoringSmoother::AdjustPathBounds(
    const std::vector<size_t>& colliding_point_index,
    std::vector<double>* bounds) {
  CHECK_NOTNULL(bounds);
  CHECK_GT(bounds->size(), *(std::max_element(colliding_point_index.begin(),
                                              colliding_point_index.end())));
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
  CHECK_NOTNULL(raw_path_points);
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
  CHECK_GT(xWS.size(), 1);
  double init_heading_angle = xWS(2, 0);
  const Vec2d init_tracking_vector(xWS(0, 1) - xWS(0, 0),
                                   xWS(1, 1) - xWS(1, 0));
  double init_tracking_angle = init_tracking_vector.Angle();
  return std::abs(NormalizeAngle(init_tracking_angle - init_heading_angle)) <
         M_PI_2;
}

bool IterativeAnchoringSmoother::SmoothSpeed(const double init_a,
                                             const double init_v,
                                             const double path_length,
                                             SpeedData* smoothed_speeds) {
  // TODO(Jinyun): move to confs
  const double max_v = 2.0;
  const double max_acc = 1.0;
  const double max_acc_jerk = 3.0;
  const double delta_t = 0.2;
  const double total_t = 8.0;
  const size_t num_of_knots = static_cast<size_t>(total_t / delta_t) + 1;
  std::array<double, 3> init_s = {0.0, std::abs(init_v), std::abs(init_a)};
  std::array<double, 3> end_s = {path_length, 0.0, 0.0};

  PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, delta_t,
                                                   init_s);

  piecewise_jerk_problem.set_end_state_ref({10000.0, 0.0, 0.0}, end_s);

  // TODO(Jinyun): tune the params and move to a config
  piecewise_jerk_problem.set_weight_ddx(1.0);
  piecewise_jerk_problem.set_weight_dddx(0.01);
  piecewise_jerk_problem.set_x_bounds(0.0, path_length);
  piecewise_jerk_problem.set_dx_bounds(0.0, std::fmax(max_v, std::abs(init_v)));
  piecewise_jerk_problem.set_ddx_bounds(-max_acc, max_acc);
  piecewise_jerk_problem.set_dddx_bound(max_acc_jerk);

  // Solve the problem
  if (!piecewise_jerk_problem.Optimize()) {
    AERROR << "Piecewise jerk speed optimizer failed!";
    return false;
  }

  // Extract output
  const std::vector<double>& s = piecewise_jerk_problem.opt_x();
  const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
  const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();

  // Assign speed point by gear
  SpeedData speed_data;
  speed_data.AppendSpeedPoint(s[0], 0.0, ds[0], dds[0], 0.0);
  for (size_t i = 1; i < num_of_knots; ++i) {
    // Avoid the very last points when already stopped
    if (s[i] - s[i - 1] <= 0.0 || ds[i] <= 0.0) {
      break;
    }
    speed_data.AppendSpeedPoint(s[i], delta_t * static_cast<double>(i), ds[i],
                                dds[i], (dds[i] - dds[i - 1]) / delta_t);
  }

  // Check the speed planning ends at desired path length
  if (speed_data.back().s() < path_length) {
    AERROR << "speed_data.back().s() is " << speed_data.back().s()
           << "and path_length is " << path_length;
    return false;
  }

  return true;
}

bool IterativeAnchoringSmoother::CombinePathAndSpeed(
    const DiscretizedPath& path_points, const SpeedData& speed_points,
    DiscretizedTrajectory* discretized_trajectory) {
  // TODO(Jinyun): move to confs
  const double kDenseTimeResoltuion = 0.2;
  if (path_points.empty()) {
    AERROR << "path data is empty";
    return false;
  }
  for (double cur_rel_time = 0.0; cur_rel_time < speed_points.TotalTime();
       cur_rel_time += kDenseTimeResoltuion) {
    common::SpeedPoint speed_point;
    if (!speed_points.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_points.Length()) {
      break;
    }

    common::PathPoint path_point = path_points.Evaluate(speed_point.s());

    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t());
    discretized_trajectory->AppendTrajectoryPoint(trajectory_point);
  }
  return true;
}

void IterativeAnchoringSmoother::AdjustPathAndSpeedByGear(
    DiscretizedTrajectory* discretized_trajectory) {
  if (gear_) {
    return;
  }
  std::for_each(
      discretized_trajectory->begin(), discretized_trajectory->end(),
      [](TrajectoryPoint& trajectory_point) {
        trajectory_point.mutable_path_point()->set_theta(
            NormalizeAngle(trajectory_point.path_point().theta() + M_PI));
        trajectory_point.mutable_path_point()->set_s(
            -1.0 * trajectory_point.path_point().s());
        trajectory_point.mutable_path_point()->set_kappa(
            -1.0 * trajectory_point.path_point().kappa());
        // dkappa stays the same as direction of both kappa and s are reversed
        trajectory_point.set_v(-1.0 * trajectory_point.v());
        trajectory_point.set_a(-1.0 * trajectory_point.a());
      });
}
}  // namespace planning
}  // namespace apollo
