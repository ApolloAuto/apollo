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

#include "modules/planning/reference_line/discrete_points_reference_line_smoother.h"

#include <algorithm>

#include "IpIpoptApplication.hpp"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/discrete_points_math.h"
#include "modules/planning/math/discretized_points_smoothing/cos_theta_smoother.h"
#include "modules/planning/math/discretized_points_smoothing/fem_pos_deviation_smoother.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;

DiscretePointsReferenceLineSmoother::DiscretePointsReferenceLineSmoother(
    const ReferenceLineSmootherConfig& config)
    : ReferenceLineSmoother(config) {
  const auto& cos_theta_config = config.discrete_points().cos_theta_smoothing();
  const auto& fem_pos_config = config.discrete_points().fem_pos_smoothing();

  use_cos_theta_ = cos_theta_config.use_cos_theta();
  weight_cos_included_angle_ = cos_theta_config.weight_cos_included_angle();
  weight_anchor_points_ = cos_theta_config.weight_anchor_points();
  weight_length_ = cos_theta_config.weight_length();
  print_level_ = cos_theta_config.print_level();
  max_num_of_iterations_ = cos_theta_config.max_num_of_iterations();
  acceptable_num_of_iterations_ =
      cos_theta_config.acceptable_num_of_iterations();
  tol_ = cos_theta_config.tol();
  acceptable_tol_ = cos_theta_config.acceptable_tol();
  use_automatic_differentiation_ =
      cos_theta_config.ipopt_use_automatic_differentiation();

  use_fem_pos_ = fem_pos_config.use_fem_pos();
  weight_fem_pos_deviation_ = fem_pos_config.weight_fem_pose_deviation();
  weight_ref_deviation_ = fem_pos_config.weight_ref_deviation();
  weight_path_length_ = fem_pos_config.weight_path_length();
  max_iter_ = fem_pos_config.max_iter();
  time_limit_ = fem_pos_config.time_limit();
  verbose_ = fem_pos_config.verbose();
  scaled_termination_ = fem_pos_config.scaled_termination();
  warm_start_ = fem_pos_config.warm_start();
}

bool DiscretePointsReferenceLineSmoother::Smooth(
    const ReferenceLine& raw_reference_line,
    ReferenceLine* const smoothed_reference_line) {
  std::vector<std::pair<double, double>> smoothed_point2d;
  std::vector<std::pair<double, double>> raw_point2d;
  std::vector<double> anchorpoints_lateralbound;

  const auto start_timestamp = std::chrono::system_clock::now();

  for (const auto& anchor_point : anchor_points_) {
    raw_point2d.emplace_back(anchor_point.path_point.x(),
                             anchor_point.path_point.y());
    anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
  }

  // fix front and back points to avoid end states deviate from the center of
  // road
  anchorpoints_lateralbound.front() = 0.0;
  anchorpoints_lateralbound.back() = 0.0;

  NormalizePoints(&raw_point2d);

  const auto solver_start_timestamp = std::chrono::system_clock::now();

  bool status = false;
  if (use_cos_theta_) {
    status = CosThetaSmooth(raw_point2d, anchorpoints_lateralbound,
                            &smoothed_point2d);
  } else if (use_fem_pos_) {
    status =
        FemPosSmooth(raw_point2d, anchorpoints_lateralbound, &smoothed_point2d);
  } else {
    AERROR << "no smoothing method chosen";
    return false;
  }

  if (!status) {
    AERROR << "discrete_points reference line smoother fails";
    return false;
  }

  const auto solver_end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> solver_diff =
      solver_end_timestamp - solver_start_timestamp;
  ADEBUG << "discrete_points reference line smoother solver time: "
         << solver_diff.count() * 1000.0 << " ms.";

  DeNormalizePoints(&smoothed_point2d);

  std::vector<ReferencePoint> ref_points;
  GenerateRefPointProfile(raw_reference_line, smoothed_point2d, &ref_points);

  ReferencePoint::RemoveDuplicates(&ref_points);
  if (ref_points.size() < 2) {
    AERROR << "Fail to generate smoothed reference line.";
    return false;
  }
  *smoothed_reference_line = ReferenceLine(ref_points);

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  ADEBUG << "discrete_points reference line smoother totoal time: "
         << diff.count() * 1000.0 << " ms.";
  return true;
}

bool DiscretePointsReferenceLineSmoother::CosThetaSmooth(
    const std::vector<std::pair<double, double>>& scaled_point2d,
    const std::vector<double>& lateral_bounds,
    std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
  CosThetaSmoother* smoother =
      new CosThetaSmoother(scaled_point2d, lateral_bounds);
  // auto smoother =
  //     std::make_shared<CosThetaSmoother>(scaled_point2d, lateral_bounds);

  smoother->set_weight_cos_included_angle(weight_cos_included_angle_);
  smoother->set_weight_anchor_points(weight_anchor_points_);
  smoother->set_weight_length(weight_length_);
  smoother->set_automatic_differentiation_flag(use_automatic_differentiation_);

  Ipopt::SmartPtr<Ipopt::TNLP> problem = smoother;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetIntegerValue("print_level",
                                  static_cast<int>(print_level_));
  app->Options()->SetIntegerValue("max_iter",
                                  static_cast<int>(max_num_of_iterations_));
  app->Options()->SetIntegerValue(
      "acceptable_iter", static_cast<int>(acceptable_num_of_iterations_));
  app->Options()->SetNumericValue("tol", tol_);
  app->Options()->SetNumericValue("acceptable_tol", acceptable_tol_);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    AERROR << "*** Error during initialization!";
    return false;
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    ADEBUG << "*** The problem solved in " << iter_count << " iterations!";
  } else {
    AERROR << "Solver fails with return code: " << static_cast<int>(status);
    return false;
  }

  std::vector<double> x;
  std::vector<double> y;
  smoother->get_optimization_results(&x, &y);
  // load the point position and estimated derivatives at each point
  if (x.size() < 2 || y.size() < 2) {
    AERROR << "Return by IPOPT is wrong. Size smaller than 2 ";
    return false;
  }

  for (size_t i = 0; i < x.size(); ++i) {
    ptr_smoothed_point2d->emplace_back(x[i], y[i]);
  }

  return true;
}

bool DiscretePointsReferenceLineSmoother::FemPosSmooth(
    const std::vector<std::pair<double, double>>& ref_points,
    const std::vector<double>& lateral_bounds,
    std::vector<std::pair<double, double>>* ptr_smoothed_point2d) {
  FemPosDeviationSmoother fem_pos_smoother;
  fem_pos_smoother.set_ref_points(ref_points);
  fem_pos_smoother.set_x_bounds_around_refs(lateral_bounds);
  fem_pos_smoother.set_y_bounds_around_refs(lateral_bounds);
  fem_pos_smoother.set_weight_fem_pos_deviation(weight_fem_pos_deviation_);
  fem_pos_smoother.set_weight_path_length(weight_path_length_);
  fem_pos_smoother.set_weight_ref_deviation(weight_ref_deviation_);

  FemPosDeviationOsqpSettings osqp_settings;
  osqp_settings.max_iter = static_cast<int>(max_iter_);
  osqp_settings.time_limit = time_limit_;
  osqp_settings.verbose = verbose_;
  osqp_settings.scaled_termination = scaled_termination_;
  osqp_settings.warm_start = warm_start_;

  if (!fem_pos_smoother.Smooth(osqp_settings)) {
    return false;
  }

  const auto& opt_x = fem_pos_smoother.opt_x();
  const auto& opt_y = fem_pos_smoother.opt_y();

  if (opt_x.size() < 2 || opt_y.size() < 2) {
    AERROR << "Return by IPOPT is wrong. Size smaller than 2 ";
    return false;
  }

  CHECK_EQ(opt_x.size(), opt_y.size());

  size_t point_size = opt_x.size();
  for (size_t i = 0; i < point_size; ++i) {
    ptr_smoothed_point2d->emplace_back(opt_x[i], opt_y[i]);
  }
  return true;
}

void DiscretePointsReferenceLineSmoother::SetAnchorPoints(
    const std::vector<AnchorPoint>& anchor_points) {
  CHECK_GT(anchor_points.size(), 1);
  anchor_points_ = anchor_points;
}

void DiscretePointsReferenceLineSmoother::NormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  zero_x_ = xy_points->front().first;
  zero_y_ = xy_points->front().second;
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x - zero_x_,
                                               curr_y - zero_y_);
                  point = std::move(xy);
                });
}

void DiscretePointsReferenceLineSmoother::DeNormalizePoints(
    std::vector<std::pair<double, double>>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](std::pair<double, double>& point) {
                  auto curr_x = point.first;
                  auto curr_y = point.second;
                  std::pair<double, double> xy(curr_x + zero_x_,
                                               curr_y + zero_y_);
                  point = std::move(xy);
                });
}

bool DiscretePointsReferenceLineSmoother::GenerateRefPointProfile(
    const ReferenceLine& raw_reference_line,
    const std::vector<std::pair<double, double>>& xy_points,
    std::vector<ReferencePoint>* reference_points) {
  // Compute path profile
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> dkappas;
  std::vector<double> accumulated_s;
  if (!DiscretePointsMath::ComputePathProfile(
          xy_points, &headings, &accumulated_s, &kappas, &dkappas)) {
    return false;
  }

  // Load into ReferencePoints
  size_t points_size = xy_points.size();
  for (size_t i = 0; i < points_size; ++i) {
    common::SLPoint ref_sl_point;
    if (!raw_reference_line.XYToSL({xy_points[i].first, xy_points[i].second},
                                   &ref_sl_point)) {
      return false;
    }
    const double kEpsilon = 1e-6;
    if (ref_sl_point.s() < -kEpsilon ||
        ref_sl_point.s() > raw_reference_line.Length()) {
      continue;
    }
    ref_sl_point.set_s(std::max(ref_sl_point.s(), 0.0));
    ReferencePoint rlp = raw_reference_line.GetReferencePoint(ref_sl_point.s());
    auto new_lane_waypoints = rlp.lane_waypoints();
    for (auto& lane_waypoint : new_lane_waypoints) {
      lane_waypoint.l = ref_sl_point.l();
    }
    reference_points->emplace_back(ReferencePoint(
        hdmap::MapPathPoint(
            common::math::Vec2d(xy_points[i].first, xy_points[i].second),
            headings[i], new_lane_waypoints),
        kappas[i], dkappas[i]));
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
