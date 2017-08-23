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
 * @file reference_line_smooother.cpp
 **/
#include "modules/planning/reference_line/reference_line_smoother.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/util/file.h"
#include "modules/planning/math/curve_math.h"
#include "modules/planning/math/double.h"

namespace apollo {
namespace planning {

bool ReferenceLineSmoother::Init(const std::string& config_file) {
  if (!common::util::GetProtoFromFile(config_file, &smoother_config_)) {
    AERROR << "failed to load config file " << config_file;
    return false;
  }
  return true;
}

void ReferenceLineSmoother::Init(const ReferenceLineSmootherConfig& config) {
  smoother_config_ = config;
}

void ReferenceLineSmoother::Reset() {
  t_knots_.clear();
  ref_points_.clear();
  spline_solver_.reset(nullptr);
}

bool ReferenceLineSmoother::smooth(
    const ReferenceLine& raw_reference_line,
    ReferenceLine* const smoothed_reference_line) {
  Reset();
  std::vector<ReferencePoint> ref_points;
  if (!sampling(raw_reference_line)) {
    AERROR << "Fail to sample reference line smoother points!";
    return false;
  }

  spline_solver_.reset(
      new Spline2dSolver(t_knots_, smoother_config_.spline_order()));

  if (!apply_constraint(raw_reference_line)) {
    AERROR << "Add constraint for spline smoother failed";
    return false;
  }

  if (!ApplyKernel()) {
    AERROR << "Add kernel for spline smoother failed.";
    return false;
  }

  if (!Solve()) {
    AERROR << "Solve spline smoother problem failed";
  }

  // mapping spline to reference line point
  const double start_t = t_knots_.front();
  const double end_t = t_knots_.back();

  const double resolution =
      (end_t - start_t) / (smoother_config_.num_of_total_points() - 1);
  double t = start_t;
  const auto& spline = spline_solver_->spline();
  for (std::uint32_t i = 0;
       i < smoother_config_.num_of_total_points() && t < end_t;
       ++i, t += resolution) {
    std::pair<double, double> xy = spline(t);
    const double heading = std::atan2(spline_solver_->spline().derivative_y(t),
                                      spline_solver_->spline().DerivativeX(t));
    const double kappa = CurveMath::ComputeCurvature(
        spline_solver_->spline().DerivativeX(t),
        spline_solver_->spline().SecondDerivativeX(t),
        spline_solver_->spline().derivative_y(t),
        spline_solver_->spline().second_derivative_y(t));
    const double dkappa = CurveMath::ComputeCurvatureDerivative(
        spline_solver_->spline().DerivativeX(t),
        spline_solver_->spline().SecondDerivativeX(t),
        spline_solver_->spline().ThirdDerivativeX(t),
        spline_solver_->spline().derivative_y(t),
        spline_solver_->spline().second_derivative_y(t),
        spline_solver_->spline().third_derivative_y(t));

    common::SLPoint ref_sl_point;
    if (!raw_reference_line.XYToSL({xy.first, xy.second}, &ref_sl_point)) {
      return false;
    }
    if (ref_sl_point.s() < 0 ||
        ref_sl_point.s() > raw_reference_line.length()) {
      continue;
    }

    ReferencePoint rlp =
        raw_reference_line.get_reference_point(ref_sl_point.s());
    ref_points.emplace_back(ReferencePoint(
        hdmap::MapPathPoint(common::math::Vec2d(xy.first, xy.second), heading,
                            rlp.lane_waypoints()),
        kappa, dkappa, 0.0, 0.0));
  }
  ReferencePoint::remove_duplicates(&ref_points);
  *smoothed_reference_line = ReferenceLine(ref_points);
  return true;
}

bool ReferenceLineSmoother::sampling(const ReferenceLine& raw_reference_line) {
  const double length = raw_reference_line.length();
  const double resolution = length / smoother_config_.num_spline();
  double accumulated_s = 0.0;
  for (std::uint32_t i = 0;
       i <= smoother_config_.num_spline() && accumulated_s <= length;
       ++i, accumulated_s += resolution) {
    ReferencePoint rlp = raw_reference_line.get_reference_point(accumulated_s);
    common::PathPoint path_point;
    path_point.set_x(rlp.x());
    path_point.set_y(rlp.y());
    path_point.set_theta(rlp.heading());
    path_point.set_s(accumulated_s);
    ref_points_.push_back(std::move(path_point));
    t_knots_.push_back(i);
  }
  return true;
}

bool ReferenceLineSmoother::apply_constraint(
    const ReferenceLine& raw_reference_line) {
  const double t_length = t_knots_.back() - t_knots_.front();
  const double dt = t_length / (smoother_config_.num_evaluated_points() - 1);
  std::vector<double> evaluated_t;
  double accumulated_eval_t = 0.0;
  for (std::uint32_t i = 0; i < smoother_config_.num_evaluated_points();
       ++i, accumulated_eval_t += dt) {
    evaluated_t.push_back(accumulated_eval_t);
  }
  std::vector<common::PathPoint> path_points;
  if (!extract_evaluated_points(raw_reference_line, evaluated_t,
                                &path_points)) {
    AERROR << "Extract evaluated points failed";
    return false;
  }

  // Add x, y boundary constraint
  std::vector<double> angles;
  std::vector<double> longitidinal_bound;
  std::vector<double> lateral_bound;
  std::vector<common::math::Vec2d> xy_points;
  for (std::uint32_t i = 0; i < path_points.size(); ++i) {
    angles.push_back(path_points[i].theta());
    longitidinal_bound.push_back(smoother_config_.boundary_bound());
    lateral_bound.push_back(smoother_config_.boundary_bound());
    xy_points.emplace_back(path_points[i].x(), path_points[i].y());
  }

  if (!spline_solver_->mutable_constraint()->Add2dBoundary(
          evaluated_t, angles, xy_points, longitidinal_bound, lateral_bound)) {
    AERROR << "Add 2d boundary constraint failed";
    return false;
  }

  if (!spline_solver_->mutable_constraint()
           ->AddThirdDerivativeSmoothConstraint()) {
    AERROR << "Add jointness constraint failed";
    return false;
  }

  return true;
}

bool ReferenceLineSmoother::ApplyKernel() {
  Spline2dKernel* kernel = spline_solver_->mutable_kernel();

  // add spline kernel
  if (smoother_config_.derivative_weight() > 0) {
    kernel->add_derivative_kernel_matrix(smoother_config_.derivative_weight());
  }

  if (smoother_config_.second_derivative_weight() > 0) {
    kernel->add_second_order_derivative_matrix(
        smoother_config_.second_derivative_weight());
  }

  if (smoother_config_.third_derivative_weight() > 0) {
    kernel->add_third_order_derivative_matrix(
        smoother_config_.third_derivative_weight());
  }

  // TODO(fanhaoyang): change to a configurable param
  kernel->AddRegularization(0.01);
  return true;
}

bool ReferenceLineSmoother::Solve() { return spline_solver_->Solve(); }

bool ReferenceLineSmoother::extract_evaluated_points(
    const ReferenceLine& raw_reference_line, const std::vector<double>& vec_t,
    std::vector<common::PathPoint>* const path_points) const {
  for (const auto t : vec_t) {
    double s = 0.0;
    if (!get_s_from_param_t(t, &s)) {
      AERROR << "get s from " << t << " failed";
      return false;
    }
    ReferencePoint rlp = raw_reference_line.get_reference_point(s);
    common::PathPoint path_point;
    path_point.set_x(rlp.x());
    path_point.set_y(rlp.y());
    path_point.set_theta(rlp.heading());
    path_point.set_s(s);
    path_points->push_back(std::move(path_point));
  }
  return true;
}

bool ReferenceLineSmoother::get_s_from_param_t(const double t,
                                               double* const s) const {
  if (t_knots_.size() < 2 || Double::Compare(t, t_knots_.back(), 1e-8) > 0) {
    return false;
  }
  std::uint32_t lower = find_index(t);
  std::uint32_t upper = lower + 1;
  double weight = 0.0;
  if (Double::Compare(t_knots_[upper], t_knots_[lower], 1e-8) > 0) {
    weight = (t - t_knots_[lower]) / (t_knots_[upper] - t_knots_[lower]);
  }
  *s =
      ref_points_[lower].s() * (1.0 - weight) + ref_points_[upper].s() * weight;
  return true;
}

std::uint32_t ReferenceLineSmoother::find_index(const double t) const {
  auto upper_bound = std::upper_bound(t_knots_.begin() + 1, t_knots_.end(), t);
  return std::min(t_knots_.size() - 1,
                  static_cast<std::size_t>(upper_bound - t_knots_.begin())) -
         1;
}

}  // namespace planning
}  // namespace apollo
