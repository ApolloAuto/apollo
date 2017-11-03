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
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/planning/math/curve_math.h"

namespace apollo {
namespace planning {

QpSplineReferenceLineSmoother::QpSplineReferenceLineSmoother(
    const QpSplineReferenceLineSmootherConfig& config,
    Spline2dSolver* const spline_solver)
    : smoother_config_(config), spline_solver_(spline_solver) {
  CHECK_NOTNULL(spline_solver);
}

void QpSplineReferenceLineSmoother::Clear() {
  t_knots_.clear();
  ref_points_.clear();
}

bool QpSplineReferenceLineSmoother::Smooth(
    const ReferenceLine& raw_reference_line,
    ReferenceLine* const smoothed_reference_line) {
  Clear();
  std::vector<ReferencePoint> ref_points;
  if (!Sampling(raw_reference_line)) {
    AERROR << "Fail to sample reference line smoother points!";
    return false;
  }

  spline_solver_->Reset(t_knots_, smoother_config_.spline_order());

  if (!AddConstraint(raw_reference_line)) {
    AERROR << "Add constraint for spline smoother failed";
    return false;
  }

  if (!AddKernel()) {
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
    const double heading = std::atan2(spline_solver_->spline().DerivativeY(t),
                                      spline_solver_->spline().DerivativeX(t));
    const double kappa = CurveMath::ComputeCurvature(
        spline_solver_->spline().DerivativeX(t),
        spline_solver_->spline().SecondDerivativeX(t),
        spline_solver_->spline().DerivativeY(t),
        spline_solver_->spline().SecondDerivativeY(t));
    const double dkappa = CurveMath::ComputeCurvatureDerivative(
        spline_solver_->spline().DerivativeX(t),
        spline_solver_->spline().SecondDerivativeX(t),
        spline_solver_->spline().ThirdDerivativeX(t),
        spline_solver_->spline().DerivativeY(t),
        spline_solver_->spline().SecondDerivativeY(t),
        spline_solver_->spline().ThirdDerivativeY(t));

    std::pair<double, double> xy = spline(t);
    xy.first += ref_x_;
    xy.second += ref_y_;
    common::SLPoint ref_sl_point;
    if (!raw_reference_line.XYToSL({xy.first, xy.second}, &ref_sl_point)) {
      return false;
    }
    if (ref_sl_point.s() < 0 ||
        ref_sl_point.s() > raw_reference_line.Length()) {
      continue;
    }

    ReferencePoint rlp = raw_reference_line.GetReferencePoint(ref_sl_point.s());
    ref_points.emplace_back(ReferencePoint(
        hdmap::MapPathPoint(common::math::Vec2d(xy.first, xy.second), heading,
                            rlp.lane_waypoints()),
        kappa, dkappa, 0.0, 0.0));
  }

  ReferencePoint::RemoveDuplicates(&ref_points);
  if (ref_points.size() < 2) {
    AERROR << "Fail to generate smoothed reference line.";
    return false;
  }
  *smoothed_reference_line = ReferenceLine(ref_points);
  return true;
}

bool QpSplineReferenceLineSmoother::Sampling(
    const ReferenceLine& raw_reference_line) {
  const double length = raw_reference_line.Length();
  ADEBUG << "Length = " << length;
  uint32_t num_spline = std::max(
      1u, static_cast<uint32_t>(length / smoother_config_.max_spline_length()));
  const double delta_s = length / num_spline;
  double s = 0.0;
  for (std::uint32_t i = 0; i <= num_spline; ++i, s += delta_s) {
    ReferencePoint rlp = raw_reference_line.GetReferencePoint(s);
    if (i == 0) {
      ref_x_ = rlp.x();
      ref_y_ = rlp.y();
    }
    common::PathPoint path_point;
    path_point.set_x(rlp.x() - ref_x_);
    path_point.set_y(rlp.y() - ref_y_);
    path_point.set_theta(rlp.heading());
    path_point.set_s(s);
    ref_points_.push_back(std::move(path_point));
    t_knots_.push_back(i * 1.0);
  }
  return true;
}

bool QpSplineReferenceLineSmoother::AddConstraint(
    const ReferenceLine& raw_reference_line) {
  uint32_t constraint_num =
      smoother_config_.constraint_to_knots_ratio() * (t_knots_.size() - 1) + 1;

  std::vector<double> evaluated_t;
  common::util::uniform_slice(t_knots_.front(), t_knots_.back(),
                              constraint_num - 1, &evaluated_t);

  std::vector<common::PathPoint> path_points;
  if (!ExtractEvaluatedPoints(raw_reference_line, evaluated_t, &path_points)) {
    AERROR << "Extract evaluated points failed";
    return false;
  }

  // Add x, y boundary constraint
  std::vector<double> headings;
  std::vector<double> longitudinal_bound;
  std::vector<double> lateral_bound;
  std::vector<common::math::Vec2d> xy_points;
  for (std::uint32_t i = 0; i < path_points.size(); ++i) {
    headings.push_back(path_points[i].theta());
    // TODO(all): change the langitudianl and lateral direction in code
    longitudinal_bound.push_back(smoother_config_.lateral_boundary_bound());
    lateral_bound.push_back(smoother_config_.longitudinal_boundary_bound());
    xy_points.emplace_back(path_points[i].x(), path_points[i].y());
  }

  static constexpr double kFixedBoundLimit = 0.01;
  if (longitudinal_bound.size() > 0) {
    longitudinal_bound.front() = kFixedBoundLimit;
    longitudinal_bound.back() = kFixedBoundLimit;
  }

  if (lateral_bound.size() > 0) {
    lateral_bound.front() = kFixedBoundLimit;
    lateral_bound.back() = kFixedBoundLimit;
  }

  CHECK_EQ(evaluated_t.size(), headings.size());
  CHECK_EQ(evaluated_t.size(), xy_points.size());
  CHECK_EQ(evaluated_t.size(), longitudinal_bound.size());
  CHECK_EQ(evaluated_t.size(), lateral_bound.size());

  auto* spline_constraint = spline_solver_->mutable_constraint();
  if (!spline_constraint->Add2dBoundary(evaluated_t, headings, xy_points,
                                        longitudinal_bound, lateral_bound)) {
    AERROR << "Add 2d boundary constraint failed";
    return false;
  }

  if (!spline_constraint->AddSecondDerivativeSmoothConstraint()) {
    AERROR << "Add jointness constraint failed";
    return false;
  }

  return true;
}

bool QpSplineReferenceLineSmoother::AddKernel() {
  Spline2dKernel* kernel = spline_solver_->mutable_kernel();

  // add spline kernel
  if (smoother_config_.second_derivative_weight() > 0.0) {
    kernel->AddSecondOrderDerivativeMatrix(
        smoother_config_.second_derivative_weight());
  }
  if (smoother_config_.third_derivative_weight() > 0.0) {
    kernel->AddThirdOrderDerivativeMatrix(
        smoother_config_.third_derivative_weight());
  }

  kernel->AddRegularization(smoother_config_.regularization_weight());
  return true;
}

bool QpSplineReferenceLineSmoother::Solve() { return spline_solver_->Solve(); }

bool QpSplineReferenceLineSmoother::ExtractEvaluatedPoints(
    const ReferenceLine& raw_reference_line, const std::vector<double>& vec_t,
    std::vector<common::PathPoint>* const path_points) const {
  for (const auto t : vec_t) {
    double s = 0.0;
    if (!GetSFromParamT(t, &s)) {
      AERROR << "get s from " << t << " failed";
      return false;
    }
    const ReferencePoint rlp = raw_reference_line.GetReferencePoint(s);
    common::PathPoint path_point;
    path_point.set_x(rlp.x() - ref_x_);
    path_point.set_y(rlp.y() - ref_y_);
    path_point.set_theta(rlp.heading());
    path_point.set_s(s);
    path_points->push_back(std::move(path_point));
  }
  return true;
}

bool QpSplineReferenceLineSmoother::GetSFromParamT(const double t,
                                                   double* const s) const {
  if (t_knots_.size() < 2) {
    AERROR << "Fail to GetSFromParamT because t_knots_.size() error.";
    return false;
  }

  if (fabs(t - t_knots_.front()) < 1e-8) {
    *s = ref_points_.front().s();
    return true;
  }
  if (fabs(t - t_knots_.back()) < 1e-8) {
    *s = ref_points_.back().s();
    return true;
  }
  if (t < t_knots_.front() || t > t_knots_.back()) {
    AERROR << "Fail to GetSFromParamT. t = " << t;
    return false;
  }

  std::uint32_t upper = FindIndex(t);
  std::uint32_t lower = upper - 1;

  const double r = (t - t_knots_[lower]) / (t_knots_[upper] - t_knots_[lower]);
  *s = ref_points_[lower].s() * (1.0 - r) + ref_points_[upper].s() * r;
  return true;
}

std::uint32_t QpSplineReferenceLineSmoother::FindIndex(const double t) const {
  auto upper_bound = std::upper_bound(t_knots_.begin(), t_knots_.end(), t);
  return std::distance(t_knots_.begin(), upper_bound);
}

}  // namespace planning
}  // namespace apollo
