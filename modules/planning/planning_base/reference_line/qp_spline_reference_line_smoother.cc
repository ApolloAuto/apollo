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
#include "modules/planning/planning_base/reference_line/qp_spline_reference_line_smoother.h"

#include <algorithm>
#include <utility>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"

#include "cyber/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_base/math/curve_math.h"
#include "modules/planning/planning_base/math/smoothing_spline/osqp_spline_2d_solver.h"

namespace apollo {
namespace planning {

QpSplineReferenceLineSmoother::QpSplineReferenceLineSmoother(
    const ReferenceLineSmootherConfig& config)
    : ReferenceLineSmoother(config) {
  spline_solver_.reset(
      new OsqpSpline2dSolver(t_knots_, config.qp_spline().spline_order()));
}

void QpSplineReferenceLineSmoother::Clear() { t_knots_.clear(); }

bool QpSplineReferenceLineSmoother::Smooth(
    const ReferenceLine& raw_reference_line,
    ReferenceLine* const smoothed_reference_line) {
  Clear();
  const double kEpsilon = 1e-6;
  if (!Sampling()) {
    AERROR << "Fail to sample reference line smoother points!";
    return false;
  }

  spline_solver_->Reset(t_knots_, config_.qp_spline().spline_order());

  if (!AddConstraint()) {
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
      (end_t - start_t) / (config_.num_of_total_points() - 1);
  double t = start_t;
  std::vector<ReferencePoint> ref_points;
  const auto& spline = spline_solver_->spline();
  for (std::uint32_t i = 0; i < config_.num_of_total_points() && t < end_t;
       ++i, t += resolution) {
    const double heading =
        std::atan2(spline.DerivativeY(t), spline.DerivativeX(t));
    const double kappa = CurveMath::ComputeCurvature(
        spline.DerivativeX(t), spline.SecondDerivativeX(t),
        spline.DerivativeY(t), spline.SecondDerivativeY(t));
    const double dkappa = CurveMath::ComputeCurvatureDerivative(
        spline.DerivativeX(t), spline.SecondDerivativeX(t),
        spline.ThirdDerivativeX(t), spline.DerivativeY(t),
        spline.SecondDerivativeY(t), spline.ThirdDerivativeY(t));

    std::pair<double, double> xy = spline(t);
    xy.first += ref_x_;
    xy.second += ref_y_;
    common::SLPoint ref_sl_point;
    if (!raw_reference_line.XYToSL({xy.first, xy.second}, &ref_sl_point)) {
      return false;
    }
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
    ref_points.emplace_back(ReferencePoint(
        hdmap::MapPathPoint(common::math::Vec2d(xy.first, xy.second), heading,
                            new_lane_waypoints),
        kappa, dkappa));
  }

  ReferencePoint::RemoveDuplicates(&ref_points);
  if (ref_points.size() < 2) {
    AERROR << "Fail to generate smoothed reference line.";
    return false;
  }
  *smoothed_reference_line = ReferenceLine(ref_points);
  return true;
}

bool QpSplineReferenceLineSmoother::Sampling() {
  const double length = anchor_points_.back().path_point.s() -
                        anchor_points_.front().path_point.s();
  uint32_t num_spline =
      std::max(1u, static_cast<uint32_t>(
                       length / config_.qp_spline().max_spline_length() + 0.5));
  for (std::uint32_t i = 0; i <= num_spline; ++i) {
    t_knots_.push_back(i * 1.0);
  }
  // normalize point xy
  ref_x_ = anchor_points_.front().path_point.x();
  ref_y_ = anchor_points_.front().path_point.y();
  return true;
}

bool QpSplineReferenceLineSmoother::AddConstraint() {
  // Add x, y boundary constraint
  std::vector<double> headings;
  std::vector<double> longitudinal_bound;
  std::vector<double> lateral_bound;
  std::vector<common::math::Vec2d> xy_points;
  for (const auto& point : anchor_points_) {
    const auto& path_point = point.path_point;
    headings.push_back(path_point.theta());
    longitudinal_bound.push_back(point.longitudinal_bound);
    lateral_bound.push_back(point.lateral_bound);
    xy_points.emplace_back(path_point.x() - ref_x_, path_point.y() - ref_y_);
  }
  const double scale = (anchor_points_.back().path_point.s() -
                        anchor_points_.front().path_point.s()) /
                       (t_knots_.back() - t_knots_.front());
  std::vector<double> evaluated_t;
  for (const auto& point : anchor_points_) {
    evaluated_t.emplace_back(point.path_point.s() / scale);
  }

  auto* spline_constraint = spline_solver_->mutable_constraint();

  // all points (x, y) should not deviate anchor points by a bounding box
  if (!spline_constraint->Add2dBoundary(evaluated_t, headings, xy_points,
                                        longitudinal_bound, lateral_bound)) {
    AERROR << "Add 2d boundary constraint failed.";
    return false;
  }

  // the heading of the first point should be identical to the anchor point.

  if (FLAGS_enable_reference_line_stitching &&
      !spline_constraint->AddPointAngleConstraint(evaluated_t.front(),
                                                  headings.front())) {
    AERROR << "Add 2d point angle constraint failed.";
    return false;
  }

  // all spline should be connected smoothly to the second order derivative.
  if (!spline_constraint->AddSecondDerivativeSmoothConstraint()) {
    AERROR << "Add jointness constraint failed.";
    return false;
  }

  return true;
}

bool QpSplineReferenceLineSmoother::AddKernel() {
  Spline2dKernel* kernel = spline_solver_->mutable_kernel();

  // add spline kernel
  if (config_.qp_spline().second_derivative_weight() > 0.0) {
    kernel->AddSecondOrderDerivativeMatrix(
        config_.qp_spline().second_derivative_weight());
  }
  if (config_.qp_spline().third_derivative_weight() > 0.0) {
    kernel->AddThirdOrderDerivativeMatrix(
        config_.qp_spline().third_derivative_weight());
  }

  kernel->AddRegularization(config_.qp_spline().regularization_weight());
  return true;
}

bool QpSplineReferenceLineSmoother::Solve() { return spline_solver_->Solve(); }

void QpSplineReferenceLineSmoother::SetAnchorPoints(
    const std::vector<AnchorPoint>& anchor_points) {
  CHECK_GE(anchor_points.size(), 2U);
  anchor_points_ = anchor_points;
}

}  // namespace planning
}  // namespace apollo
