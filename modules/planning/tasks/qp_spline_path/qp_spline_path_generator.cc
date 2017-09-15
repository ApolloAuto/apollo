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
 * @file qp_spline_path_generator.cc
 **/
#include <algorithm>
#include <utility>
#include <vector>

#include "modules/planning/tasks/qp_spline_path/qp_spline_path_generator.h"

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/double.h"
#include "modules/planning/math/frame_conversion/cartesian_frenet_conversion.h"

namespace apollo {
namespace planning {

using Vec2d = apollo::common::math::Vec2d;

QpSplinePathGenerator::QpSplinePathGenerator(
    const ReferenceLine& reference_line,
    const QpSplinePathConfig& qp_spline_path_config)
    : reference_line_(reference_line),
      qp_spline_path_config_(qp_spline_path_config) {
  CHECK_GE(qp_spline_path_config_.regularization_weight(), 0.0)
      << "regularization_weight should NOT be negative.";
  CHECK_GE(qp_spline_path_config_.derivative_weight(), 0.0)
      << "derivative_weight should NOT be negative.";
  CHECK_GE(qp_spline_path_config_.second_derivative_weight(), 0.0)
      << "second_derivative_weight should NOT be negative.";
  CHECK_GE(qp_spline_path_config_.third_derivative_weight(), 0.0)
      << "third_derivative_weight should NOT be negative.";
}

void QpSplinePathGenerator::SetDebugLogger(
    apollo::planning_internal::Debug* debug) {
  planning_debug_ = debug;
}

bool QpSplinePathGenerator::Generate(
    const std::vector<const PathObstacle*>& path_obstacles,
    const SpeedData& speed_data, const common::TrajectoryPoint& init_point,
    PathData* const path_data) {
  if (!CalculateInitFrenetPoint(init_point, &init_frenet_point_)) {
    AERROR << "Fail to map init point: " << init_point.ShortDebugString();
    return false;
  }
  double start_s = init_frenet_point_.s();
  double end_s = reference_line_.Length();

  QpFrenetFrame qp_frenet_frame(reference_line_, path_obstacles, speed_data,
                                init_frenet_point_, start_s, end_s,
                                qp_spline_path_config_.time_resolution());
  if (!qp_frenet_frame.Init(qp_spline_path_config_.num_output())) {
    AERROR << "Fail to initialize qp frenet frame";
    return false;
  }
  qp_frenet_frame.LogQpBound(planning_debug_);

  ADEBUG << "pss path start with " << start_s << ", end with " << end_s;

  if (!InitSpline(start_s, end_s)) {
    AERROR << "Init smoothing spline failed with (" << start_s << ",  end_s "
           << end_s;
    return false;
  }

  if (!AddConstraint(qp_frenet_frame)) {
    AERROR << "Fail to setup pss path constraint.";
    return false;
  }

  AddKernel();

  if (!Solve()) {
    AERROR << "Fail to solve the qp problem.";
    return false;
  }

  ADEBUG << common::util::StrCat("Spline dl:", init_frenet_point_.dl(),
                                 ", ddl:", init_frenet_point_.ddl());

  // extract data
  const Spline1d& spline = spline_generator_->spline();
  std::vector<common::PathPoint> path_points;

  double start_l = spline(init_frenet_point_.s());
  ReferencePoint ref_point =
      reference_line_.GetReferencePoint(init_frenet_point_.s());
  Vec2d xy_point = CartesianFrenetConverter::CalculateCartesianPoint(
      ref_point.heading(), Vec2d(ref_point.x(), ref_point.y()), start_l);

  double x_diff = xy_point.x() - init_point.path_point().x();
  double y_diff = xy_point.y() - init_point.path_point().y();

  double s = init_frenet_point_.s();
  double s_resolution =
      (end_s - init_frenet_point_.s()) / qp_spline_path_config_.num_output();
  while (Double::Compare(s, end_s) < 0) {
    double l = spline(s);
    if (planning_debug_ &&
        planning_debug_->planning_data().sl_frame().size() >= 1) {
      auto sl_point = planning_debug_->mutable_planning_data()
                          ->mutable_sl_frame(0)
                          ->mutable_sl_path()
                          ->Add();
      sl_point->set_l(l);
      sl_point->set_s(s);
    }
    double dl = spline.Derivative(s);
    double ddl = spline.SecondOrderDerivative(s);
    ReferencePoint ref_point = reference_line_.GetReferencePoint(s);
    Vec2d curr_xy_point = CartesianFrenetConverter::CalculateCartesianPoint(
        ref_point.heading(), Vec2d(ref_point.x(), ref_point.y()), l);
    curr_xy_point.set_x(curr_xy_point.x() - x_diff);
    curr_xy_point.set_y(curr_xy_point.y() - y_diff);
    double theta = CartesianFrenetConverter::CalculateTheta(
        ref_point.heading(), ref_point.kappa(), l, dl);
    double kappa = CartesianFrenetConverter::CalculateKappa(
        ref_point.kappa(), ref_point.dkappa(), l, dl, ddl);
    common::PathPoint path_point = common::util::MakePathPoint(
        curr_xy_point.x(), curr_xy_point.y(), 0.0, theta, kappa, 0.0, 0.0);
    if (path_points.size() != 0) {
      double distance =
          common::util::Distance2D(path_points.back(), path_point);
      path_point.set_s(path_points.back().s() + distance);
    }
    if (Double::Compare(path_point.s(), end_s) >= 0) {
      break;
    }
    path_points.push_back(std::move(path_point));
    s += s_resolution;
  }
  path_data->SetReferenceLine(&reference_line_);
  path_data->SetDiscretizedPath(DiscretizedPath(path_points));
  return true;
}

bool QpSplinePathGenerator::CalculateInitFrenetPoint(
    const common::TrajectoryPoint& traj_point,
    common::FrenetFramePoint* const frenet_frame_point) {
  common::SLPoint sl_point;
  if (!reference_line_.XYToSL(
          {traj_point.path_point().x(), traj_point.path_point().y()},
          &sl_point)) {
    return false;
  }
  frenet_frame_point->set_s(sl_point.s());
  frenet_frame_point->set_l(sl_point.l());

  const double theta = traj_point.path_point().theta();
  const double kappa = traj_point.path_point().kappa();
  const double l = frenet_frame_point->l();

  ReferencePoint ref_point;
  ref_point = reference_line_.GetReferencePoint(frenet_frame_point->s());

  const double theta_ref = ref_point.heading();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();

  const double dl = CartesianFrenetConverter::CalculateLateralDerivative(
      theta_ref, theta, l, kappa_ref);
  const double ddl =
      CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point->set_dl(dl);
  frenet_frame_point->set_ddl(ddl);
  return true;
}

bool QpSplinePathGenerator::InitSpline(const double start_s,
                                       const double end_s) {
  // set knots
  if (qp_spline_path_config_.number_of_knots() <= 1) {
    AERROR << "Too few number of knots: "
           << qp_spline_path_config_.number_of_knots();
    return false;
  }
  const double delta_s =
      (end_s - start_s) / qp_spline_path_config_.number_of_knots();
  double curr_knot_s = start_s;

  for (uint32_t i = 0; i <= qp_spline_path_config_.number_of_knots();
       ++i, curr_knot_s = std::min(curr_knot_s + delta_s, end_s)) {
    knots_.push_back(curr_knot_s);
  }

  // spawn a new spline generator
  spline_generator_.reset(
      new Spline1dGenerator(knots_, qp_spline_path_config_.spline_order()));

  // set evaluated_s_
  std::uint32_t num_evaluated_s =
      qp_spline_path_config_.number_of_fx_constraint_knots();
  if (num_evaluated_s <= 2) {
    AERROR << "Too few evaluated positions. Suggest: > 2, current number: "
           << num_evaluated_s;
    return false;
  }
  const auto& x_knots = spline_generator_->spline().x_knots();
  const double back_s = x_knots.back();
  const double front_s = x_knots.front();
  const double ds = (back_s - front_s) / num_evaluated_s;
  double curr_evaluated_s = front_s;
  for (uint32_t i = 0; i < num_evaluated_s;
       ++i, curr_evaluated_s = std::min(curr_evaluated_s + ds, back_s)) {
    evaluated_s_.push_back(curr_evaluated_s);
  }

  return true;
}

bool QpSplinePathGenerator::AddConstraint(
    const QpFrenetFrame& qp_frenet_frame) {
  Spline1dConstraint* spline_constraint =
      spline_generator_->mutable_spline_constraint();

  // add init status constraint, equality constraint
  spline_constraint->AddPointConstraint(init_frenet_point_.s(),
                                        init_frenet_point_.l());
  spline_constraint->AddPointDerivativeConstraint(init_frenet_point_.s(),
                                                  init_frenet_point_.dl());
  spline_constraint->AddPointSecondDerivativeConstraint(
      init_frenet_point_.s(), init_frenet_point_.ddl());

  ADEBUG << "init frenet point: " << init_frenet_point_.ShortDebugString();

  // add end point constraint, equality constraint
  spline_constraint->AddPointConstraint(knots_.back(), 0.0);

  spline_constraint->AddPointDerivativeConstraint(knots_.back(), 0.0);

  spline_constraint->AddPointSecondDerivativeConstraint(knots_.back(), 0.0);

  // kappa bound is based on the inequality:
  // kappa = d(phi)/ds <= d(phi)/dx = d2y/dx2
  std::vector<double> kappa_lower_bound(evaluated_s_.size(),
                                        -FLAGS_kappa_bound);
  std::vector<double> kappa_upper_bound(evaluated_s_.size(), FLAGS_kappa_bound);
  if (!spline_constraint->AddSecondDerivativeBoundary(
          evaluated_s_, kappa_lower_bound, kappa_upper_bound)) {
    AERROR << "Fail to add second derivative boundary.";
    return false;
  }

  // add map bound constraint
  const auto lateral_buf = qp_spline_path_config_.cross_lane_extension_buffer();
  std::vector<double> boundary_low;
  std::vector<double> boundary_high;
  for (const double s : evaluated_s_) {
    std::pair<double, double> road_boundary(0.0, 0.0);
    std::pair<double, double> static_obs_boundary(0.0, 0.0);
    std::pair<double, double> dynamic_obs_boundary(0.0, 0.0);

    qp_frenet_frame.GetMapBound(s, &road_boundary);
    qp_frenet_frame.GetStaticObstacleBound(s, &static_obs_boundary);
    qp_frenet_frame.GetDynamicObstacleBound(s, &dynamic_obs_boundary);

    road_boundary.first =
        std::fmin(road_boundary.first, init_frenet_point_.l() - lateral_buf);
    road_boundary.second =
        std::fmax(road_boundary.second, init_frenet_point_.l() + lateral_buf);

    boundary_low.emplace_back(common::util::MaxElement(
        std::vector<double>{road_boundary.first, static_obs_boundary.first,
                            dynamic_obs_boundary.first}));
    boundary_high.emplace_back(common::util::MinElement(
        std::vector<double>{road_boundary.second, static_obs_boundary.second,
                            dynamic_obs_boundary.second}));

    ADEBUG << "s:" << s << " boundary_low:" << boundary_low.back()
           << " boundary_high:" << boundary_high.back()
           << " road_boundary_low: " << road_boundary.first
           << " road_boundary_high: " << road_boundary.second
           << " static_obs_boundary_low: " << static_obs_boundary.first
           << " static_obs_boundary_high: " << static_obs_boundary.second
           << " dynamic_obs_boundary_low: " << dynamic_obs_boundary.first
           << " dynamic_obs_boundary_high: " << dynamic_obs_boundary.second;
  }

  if (planning_debug_) {
    apollo::planning_internal::SLFrameDebug* sl_frame =
        planning_debug_->mutable_planning_data()->mutable_sl_frame()->Add();
    for (size_t i = 0; i < evaluated_s_.size(); ++i) {
      sl_frame->mutable_aggregated_boundary_s()->Add(evaluated_s_[i]);
      sl_frame->mutable_aggregated_boundary_low()->Add(boundary_low[i]);
      sl_frame->mutable_aggregated_boundary_high()->Add(boundary_high[i]);
    }
  }

  if (!spline_constraint->AddBoundary(evaluated_s_, boundary_low,
                                      boundary_high)) {
    AERROR << "Add boundary constraint failed";
    return false;
  }

  // add spline joint third derivative constraint
  if (!spline_constraint->AddThirdDerivativeSmoothConstraint()) {
    AERROR << "Add spline joint third derivative constraint failed!";
    return false;
  }
  return true;
}

void QpSplinePathGenerator::AddKernel() {
  Spline1dKernel* spline_kernel = spline_generator_->mutable_spline_kernel();

  if (qp_spline_path_config_.regularization_weight() > 0.0) {
    spline_kernel->AddRegularization(
        qp_spline_path_config_.regularization_weight());
  }

  if (qp_spline_path_config_.derivative_weight() > 0.0) {
    spline_kernel->AddDerivativeKernelMatrix(
        qp_spline_path_config_.derivative_weight());
  }

  if (qp_spline_path_config_.second_derivative_weight() > 0.0) {
    spline_kernel->AddSecondOrderDerivativeMatrix(
        qp_spline_path_config_.second_derivative_weight());
  }

  if (qp_spline_path_config_.third_derivative_weight() > 0.0) {
    spline_kernel->AddThirdOrderDerivativeMatrix(
        qp_spline_path_config_.third_derivative_weight());
  }
}

bool QpSplinePathGenerator::Solve() {
  if (!spline_generator_->Solve()) {
    for (size_t i = 0; i < knots_.size(); ++i) {
      AERROR << "knots_[" << i << "]: " << knots_[i];
    }
    for (size_t i = 0; i < evaluated_s_.size(); ++i) {
      AERROR << "evaluated_s_[" << i << "]: " << evaluated_s_[i];
    }
    AERROR << "Could not solve the qp problem in spline path generator.";
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
