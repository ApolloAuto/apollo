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
#include "modules/planning/tasks/qp_spline_path/qp_spline_path_generator.h"

#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/frame_conversion/cartesian_frenet_conversion.h"

namespace apollo {
namespace planning {

using Vec2d = apollo::common::math::Vec2d;

QpSplinePathGenerator::QpSplinePathGenerator(
    Spline1dGenerator* spline_generator, const ReferenceLine& reference_line,
    const QpSplinePathConfig& qp_spline_path_config,
    const SLBoundary& adc_sl_boundary)
    : spline_generator_(spline_generator),
      reference_line_(reference_line),
      qp_spline_path_config_(qp_spline_path_config),
      adc_sl_boundary_(adc_sl_boundary) {
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
  ADEBUG << "Init point: " << init_point.DebugString();

  const auto& path_data_history = path_data->path_data_history();
  if (!path_data_history.empty()) {
    last_discretized_path_ = &path_data_history.back().first;
  }

  if (!CalculateFrenetPoint(init_point, &init_frenet_point_)) {
    AERROR << "Fail to map init point: " << init_point.ShortDebugString();
    return false;
  }
  double start_s = init_frenet_point_.s();
  double end_s = reference_line_.Length();

  const double kMinPathLength = 1.0e-6;
  if (start_s + kMinPathLength > end_s) {
    AERROR << "Path length is too small. Path start_s: " << start_s
           << ", end_s: " << end_s;
    return false;
  } else {
    ADEBUG << "path start with " << start_s << ", end with " << end_s;
  }

  if (!InitSpline(start_s, end_s)) {
    AERROR << "Init smoothing spline failed with (" << start_s << ",  end_s "
           << end_s;
    return false;
  }

  QpFrenetFrame qp_frenet_frame(reference_line_, speed_data, init_frenet_point_,
                                qp_spline_path_config_.time_resolution(),
                                evaluated_s_);
  if (!qp_frenet_frame.Init(path_obstacles)) {
    AERROR << "Fail to initialize qp frenet frame";
    return false;
  }
  qp_frenet_frame.LogQpBound(planning_debug_);

  if (!AddConstraint(qp_frenet_frame)) {
    AERROR << "Fail to setup pss path constraint.";
    return false;
  }

  AddKernel();

  bool is_solved = Solve();

  if (!is_solved) {
    AERROR << "Fail to solve qp_spline_path. Use reference line as qp_path "
              "output.";
  }
  ADEBUG << common::util::StrCat("Spline dl:", init_frenet_point_.dl(),
                                 ", ddl:", init_frenet_point_.ddl());

  // extract data
  const Spline1d& spline = spline_generator_->spline();
  std::vector<common::PathPoint> path_points;

  double start_l = 0.0;
  if (is_solved) {
    start_l = spline(init_frenet_point_.s());
  }
  ReferencePoint ref_point =
      reference_line_.GetReferencePoint(init_frenet_point_.s());
  Vec2d xy_point = CartesianFrenetConverter::CalculateCartesianPoint(
      ref_point.heading(), Vec2d(ref_point.x(), ref_point.y()), start_l);

  const double x_diff = xy_point.x() - init_point.path_point().x();
  const double y_diff = xy_point.y() - init_point.path_point().y();

  double s = init_frenet_point_.s();
  double s_resolution =
      (end_s - init_frenet_point_.s()) / qp_spline_path_config_.num_output();
  constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  while (s + kEpsilon < end_s) {
    double l = 0.0;
    if (is_solved) {
      l = spline(s);
    }
    if (planning_debug_ &&
        planning_debug_->planning_data().sl_frame().size() >= 1) {
      auto sl_point = planning_debug_->mutable_planning_data()
                          ->mutable_sl_frame(0)
                          ->mutable_sl_path()
                          ->Add();
      sl_point->set_l(l);
      sl_point->set_s(s);
    }
    double dl = 0.0;
    double ddl = 0.0;
    if (is_solved) {
      dl = spline.Derivative(s);
      ddl = spline.SecondOrderDerivative(s);
    }
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
    if (!path_points.empty()) {
      double distance =
          common::util::DistanceXY(path_points.back(), path_point);
      path_point.set_s(path_points.back().s() + distance);
      if (distance > 1e-4) {
        path_point.set_dkappa((kappa - path_points.back().dkappa()) / distance);
      }
    }

    if (path_point.s() > end_s) {
      break;
    }
    path_points.push_back(std::move(path_point));
    s += s_resolution;
  }
  path_data->SetReferenceLine(&reference_line_);
  path_data->SetDiscretizedPath(DiscretizedPath(path_points));
  return true;
}

bool QpSplinePathGenerator::CalculateFrenetPoint(
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
  uint32_t number_of_spline = static_cast<uint32_t>(
      (end_s - start_s) / qp_spline_path_config_.max_spline_length() + 1.0);
  number_of_spline = std::max(1u, number_of_spline);
  common::util::uniform_slice(start_s, end_s, number_of_spline, &knots_);

  // spawn a new spline generator
  spline_generator_->Reset(knots_, qp_spline_path_config_.spline_order());

  // set evaluated_s_
  uint32_t constraint_num = 3 * number_of_spline + 1;
  common::util::uniform_slice(start_s, end_s, constraint_num - 1,
                              &evaluated_s_);
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
  spline_constraint->AddPointConstraint(evaluated_s_.back(), 0.0);
  spline_constraint->AddPointDerivativeConstraint(evaluated_s_.back(), 0.0);
  spline_constraint->AddPointSecondDerivativeConstraint(evaluated_s_.back(),
                                                        0.0);

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

  // dkappa = d(kappa) / ds <= d3y/dx3
  std::vector<double> dkappa_lower_bound(evaluated_s_.size(),
                                         -FLAGS_dkappa_bound);
  std::vector<double> dkappa_upper_bound(evaluated_s_.size(),
                                         FLAGS_dkappa_bound);
  if (!spline_constraint->AddThirdDerivativeBoundary(
          evaluated_s_, dkappa_lower_bound, dkappa_upper_bound)) {
    AERROR << "Fail to add third derivative boundary.";
    return false;
  }

  // add map bound constraint
  const double lateral_buf =
      qp_spline_path_config_.cross_lane_lateral_extension();
  std::vector<double> boundary_low;
  std::vector<double> boundary_high;

  for (uint32_t i = 0; i < evaluated_s_.size(); ++i) {
    auto road_boundary = qp_frenet_frame.GetMapBound().at(i);
    auto static_obs_boundary = qp_frenet_frame.GetStaticObstacleBound().at(i);
    auto dynamic_obs_boundary = qp_frenet_frame.GetDynamicObstacleBound().at(i);

    if (evaluated_s_.at(i) - evaluated_s_.at(0) <
        qp_spline_path_config_.cross_lane_longitudinal_extension()) {
      road_boundary.first = std::fmin(road_boundary.first,
                                      adc_sl_boundary_.start_l() - lateral_buf);
      road_boundary.second = std::fmax(road_boundary.second,
                                       adc_sl_boundary_.end_l() + lateral_buf);
    }

    boundary_low.emplace_back(common::util::MaxElement(
        std::vector<double>{road_boundary.first, static_obs_boundary.first,
                            dynamic_obs_boundary.first}));
    boundary_high.emplace_back(common::util::MinElement(
        std::vector<double>{road_boundary.second, static_obs_boundary.second,
                            dynamic_obs_boundary.second}));
    ADEBUG << "s:" << evaluated_s_[i] << " boundary_low:" << boundary_low.back()
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
  if (knots_.size() >= 3 &&
      !spline_constraint->AddThirdDerivativeSmoothConstraint()) {
    AERROR << "Add spline joint derivative constraint failed!";
    return false;
  }
  return true;
}

void QpSplinePathGenerator::AddHistoryPathKernel() {
  if (last_discretized_path_ == nullptr) {
    return;
  }

  PathData last_path_data;
  last_path_data.SetReferenceLine(&reference_line_);
  last_path_data.SetDiscretizedPath(*last_discretized_path_);

  std::vector<double> history_s;
  std::vector<double> histroy_l;

  for (uint32_t i = 0; i < last_path_data.frenet_frame_path().NumOfPoints();
       ++i) {
    const auto p = last_path_data.frenet_frame_path().PointAt(i);
    history_s.push_back(p.s());
    histroy_l.push_back(p.l());
  }

  Spline1dKernel* spline_kernel = spline_generator_->mutable_spline_kernel();
  spline_kernel->AddReferenceLineKernelMatrix(
      history_s, histroy_l, qp_spline_path_config_.history_path_weight());
}

void QpSplinePathGenerator::AddKernel() {
  Spline1dKernel* spline_kernel = spline_generator_->mutable_spline_kernel();

  if (qp_spline_path_config_.reference_line_weight() > 0.0) {
    std::vector<double> ref_l(evaluated_s_.size(), 0.0);

    DCHECK_EQ(evaluated_s_.size(), ref_l.size());
    spline_kernel->AddReferenceLineKernelMatrix(
        evaluated_s_, ref_l, qp_spline_path_config_.reference_line_weight());
  }

  if (qp_spline_path_config_.history_path_weight() > 0.0) {
    AddHistoryPathKernel();
  }

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
