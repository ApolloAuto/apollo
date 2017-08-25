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
 * @file qp_spline_st_graph.cc
 **/

#include "modules/planning/tasks/qp_spline_st_speed/qp_spline_st_graph.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "modules/common/log.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleParam;

QpSplineStGraph::QpSplineStGraph(
    const QpSplineStSpeedConfig& qp_spline_st_speed_config,
    const VehicleParam& veh_param)
    : qp_spline_st_speed_config_(qp_spline_st_speed_config),
      t_knots_resolution_(
          qp_spline_st_speed_config_.total_time() /
          qp_spline_st_speed_config_.number_of_discrete_graph_t()),
      t_evaluated_resolution_(
          qp_spline_st_speed_config_.total_time() /
          qp_spline_st_speed_config_.number_of_evaluated_graph_t()) {}

void QpSplineStGraph::Init() {
  // init knots
  double curr_t = 0.0;
  for (uint32_t i = 0;
       i <= qp_spline_st_speed_config_.number_of_discrete_graph_t(); ++i) {
    t_knots_.push_back(curr_t);
    curr_t += t_knots_resolution_;
  }

  // init evaluated t positions
  curr_t = 0.0;
  for (uint32_t i = 0;
       i <= qp_spline_st_speed_config_.number_of_evaluated_graph_t(); ++i) {
    t_evaluated_.push_back(curr_t);
    curr_t += t_evaluated_resolution_;
  }

  // init spline generator
  spline_generator_.reset(new Spline1dGenerator(
      t_knots_, qp_spline_st_speed_config_.spline_order()));
}

Status QpSplineStGraph::Search(const StGraphData& st_graph_data,
                               const PathData& path_data,
                               SpeedData* const speed_data) {
  init_point_ = st_graph_data.init_point();
  if (st_graph_data.path_data_length() <
      qp_spline_st_speed_config_.total_path_length()) {
    qp_spline_st_speed_config_.set_total_path_length(
        st_graph_data.path_data_length());
  }

  // TODO(all): update config through veh physical limit here generate knots

  // initialize time resolution and
  Init();

  if (!ApplyConstraint(st_graph_data.init_point(), st_graph_data.speed_limit(),
                       st_graph_data.st_boundaries())
           .ok()) {
    const std::string msg = "Apply constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!ApplyKernel(st_graph_data.st_boundaries(),
                   st_graph_data.speed_limit())
           .ok()) {
    const std::string msg = "Apply kernel failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!Solve().ok()) {
    const std::string msg = "Solve qp problem failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // extract output
  speed_data->Clear();
  const Spline1d& spline = spline_generator_->spline();

  double t_output_resolution =
      qp_spline_st_speed_config_.output_time_resolution();
  double time = 0.0;
  while (time < qp_spline_st_speed_config_.total_time() + t_output_resolution) {
    double s = spline(time);
    double v = spline.Derivative(time);
    double a = spline.SecondOrderDerivative(time);
    double da = spline.ThirdOrderDerivative(time);
    speed_data->AppendSpeedPoint(s, time, v, a, da);
    time += t_output_resolution;
  }

  return Status::OK();
}

Status QpSplineStGraph::ApplyConstraint(
    const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
    const std::vector<StBoundary>& boundaries) {
  Spline1dConstraint* constraint =
      spline_generator_->mutable_spline_constraint();
  // position, velocity, acceleration

  if (!constraint->AddPointConstraint(0.0, 0.0)) {
    const std::string msg = "add st start point constraint failed";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  ADEBUG << "init point constraint:" << init_point.DebugString();
  if (!constraint->AddPointDerivativeConstraint(0.0, init_point_.v())) {
    const std::string msg = "add st start point velocity constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!constraint->AddPointSecondDerivativeConstraint(0.0, init_point_.a())) {
    const std::string msg =
        "add st start point acceleration constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!constraint->AddPointSecondDerivativeConstraint(
          spline_generator_->spline().x_knots().back(), 0.0)) {
    const std::string msg = "add st end point acceleration constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // monotone constraint
  if (!constraint->AddMonotoneInequalityConstraintAtKnots()) {
    const std::string msg = "add monotone inequality constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // smoothness constraint
  if (!constraint->AddThirdDerivativeSmoothConstraint()) {
    const std::string msg = "add smoothness joint constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // boundary constraint
  std::vector<double> s_upper_bound;
  std::vector<double> s_lower_bound;

  for (const double curr_t : t_evaluated_) {
    double lower_s = 0.0;
    double upper_s = 0.0;
    GetSConstraintByTime(boundaries, curr_t,
                         qp_spline_st_speed_config_.total_path_length(),
                         &upper_s, &lower_s);
    s_upper_bound.push_back(upper_s);
    s_lower_bound.push_back(lower_s);
    ADEBUG << "Add constraint by time: " << curr_t << " upper_s: " << upper_s
           << " lower_s: " << lower_s;
  }

  DCHECK_EQ(t_evaluated_.size(), s_lower_bound.size());
  DCHECK_EQ(t_evaluated_.size(), s_upper_bound.size());
  if (!constraint->AddBoundary(t_evaluated_, s_lower_bound, s_upper_bound)) {
    const std::string msg = "Fail to apply distance constraints.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<double> speed_upper_bound;
  if (!EstimateSpeedUpperBound(init_point, speed_limit, &speed_upper_bound)
           .ok()) {
    std::string msg = "Fail to estimate speed upper constraints.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<double> speed_lower_bound(t_evaluated_.size(), 0.0);

  DCHECK_EQ(t_evaluated_.size(), speed_upper_bound.size());
  DCHECK_EQ(t_evaluated_.size(), speed_lower_bound.size());
  if (!constraint->AddDerivativeBoundary(t_evaluated_, speed_lower_bound,
                                         speed_upper_bound)) {
    const std::string msg = "Fail to apply speed constraints.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  for (size_t i = 0; i < t_evaluated_.size(); ++i) {
    ADEBUG << "t_evaluated_: " << t_evaluated_[i]
           << "speed_lower_bound: " << speed_lower_bound[i]
           << "speed_upper_bound: " << speed_upper_bound[i] << std::endl;
  }

  // TODO : add acceleration constraint here
  return Status::OK();
}

Status QpSplineStGraph::ApplyKernel(
    const std::vector<StBoundary>& boundaries,
    const SpeedLimit& speed_limit) {
  Spline1dKernel* spline_kernel = spline_generator_->mutable_spline_kernel();

  if (qp_spline_st_speed_config_.speed_kernel_weight() > 0) {
    spline_kernel->add_derivative_kernel_matrix(
        qp_spline_st_speed_config_.speed_kernel_weight());
  }

  if (qp_spline_st_speed_config_.accel_kernel_weight() > 0) {
    spline_kernel->add_second_order_derivative_matrix(
        qp_spline_st_speed_config_.accel_kernel_weight());
  }

  if (qp_spline_st_speed_config_.jerk_kernel_weight() > 0) {
    spline_kernel->add_third_order_derivative_matrix(
        qp_spline_st_speed_config_.jerk_kernel_weight());
  }

  if (AddCruiseReferenceLineKernel(
          t_evaluated_, speed_limit,
          qp_spline_st_speed_config_.cruise_weight()) != Status::OK()) {
    return Status(ErrorCode::PLANNING_ERROR, "QpSplineStGraph::ApplyKernel");
  }

  if (AddFollowReferenceLineKernel(
          t_evaluated_, boundaries,
          qp_spline_st_speed_config_.follow_weight()) != Status::OK()) {
    return Status(ErrorCode::PLANNING_ERROR, "QpSplineStGraph::ApplyKernel");
  }
  return Status::OK();
}

Status QpSplineStGraph::Solve() {
  return spline_generator_->Solve()
             ? Status::OK()
             : Status(ErrorCode::PLANNING_ERROR, "QpSplineStGraph::solve");
}

Status QpSplineStGraph::AddCruiseReferenceLineKernel(
    const std::vector<double>& evaluate_t, const SpeedLimit& speed_limit,
    const double weight) {
  auto* spline_kernel = spline_generator_->mutable_spline_kernel();
  std::vector<double> s_vec;
  if (speed_limit.speed_limit_points().size() == 0) {
    std::string msg = "Fail to apply_kernel due to empty speed limits.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  double dist_ref = 0.0;
  s_vec.push_back(dist_ref);
  for (uint32_t i = 1; i < evaluate_t.size(); ++i) {
    dist_ref += (evaluate_t[i] - evaluate_t[i - 1]) *
                speed_limit.GetSpeedLimitByS(dist_ref);
    s_vec.push_back(dist_ref);
  }
  DCHECK_EQ(evaluate_t.size(), s_vec.size());

  for (std::size_t i = 0; i < evaluate_t.size(); ++i) {
    ADEBUG << "Cruise Ref S: " << s_vec[i]
           << " Relative time: " << evaluate_t[i] << std::endl;
  }

  if (evaluate_t.size() > 0) {
    spline_kernel->add_reference_line_kernel_matrix(
        evaluate_t, s_vec,
        weight * qp_spline_st_speed_config_.total_time() / evaluate_t.size());
  }
  spline_kernel->AddRegularization(0.01);
  return Status::OK();
}

Status QpSplineStGraph::AddFollowReferenceLineKernel(
    const std::vector<double>& evaluate_t,
    const std::vector<StBoundary>& boundaries, const double weight) {
  auto* spline_kernel = spline_generator_->mutable_spline_kernel();
  std::vector<double> ref_s;
  std::vector<double> filtered_evaluate_t;
  for (const double curr_t : evaluate_t) {
    double s_min = std::numeric_limits<double>::infinity();
    bool success = false;
    for (const auto& boundary : boundaries) {
      if (boundary.boundary_type() != StBoundary::BoundaryType::FOLLOW) {
        continue;
      }
      double s_upper = 0.0;
      double s_lower = 0.0;
      if (boundary.GetUnblockSRange(curr_t, &s_upper, &s_lower)) {
        success = true;
        s_min = std::min(s_min, s_upper - boundary.characteristic_length());
      }
    }
    if (success) {
      filtered_evaluate_t.push_back(curr_t);
      ref_s.push_back(s_min);
    }
  }
  DCHECK_EQ(filtered_evaluate_t.size(), ref_s.size());

  if (!ref_s.empty()) {
    spline_kernel->add_reference_line_kernel_matrix(
        filtered_evaluate_t, ref_s,
        weight * qp_spline_st_speed_config_.total_time() / evaluate_t.size());
  }

  for (std::size_t i = 0; i < filtered_evaluate_t.size(); ++i) {
    ADEBUG << "Follow Ref S: " << ref_s[i]
           << " Relative time: " << filtered_evaluate_t[i] << std::endl;
  }
  return Status::OK();
}

Status QpSplineStGraph::GetSConstraintByTime(
    const std::vector<StBoundary>& boundaries, const double time,
    const double total_path_s, double* const s_upper_bound,
    double* const s_lower_bound) const {
  *s_upper_bound = total_path_s;

  for (const StBoundary& boundary : boundaries) {
    double s_upper = 0.0;
    double s_lower = 0.0;

    if (!boundary.GetUnblockSRange(time, &s_upper, &s_lower)) {
      continue;
    }

    if (boundary.boundary_type() == StBoundary::BoundaryType::STOP ||
        boundary.boundary_type() == StBoundary::BoundaryType::FOLLOW ||
        boundary.boundary_type() == StBoundary::BoundaryType::YIELD) {
      *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
    } else {
      DCHECK(boundary.boundary_type() ==
             StBoundary::BoundaryType::OVERTAKE);
      *s_lower_bound = std::fmax(*s_lower_bound, s_lower);
    }
  }

  return Status::OK();
}

Status QpSplineStGraph::EstimateSpeedUpperBound(
    const common::TrajectoryPoint& init_point, const SpeedLimit& speed_limit,
    std::vector<double>* speed_upper_bound) const {
  DCHECK_NOTNULL(speed_upper_bound);
  speed_upper_bound->clear();

  // use v to estimate position: not accurate, but feasible in cyclic
  // processing. We can do the following process multiple times and use
  // previous cycle's results for better estimation.
  const double v = init_point.v();

  if (static_cast<double>(t_evaluated_.size() +
                          speed_limit.speed_limit_points().size()) <
      t_evaluated_.size() * std::log(static_cast<double>(
                                speed_limit.speed_limit_points().size()))) {
    uint32_t i = 0;
    uint32_t j = 0;
    const double kDistanceEpsilon = 1e-6;
    while (i < t_evaluated_.size() &&
           j + 1 < speed_limit.speed_limit_points().size()) {
      const double distance = v * t_evaluated_[i];
      if (fabs(distance - speed_limit.speed_limit_points()[j].first) <
          kDistanceEpsilon) {
        speed_upper_bound->push_back(
            speed_limit.speed_limit_points()[j].second);
        ++i;
        ADEBUG << "speed upper bound:" << speed_upper_bound->back();
      } else if (distance < speed_limit.speed_limit_points()[j].first) {
        ++i;
      } else if (distance <= speed_limit.speed_limit_points()[j + 1].first) {
        speed_upper_bound->push_back(speed_limit.GetSpeedLimitByS(distance));
        ADEBUG << "speed upper bound:" << speed_upper_bound->back();
        ++i;
      } else {
        ++j;
      }
    }

    for (uint32_t k = speed_upper_bound->size(); k < t_evaluated_.size(); ++k) {
      speed_upper_bound->push_back(qp_spline_st_speed_config_.max_speed());
      ADEBUG << "speed upper bound:" << speed_upper_bound->back();
    }
  } else {
    auto cmp = [](const std::pair<double, double>& p1, const double s) {
      return p1.first < s;
    };

    for (const double t : t_evaluated_) {
      const double s = v * t;

      // NOTICE: we are using binary search here based on two assumptions:
      // (1) The s in speed_limit_points increase monotonically.
      // (2) The evaluated_t_.size() << number of speed_limit_points.size()
      //
      // If either of the two assumption is failed, a new algorithm must be used
      // to replace the binary search.
      const auto& it =
          std::lower_bound(speed_limit.speed_limit_points().begin(),
                           speed_limit.speed_limit_points().end(), s, cmp);
      speed_upper_bound->push_back(it->second);
    }
  }

  const double kTimeBuffer = 2.0;
  for (uint32_t k = 0; k < t_evaluated_.size() && t_evaluated_[k] < kTimeBuffer;
       ++k) {
    speed_upper_bound->at(k) =
        std::fmax(init_point_.v(), speed_upper_bound->at(k));
  }

  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
