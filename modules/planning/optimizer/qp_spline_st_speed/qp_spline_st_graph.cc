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

#include "modules/planning/optimizer/qp_spline_st_speed/qp_spline_st_graph.h"

#include <algorithm>
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
    const apollo::common::VehicleParam& veh_param)
    : qp_spline_st_speed_config_(qp_spline_st_speed_config),
      time_resolution_(
          qp_spline_st_speed_config_.total_time() /
          qp_spline_st_speed_config_.number_of_discrete_graph_t()) {}

void QpSplineStGraph::Init() {
  double curr_t = 0.0;
  for (uint32_t i = 0;
       i <= qp_spline_st_speed_config_.number_of_discrete_graph_t(); ++i) {
    t_knots_.push_back(curr_t);
    curr_t += time_resolution_;
  }
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

  // TODO(all): update speed limit here
  // TODO(all): update config through veh physical limit here generate knots

  // initialize time resolution and
  Init();

  if (!ApplyConstraint(st_graph_data.obs_boundary()).ok()) {
    const std::string msg = "Apply constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!ApplyKernel(st_graph_data.speed_limit()).ok()) {
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

  double time_resolution = qp_spline_st_speed_config_.output_time_resolution();
  double time = 0.0;
  while (time < qp_spline_st_speed_config_.total_time() + time_resolution) {
    double s = spline(time);
    double v = spline.derivative(time);
    double a = spline.second_order_derivative(time);
    double da = spline.third_order_derivative(time);
    speed_data->add_speed_point(s, time, v, a, da);
    time += time_resolution;
  }

  return Status::OK();
}

Status QpSplineStGraph::ApplyConstraint(
    const std::vector<StGraphBoundary>& boundaries) {
  Spline1dConstraint* constraint =
      spline_generator_->mutable_spline_constraint();
  // position, velocity, acceleration

  if (!constraint->add_point_fx_constraint(0.0, 0.0)) {
    const std::string msg = "add st start point constraint failed";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!constraint->add_point_derivative_constraint(0.0, init_point_.v())) {
    const std::string msg = "add st start point velocity constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!constraint->add_point_second_derivative_constraint(0.0,
                                                          init_point_.a())) {
    const std::string msg =
        "add st start point acceleration constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!constraint->add_point_second_derivative_constraint(
          spline_generator_->spline().x_knots().back(), 0.0)) {
    const std::string msg = "add st end point acceleration constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // monotone constraint
  if (!constraint->add_monotone_fx_inequality_constraint_at_knots()) {
    const std::string msg = "add monotonicity constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // smoothness constraint
  if (!constraint->add_third_derivative_smooth_constraint()) {
    const std::string msg = "add smoothness joint constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // boundary constraint
  std::vector<double> s_upper_bound;
  std::vector<double> s_lower_bound;

  for (const double curr_t : t_knots_) {
    double lower_s = 0.0;
    double upper_s = 0.0;
    GetSConstraintByTime(boundaries, curr_t,
                         qp_spline_st_speed_config_.total_path_length(),
                         &upper_s, &lower_s);
    s_upper_bound.push_back(upper_s);
    s_lower_bound.push_back(lower_s);
  }
  if (!constraint->add_fx_boundary(t_knots_, s_lower_bound, s_upper_bound)) {
    const std::string msg = "Fail to apply obstacle constraint";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // TODO(Liangliang):
  // add speed constraint and other limits according to adu/planning
  return Status::OK();
}

Status QpSplineStGraph::ApplyKernel(const SpeedLimit& speed_limit) {
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

  // TODO(all): add reference speed profile for different main decision
  std::vector<double> s_vec;
  if (speed_limit.speed_limits().size() == 0) {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to apply_kernel due to empty speed limits.");
  }
  double dist_ref = 0.0;
  for (uint32_t i = 0;
       i <= qp_spline_st_speed_config_.number_of_discrete_graph_t(); ++i) {
    s_vec.push_back(dist_ref);
    dist_ref += time_resolution_ * speed_limit.get_speed_limit(dist_ref);
  }
  // TODO: change reference line kernel to configurable version
  spline_kernel->add_reference_line_kernel_matrix(t_knots_, s_vec, 1);
  return Status::OK();
}

Status QpSplineStGraph::Solve() {
  return spline_generator_->solve()
             ? Status::OK()
             : Status(ErrorCode::PLANNING_ERROR, "QpSplineStGraph::solve");
}

Status QpSplineStGraph::AddFollowReferenceLineKernel(
    const StGraphBoundary& follow_boundary) {
  if (follow_boundary.boundary_type() !=
      StGraphBoundary::BoundaryType::FOLLOW) {
    std::string msg = common::util::StrCat(
        "Fail to add follow reference line kernel because boundary type is ",
        "incorrect. type: ", static_cast<int>(follow_boundary.boundary_type()),
        ".");
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  auto* spline_kernel = spline_generator_->mutable_spline_kernel();
  std::vector<double> ref_s;
  for (const double curr_t : t_knots_) {
    double s_upper = 0.0;
    double s_lower = 0.0;
    if (follow_boundary.get_s_boundary_position(curr_t, &s_upper, &s_lower)) {
      ref_s.push_back(s_upper - follow_boundary.characteristic_length());
    }
    ref_s.push_back(s_upper);
  }

  spline_kernel->add_reference_line_kernel_matrix(t_knots_, ref_s, 1.0);
  return Status::OK();
}

Status QpSplineStGraph::GetSConstraintByTime(
    const std::vector<StGraphBoundary>& boundaries, const double time,
    const double total_path_s, double* const s_upper_bound,
    double* const s_lower_bound) const {
  *s_upper_bound =
      std::min(total_path_s, time * qp_spline_st_speed_config_.max_speed());

  for (const StGraphBoundary& boundary : boundaries) {
    double s_upper = 0.0;
    double s_lower = 0.0;

    if (!boundary.get_s_boundary_position(time, &s_upper, &s_lower)) {
      continue;
    }

    if (boundary.boundary_type() == StGraphBoundary::BoundaryType::STOP ||
        boundary.boundary_type() == StGraphBoundary::BoundaryType::FOLLOW ||
        boundary.boundary_type() == StGraphBoundary::BoundaryType::YIELD) {
      *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
    } else {
      *s_lower_bound = std::fmax(*s_lower_bound, s_lower);
    }
  }

  return Status::OK();
}
}  // namespace planning
}  // namespace apollo
