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
#include "modules/planning/common/planning_util.h"
namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleParam;

QpSplineStGraph::QpSplineStGraph(
    const QpSplineStSpeedConfig& qp_spline_st_speed_config,
    const apollo::common::VehicleParam& veh_param)
    : _qp_spline_st_speed_config(qp_spline_st_speed_config) {}

Status QpSplineStGraph::search(const StGraphData& st_graph_data,
                               const PathData& path_data,
                               SpeedData* const speed_data) {
  _init_point = st_graph_data.init_point();
  if (st_graph_data.path_data_length() <
      _qp_spline_st_speed_config.total_path_length()) {
    _qp_spline_st_speed_config.set_total_path_length(
        st_graph_data.path_data_length());
  }

  // TODO(all): update speed limit here
  // TODO(all): update config through veh physical limit here generate knots
  std::vector<double> t_knots;
  double t_resolution = _qp_spline_st_speed_config.total_time() /
                        _qp_spline_st_speed_config.number_of_discrete_graph_t();
  for (int32_t i = 0;
       i <= _qp_spline_st_speed_config.number_of_discrete_graph_t(); ++i) {
    t_knots.push_back(i * t_resolution);
  }

  _spline_generator.reset(new Spline1dGenerator(
      t_knots, _qp_spline_st_speed_config.spline_order()));

  if (!apply_constraint(st_graph_data.obs_boundary()).ok()) {
    const std::string msg = "Apply constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!apply_kernel(st_graph_data.speed_limit()).ok()) {
    const std::string msg = "Apply kernel failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!solve().ok()) {
    const std::string msg = "Solve qp problem failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // extract output
  const Spline1d& spline = _spline_generator->spline();

  double time_resolution = _qp_spline_st_speed_config.output_time_resolution();
  double time = 0.0;
  for (; time < _qp_spline_st_speed_config.total_time() + time_resolution;
       time += time_resolution) {
    double s = spline(time);
    double v = spline.derivative(time);
    double a = spline.second_order_derivative(time);
    double da = spline.third_order_derivative(time);
    speed_data->add_speed_point(s, time, v, a, da);
  }

  return Status::OK();
}

Status QpSplineStGraph::apply_constraint(
    const std::vector<StGraphBoundary>& boundaries) {
  Spline1dConstraint* constraint =
      _spline_generator->mutable_spline_constraint();
  // position, velocity, acceleration
  if (!constraint->add_point_fx_constraint(0.0, 0.0)) {
    const std::string msg = "add st start point constraint failed";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!constraint->add_point_derivative_constraint(0.0, _init_point.v())) {
    const std::string msg = "add st start point velocity constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!constraint->add_point_second_derivative_constraint(0.0,
                                                          _init_point.a())) {
    const std::string msg =
        "add st start point acceleration constraint failed!";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!constraint->add_point_second_derivative_constraint(
          _spline_generator->spline().x_knots().back(), 0.0)) {
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
  double evaluated_resolution =
      _qp_spline_st_speed_config.total_time() /
      _qp_spline_st_speed_config.number_of_evaluated_graph_t();

  std::vector<double> evaluated_knots;
  std::vector<double> s_upper_bound;
  std::vector<double> s_lower_bound;
  for (int32_t i = 1;
       i <= _qp_spline_st_speed_config.number_of_evaluated_graph_t(); ++i) {
    evaluated_knots.push_back(i * evaluated_resolution);
    double lower_s = 0.0;
    double upper_s = 0.0;
    get_s_constraints_by_time(boundaries, evaluated_knots.back(),
                              _qp_spline_st_speed_config.total_path_length(),
                              &upper_s, &lower_s);
    s_upper_bound.push_back(std::move(upper_s));
    s_lower_bound.push_back(std::move(lower_s));
  }
  if (!constraint->add_fx_boundary(evaluated_knots, s_lower_bound,
                                   s_upper_bound)) {
    const std::string msg = "Fail to apply obstacle constraint";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // TODO(all): add speed constraint
  return Status::OK();
}

Status QpSplineStGraph::apply_kernel(const SpeedLimit& speed_limit) {
  Spline1dKernel* spline_kernel = _spline_generator->mutable_spline_kernel();

  if (_qp_spline_st_speed_config.speed_kernel_weight() > 0) {
    spline_kernel->add_derivative_kernel_matrix(
        _qp_spline_st_speed_config.speed_kernel_weight());
  }

  if (_qp_spline_st_speed_config.accel_kernel_weight() > 0) {
    spline_kernel->add_second_order_derivative_matrix(
        _qp_spline_st_speed_config.accel_kernel_weight());
  }

  if (_qp_spline_st_speed_config.jerk_kernel_weight() > 0) {
    spline_kernel->add_third_order_derivative_matrix(
        _qp_spline_st_speed_config.jerk_kernel_weight());
  }

  // TODO(all): add reference speed profile for different main decision
  std::vector<double> t_knots;
  std::vector<double> s_vec;
  double t_resolution = _qp_spline_st_speed_config.total_time() /
                        _qp_spline_st_speed_config.number_of_discrete_graph_t();

  // TODO: change reference line kernel to configurable version
  if (speed_limit.speed_limits().size() == 0) {
    return Status(ErrorCode::PLANNING_ERROR, "QpSplineStGraph::apply_kernel");
  }

  double dist_ref = 0.0;
  for (int i = 0; i <=
       _qp_spline_st_speed_config.number_of_discrete_graph_t(); ++i) {
      t_knots.push_back(i * t_resolution);
      s_vec.push_back(dist_ref);
      dist_ref += t_resolution * speed_limit.get_speed_limit(dist_ref);
  }
  spline_kernel->add_reference_line_kernel_matrix(t_knots, s_vec, 1);
  return Status::OK();
}

Status QpSplineStGraph::solve() {
  return _spline_generator->solve()
             ? Status::OK()
             : Status(ErrorCode::PLANNING_ERROR, "QpSplineStGraph::solve");
}

Status QpSplineStGraph::get_s_constraints_by_time(
    const std::vector<StGraphBoundary>& boundaries, const double time,
    const double total_path_s, double* const s_upper_bound,
    double* const s_lower_bound) const {
  *s_upper_bound =
      std::min(total_path_s, time * _qp_spline_st_speed_config.max_speed());

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
