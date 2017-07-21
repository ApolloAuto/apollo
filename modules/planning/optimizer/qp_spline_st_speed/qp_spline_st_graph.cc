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

#include "modules/common/log.h"
#include "modules/planning/util/planning_util.h"

namespace apollo {
namespace planning {

using ErrorCode = apollo::common::ErrorCode;
using VehicleParam = apollo::common::config::VehicleParam;

QpSplineStGraph::QpSplineStGraph(
    const QpSplineStSpeedConfig& qp_spline_st_speed_config,
    const apollo::common::config::VehicleParam& veh_param)
    : _qp_spline_st_speed_config(qp_spline_st_speed_config) {}

ErrorCode QpSplineStGraph::search(const STGraphData& st_graph_data,
                                  const PathData& path_data,
                                  SpeedData* const speed_data) {
  _init_point = st_graph_data.init_point();
  if (st_graph_data.path_data_length() <
      _qp_spline_st_speed_config.total_path_length()) {
    _qp_spline_st_speed_config.set_total_path_length(
        st_graph_data.path_data_length());
  }

  // TODO: update speed limit here
  // TODO: update config through veh physical limit here
  // generate knots
  std::vector<double> t_knots;
  double t_resolution = _qp_spline_st_speed_config.total_time() /
                        _qp_spline_st_speed_config.number_of_discrete_graph_t();
  for (int32_t i = 0;
       i <= _qp_spline_st_speed_config.number_of_discrete_graph_t(); ++i) {
    t_knots.push_back(i * t_resolution);
  }

  _spline_generator.reset(new Spline1dGenerator(
      t_knots, _qp_spline_st_speed_config.spline_order()));

  if (apply_constraint(st_graph_data.obs_boundary()) !=
      ErrorCode::PLANNING_OK) {
    AERROR << "Apply constraint failed!";
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (apply_kernel() != ErrorCode::PLANNING_OK) {
    AERROR << "Apply kernel failed!";
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (solve() != ErrorCode::PLANNING_OK) {
    AERROR << "Solve qp problem failed!";
    return ErrorCode::PLANNING_ERROR_FAILED;
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
    speed_data->mutable_speed_vector()->push_back(
        util::MakeSpeedPoint(s, time, v, a, da));
  }

  return ErrorCode::PLANNING_OK;
}

ErrorCode QpSplineStGraph::apply_constraint(
    const std::vector<STGraphBoundary>& boundaries) {
  Spline1dConstraint* constraint =
      _spline_generator->mutable_spline_constraint();
  // position, velocity, acceleration
  if (!constraint->add_point_fx_constraint(0.0, 0.0)) {
    AERROR << "add st start point constraint failed";
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!constraint->add_point_derivative_constraint(0.0, _init_point.v())) {
    AERROR << "add st start point velocity constraint failed!";
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!constraint->add_point_second_derivative_constraint(0.0,
                                                          _init_point.a())) {
    AERROR << "add st start point acceleration constraint failed!";
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  if (!constraint->add_point_second_derivative_constraint(
          _spline_generator->spline().x_knots().back(), 0.0)) {
    AERROR << "add st end point acceleration constraint failed!";
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  // monotone constraint
  if (!constraint->add_monotone_fx_inequality_constraint_at_knots()) {
    AERROR << "add monotonicity constraint failed!";
    return ErrorCode::PLANNING_ERROR_FAILED;
  }

  // smoothness constraint
  if (!constraint->add_third_derivative_smooth_constraint()) {
    AERROR << "add smoothness joint constraint failed!";
    return ErrorCode::PLANNING_ERROR_FAILED;
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
    AERROR << "Fail to apply obstacle constraint";
    return ErrorCode::PLANNING_ERROR_FAILED;
  }
  // TODO: add speed constraint
  return ErrorCode::PLANNING_OK;
}

ErrorCode QpSplineStGraph::apply_kernel() {
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

  // TODO add reference speed profile for different main decision
  std::vector<double> t_knots;
  std::vector<double> s_vec;
  double t_resolution = _qp_spline_st_speed_config.total_time() /
                        _qp_spline_st_speed_config.number_of_discrete_graph_t();
  for (int32_t i = 0;
       i <= _qp_spline_st_speed_config.number_of_discrete_graph_t(); ++i) {
    t_knots.push_back(i * t_resolution);
    s_vec.push_back(i * t_resolution * 10);
  }

  spline_kernel->add_reference_line_kernel_matrix(t_knots, s_vec, 1);

  return ErrorCode::PLANNING_OK;
}

ErrorCode QpSplineStGraph::solve() {
  return _spline_generator->solve() ? ErrorCode::PLANNING_OK
                                    : ErrorCode::PLANNING_ERROR_FAILED;
}

ErrorCode QpSplineStGraph::get_s_constraints_by_time(
    const std::vector<STGraphBoundary>& boundaries, const double time,
    const double total_path_s, double* const s_upper_bound,
    double* const s_lower_bound) const {
  *s_upper_bound =
      std::min(total_path_s, time * _qp_spline_st_speed_config.max_speed());

  for (const STGraphBoundary& boundary : boundaries) {
    double s_upper = 0.0;
    double s_lower = 0.0;

    if (!boundary.get_s_boundary_position(time, &s_upper, &s_lower)) {
      continue;
    }

    if (boundary.boundary_type() == STGraphBoundary::BoundaryType::STOP ||
        boundary.boundary_type() == STGraphBoundary::BoundaryType::FOLLOW ||
        boundary.boundary_type() == STGraphBoundary::BoundaryType::YIELD) {
      *s_upper_bound = std::fmin(*s_upper_bound, s_upper);
    } else {
      *s_lower_bound = std::fmax(*s_lower_bound, s_lower);
    }
  }

  return ErrorCode::PLANNING_OK;
}
}  // namespace planning
}  // namespace apollo
