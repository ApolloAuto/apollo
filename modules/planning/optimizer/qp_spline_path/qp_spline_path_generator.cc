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

#include "modules/planning/optimizer/qp_spline_path/qp_spline_path_generator.h"

#include "modules/common/proto/path_point.pb.h"

#include "modules/common/log.h"
#include "modules/common/macro.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/sl_analytic_transformation.h"
#include "modules/planning/optimizer/qp_spline_path/qp_spline_path_sampler.h"

namespace apollo {
namespace planning {

bool QpSplinePathGenerator::SetConfig(const std::string& config_file) {
  if (!common::util::GetProtoFromFile(config_file, &_qp_spline_path_config)) {
    AERROR << "Failed to load config file " << config_file;
    return false;
  }
  return true;
}

bool QpSplinePathGenerator::generate(const ReferenceLine& reference_line,
                                     const DecisionData& decision_data,
                                     const SpeedData& speed_data,
                                     const common::TrajectoryPoint& init_point,
                                     PathData* const path_data) {
  if (!calculate_sl_point(reference_line, init_point, &_init_point)) {
    AERROR << "Fail to map init point: " << init_point.ShortDebugString();
    return false;
  }
  double start_s = _init_point.s();
  double end_s = std::min(reference_line.length(),
                          _init_point.s() + FLAGS_planning_distance);

  if (!_qp_frenet_frame.Init(reference_line, decision_data, speed_data,
                             _init_point, start_s, end_s,
                             _qp_spline_path_config.time_resolution(),
                             _qp_spline_path_config.num_output())) {
    AERROR << "Fail to initialize qp frenet frame";
    return false;
  }

  if (!init_coord_range(&start_s, &end_s)) {
    AERROR << "Measure natural coord system with s range failed!";
    return false;
  }

  AINFO << "pss path start with " << start_s << ", end with " << end_s;
  if (!init_smoothing_spline(reference_line, start_s, end_s)) {
    AERROR << "Init smoothing spline failed with (" << start_s << ",  end_s "
           << end_s;
    return false;
  }

  if (!setup_constraint()) {
    AERROR << "Fail to setup pss path constraint.";
    return false;
  }
  if (!setup_kernel()) {
    AERROR << "Fail to setup pss path kernel.";
    return false;
  }
  if (!solve()) {
    AERROR << "Fail to solve the qp problem.";
    return false;
  }

  AINFO << common::util::StrCat("Spline dl:", _init_point.dl(), ", ddl:",
                                _init_point.ddl());

  // extract data
  const Spline1d& spline = _spline_generator->spline();
  double s_resolution =
      (end_s - _init_point.s()) / _qp_spline_path_config.num_output();
  std::vector<common::PathPoint> path_points;

  double start_l = spline(_init_point.s());
  ReferencePoint ref_point =
      reference_line.get_reference_point(_init_point.s());
  common::math::Vec2d xy_point = SLAnalyticTransformation::calculate_xypoint(
      ref_point.heading(), common::math::Vec2d(ref_point.x(), ref_point.y()),
      start_l);

  double x_diff = xy_point.x() - init_point.path_point().x();
  double y_diff = xy_point.y() - init_point.path_point().y();

  double s = _init_point.s();
  for (std::uint32_t i = 0; i <= _qp_spline_path_config.num_output(); ++i) {
    double l = spline(s);
    double dl = spline.derivative(s);
    double ddl = spline.second_order_derivative(s);
    ReferencePoint ref_point = reference_line.get_reference_point(s);
    common::math::Vec2d xy_point = SLAnalyticTransformation::calculate_xypoint(
        ref_point.heading(), common::math::Vec2d(ref_point.x(), ref_point.y()),
        l);
    xy_point.set_x(xy_point.x() - x_diff);
    xy_point.set_y(xy_point.y() - y_diff);
    double theta = SLAnalyticTransformation::calculate_theta(
        ref_point.heading(), ref_point.kappa(), l, dl);
    double kappa = SLAnalyticTransformation::calculate_kappa(
        ref_point.kappa(), ref_point.dkappa(), l, dl, ddl);
    common::PathPoint path_point = common::util::MakePathPoint(
        xy_point.x(), xy_point.y(), 0.0, theta, kappa, 0.0, 0.0);
    if (path_points.size() != 0) {
      double distance =
          common::util::Distance2D(path_points.back(), path_point);
      path_point.set_s(path_points.back().s() + distance);
    }
    path_points.push_back(std::move(path_point));
    s += s_resolution;
  }
  path_data->set_discretized_path(path_points);
  return true;
}

bool QpSplinePathGenerator::calculate_sl_point(
    const ReferenceLine& reference_line,
    const common::TrajectoryPoint& traj_point,
    common::FrenetFramePoint* const frenet_frame_point) {
  common::SLPoint sl_point;
  if (!reference_line.get_point_in_frenet_frame(
          {traj_point.path_point().x(), traj_point.path_point().y()},
          &sl_point)) {
    return false;
  }
  frenet_frame_point->set_s(sl_point.s());
  frenet_frame_point->set_l(sl_point.l());
  const double theta = traj_point.path_point().theta();
  const double kappa = traj_point.path_point().kappa();
  const double l = frenet_frame_point->l();

  ReferencePoint ref_point =
      reference_line.get_reference_point(frenet_frame_point->s());

  const double theta_ref = ref_point.heading();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();

  const double dl = SLAnalyticTransformation::calculate_lateral_derivative(
      theta_ref, theta, l, kappa_ref);
  const double ddl =
      SLAnalyticTransformation::calculate_second_order_lateral_derivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point->set_dl(dl);
  frenet_frame_point->set_ddl(ddl);
  return true;
}

bool QpSplinePathGenerator::init_coord_range(double* const start_s,
                                             double* const end_s) {
  // TODO(all): step 1 get current sl coordinate - with init coordinate point
  double start_point = std::max(_init_point.s() - 5.0, 0.0);

  const ReferenceLine* reference_line = _qp_frenet_frame.reference_line();

  if (reference_line == nullptr) {
    AERROR << "Could not retrieve reference line from frenet frame";
    return false;
  }
  double end_point =
      std::min(reference_line->length(), *start_s + FLAGS_planning_distance);

  end_point =
      std::min(_qp_frenet_frame.feasible_longitudinal_upper_bound(), end_point);
  *start_s = start_point;
  *end_s = end_point;
  return true;
}

bool QpSplinePathGenerator::init_smoothing_spline(
    const ReferenceLine& reference_line, const double start_s,
    const double end_s) {
  QpSplinePathSampler sampler(_qp_spline_path_config);

  // TODO(all): refine here, add end_s tolorence here
  std::vector<double> sampling_point;
  if (!sampler.Sample(_init_point, reference_line, start_s, end_s - 0.1,
                      &sampling_point)) {
    AERROR << "Qp spline sampler failed!";
    return false;
  }

  _spline_generator.reset(new Spline1dGenerator(
      sampling_point, _qp_spline_path_config.spline_order()));
  return true;
}

bool QpSplinePathGenerator::setup_constraint() {
  Spline1dConstraint* spline_constraint =
      _spline_generator->mutable_spline_constraint();
  // add init status constraint
  spline_constraint->add_point_fx_constraint(_init_point.s(), _init_point.l());
  spline_constraint->add_point_derivative_constraint(_init_point.s(),
                                                     _init_point.dl());
  spline_constraint->add_point_second_derivative_constraint(_init_point.s(),
                                                            _init_point.ddl());

  AINFO << "init frenet point: " << _init_point.ShortDebugString();

  // add end point constraint
  const std::vector<double> spline_knots =
      _spline_generator->spline().x_knots();
  if (spline_knots.size() < 2) {
    AERROR << common::util::StrCat("Smoothing spline knot size(",
                                   spline_knots.size(), ") < 2");
    return false;
  }
  double s_length = spline_knots.back() - spline_knots.front();
  if (s_length <= 0) {
    AERROR << common::util::StrCat("Smoothing spline knot length(", s_length,
                                   ") <= 0");
    return false;
  }

  std::pair<double, double> boundary = std::make_pair(0.0, 0.0);
  double end_ref_l = (boundary.first + boundary.second) / 2.0;
  spline_constraint->add_point_fx_constraint(spline_knots.back(), end_ref_l);
  spline_constraint->add_point_derivative_constraint(spline_knots.back(), 0.0);
  spline_constraint->add_point_second_derivative_constraint(spline_knots.back(),
                                                            0.0);
  AINFO << "end frenet point:" << s_length << ", " << end_ref_l
        << ", 0.0, 0.0.";
  const std::vector<double> sampling_knots = spline_knots;

  if (sampling_knots.size() <= 2) {
    return false;
  }

  std::uint32_t num_fx_bound =
      _qp_spline_path_config.number_of_fx_constraint_knots();
  std::vector<double> boundary_low;
  std::vector<double> boundary_high;
  std::vector<double> fx_knots;

  if (num_fx_bound > 1) {
    double ds = (sampling_knots.back() - sampling_knots.front()) / num_fx_bound;
    double s = sampling_knots.front();

    for (std::uint32_t i = 0; i < num_fx_bound + 1; ++i) {
      fx_knots.push_back(s);
      std::pair<double, double> boundary = std::make_pair(0.0, 0.0);
      _qp_frenet_frame.get_map_bound(s, &boundary);
      boundary_low.push_back(boundary.first);
      boundary_high.push_back(boundary.second);
      s += ds;
      // calculate boundary here
    }

    if (!spline_constraint->add_fx_boundary(fx_knots, boundary_low,
                                            boundary_high)) {
      AERROR << "Add boundary constraint failed";
      return false;
    }
  }
  // add smooth jointness constraint
  if (!spline_constraint->add_third_derivative_smooth_constraint()) {
    AERROR << "Add spline jointness constraint failed!";
    return false;
  }

  return true;
}

bool QpSplinePathGenerator::setup_kernel() {
  Spline1dKernel* spline_kernel = _spline_generator->mutable_spline_kernel();

  if (_qp_spline_path_config.regularization_weight() > 0) {
    spline_kernel->add_regularization(
        _qp_spline_path_config.regularization_weight());
  }

  if (_qp_spline_path_config.derivative_weight() > 0) {
    spline_kernel->add_derivative_kernel_matrix(
        _qp_spline_path_config.derivative_weight());
  }

  if (_qp_spline_path_config.second_derivative_weight() > 0) {
    spline_kernel->add_second_order_derivative_matrix(
        _qp_spline_path_config.second_derivative_weight());
  }

  if (_qp_spline_path_config.third_derivative_weight() > 0) {
    spline_kernel->add_third_order_derivative_matrix(
        _qp_spline_path_config.third_derivative_weight());
  }

  // TODO(all): Add reference line kernel here
  return true;
}

bool QpSplinePathGenerator::solve() {
  if (!_spline_generator->solve()) {
    AERROR << "Could not solve the qp problem in spline path generator.";
    return false;
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
