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
 * @file qp_spline_path_generator.cpp
 **/

#include "optimizer/qp_spline_path_optimizer/qp_spline_path_generator.h"
#include "common/houston_gflags.h"
#include "common/logger.h"
#include "common/path/frenet_frame_point.h"
#include "common/path/sl_point.h"
#include "common/planning_macros.h"
#include "math/sl_analytic_transformation.h"
#include "optimizer/qp_spline_path_optimizer/qp_spline_path_simple_sampler.h"

namespace apollo {
namespace planning {

QPSplinePathGenerator::QPSplinePathGenerator(
    const boost::property_tree::ptree& property)
    : _qp_spline_path_configuration(property) {
  _qp_frenet_frame.reset(new QpFrenetFrame());
}

bool QPSplinePathGenerator::generate(
    const Environment& environment, const ReferenceLine& reference_line,
    const DecisionData& decision_data, const SpeedData& speed_data,
    const ::adu::planning::TrajectoryPoint& init_point,
    PathData* const path_data) {
  QUIT_IF(!calculate_sl_point(reference_line, init_point, &_init_point), false,
          Level::ERROR, "Fail to map init point.");

  double start_s = _init_point.s();
  double end_s = std::min(reference_line.reference_map_line().length(),
                          _init_point.s() + FLAGS_planning_distance);

  QUIT_IF(
      _qp_frenet_frame->init(
          environment, reference_line, decision_data, speed_data, _init_point,
          start_s, end_s, _qp_spline_path_configuration.time_resolution(),
          _qp_spline_path_configuration.num_output()) != ErrorCode::PLANNING_OK,
      false, Level::ERROR, "Fail to initialize qp frenet frame");

  QUIT_IF(!init_coord_range(&start_s, &end_s), false, Level::ERROR,
          "Measure natural coord system with s range failed!");

  IDG_LOG(Level::INFO, "pss path start with %f, end with %f", start_s, end_s);
  QUIT_IF(!init_smoothing_spline(environment, reference_line, start_s, end_s),
          false, Level::ERROR, "Init smoothing spline failed with (%f, %f)!",
          start_s, end_s);

  QUIT_IF(!setup_constraint(), false, Level::ERROR,
          "Fail to setup pss path constraint.");
  QUIT_IF(!setup_kernel(), false, Level::ERROR,
          "Fail to setup pss path kernel.");
  QUIT_IF(!solve(), false, Level::ERROR, "Fail to solve the qp problem.");

  // extract data
  const Spline1d& spline = _spline_generator->spline();
  double s_resolution =
      (end_s - _init_point.s()) / _qp_spline_path_configuration.num_output();
  std::vector<PathPoint> path_points;

  double start_l = spline(_init_point.s());
  ReferencePoint ref_point =
      reference_line.get_reference_point(_init_point.s());
  Eigen::Vector2d xy_point = SLAnalyticTransformation::calculate_xypoint(
      ref_point.heading(), Eigen::Vector2d(ref_point.x(), ref_point.y()),
      start_l);

  double x_diff = xy_point.x() - init_point.x();
  double y_diff = xy_point.y() - init_point.y();
  for (std::size_t i = 0; i <= _qp_spline_path_configuration.num_output();
       ++i) {
    double s = _init_point.s() + s_resolution * i;
    double l = spline(s);
    double dl = spline.derivative(s);
    double ddl = spline.second_order_derivative(s);
    ReferencePoint ref_point = reference_line.get_reference_point(s);
    Eigen::Vector2d xy_point = SLAnalyticTransformation::calculate_xypoint(
        ref_point.heading(), Eigen::Vector2d(ref_point.x(), ref_point.y()), l);
    xy_point.x() = (xy_point.x() - x_diff);
    xy_point.y() = (xy_point.y() - y_diff);
    double theta = SLAnalyticTransformation::calculate_theta(
        ref_point.heading(), ref_point.kappa(), l, dl);
    double kappa = SLAnalyticTransformation::calculate_kappa(
        ref_point.kappa(), ref_point.dkappa(), l, dl, ddl);
    PathPoint path_point(xy_point, theta, kappa, 0.0, 0.0, 0.0);
    if (path_points.size() != 0) {
      double distance =
          (path_points.back().coordinate() - path_point.coordinate()).norm();
      path_point.set_s(path_points.back().s() + distance);
    }
    path_points.push_back(std::move(path_point));
  }
  *(path_data->mutable_path()->mutable_path_points()) = path_points;
  // TODO: refine the path init point mapping error;
  return true;
}

bool QPSplinePathGenerator::calculate_sl_point(
    const ReferenceLine& reference_line,
    const ::adu::planning::TrajectoryPoint& traj_point,
    FrenetFramePoint* const sl_point) {
  if (!reference_line.get_point_in_Frenet_frame(
          {traj_point.x(), traj_point.y()}, sl_point)) {
    return false;
  };
  const double theta = traj_point.theta();
  const double kappa = traj_point.kappa();
  const double l = sl_point->l();

  ReferencePoint ref_point = reference_line.get_reference_point(sl_point->s());

  const double theta_ref = ref_point.heading();
  const double kappa_ref = ref_point.kappa();
  const double dkappa_ref = ref_point.dkappa();

  const double dl = SLAnalyticTransformation::calculate_lateral_derivative(
      theta_ref, theta, l, kappa_ref);
  const double ddl =
      SLAnalyticTransformation::calculate_second_order_lateral_derivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  sl_point->set_dl(dl);
  sl_point->set_ddl(ddl);
  return true;
}

bool QPSplinePathGenerator::init_coord_range(double* const start_s,
                                             double* const end_s) {
  // TODO: step 1 get current sl coordinate - with init coordinate point
  double start_point = std::max(_init_point.s() - 5.0, 0.0);

  const ReferenceLine* reference_line = _qp_frenet_frame->reference_line();

  if (reference_line == nullptr) {
    IDG_LOG(Level::ERROR,
            "Could not retrieve reference line from frenet frame");
    return false;
  }
  double end_point = std::min(reference_line->reference_map_line().length(),
                              (*start_s + FLAGS_planning_distance));

  end_point = std::min(_qp_frenet_frame->feasible_longitudinal_upper_bound(),
                       end_point);
  *start_s = start_point;
  *end_s = end_point;
  return true;
}

bool QPSplinePathGenerator::init_smoothing_spline(
    const Environment& environment, const ReferenceLine& reference_line,
    const double start_s, const double end_s) {
  std::size_t num_knots = _qp_spline_path_configuration.number_of_spline();
  QPSplinePathSimpleSampler simple_sampler;

  // TODO refine here, add end_s tolorence here
  std::vector<double> sampling_point;
  QUIT_IF(simple_sampler.sample(environment, _init_point, reference_line,
                                num_knots, start_s, end_s - 0.1,
                                &sampling_point) != ErrorCode::PLANNING_OK,
          false, Level::ERROR, "Qp spline sampler failed!");

  _spline_generator.reset(new Spline1dGenerator(
      sampling_point, _qp_spline_path_configuration.spline_order()));
  return true;
}

bool QPSplinePathGenerator::setup_constraint() {
  Spline1dConstraint* spline_constraint =
      _spline_generator->mutable_spline_constraint();
  // add init status constraint
  spline_constraint->add_point_fx_constraint(_init_point.s(), _init_point.l());
  spline_constraint->add_point_derivative_constraint(_init_point.s(),
                                                     _init_point.dl());
  spline_constraint->add_point_second_derivative_constraint(_init_point.s(),
                                                            _init_point.ddl());

  IDG_LOG(Level::INFO, "init frenet point: %f, %f, %f, %f", _init_point.s(),
          _init_point.l(), _init_point.dl(), _init_point.ddl());

  // add end point constraint
  const std::vector<double> spline_knots =
      _spline_generator->spline().x_knots();
  QUIT_IF(spline_knots.size() < 2, false, Level::ERROR,
          "Smoothing spline knot size < 2");
  double s_length = spline_knots.back() - spline_knots.front();
  QUIT_IF(s_length <= 0, false, Level::ERROR,
          "Smoothing spline knot lenght <= 0");

  std::pair<double, double> boundary = std::make_pair(0.0, 0.0);
  double end_ref_l = (boundary.first + boundary.second) / 2.0;
  spline_constraint->add_point_fx_constraint(spline_knots.back(), end_ref_l);
  spline_constraint->add_point_derivative_constraint(spline_knots.back(), 0.0);
  spline_constraint->add_point_second_derivative_constraint(spline_knots.back(),
                                                            0.0);
  IDG_LOG(Level::INFO, "end frenet point %f, %f, %f, %f", s_length, end_ref_l,
          0.0, 0.0);
  const std::vector<double> sampling_knots =
      _spline_generator->spline().x_knots();

  if (sampling_knots.size() <= 2) {
    return false;
  }

  std::size_t num_fx_bound =
      _qp_spline_path_configuration.number_of_fx_constraint_knots();
  std::vector<double> boundary_low;
  std::vector<double> boundary_high;
  std::vector<double> fx_knots;

  if (num_fx_bound > 1) {
    double ds = (sampling_knots.back() - sampling_knots.front()) / num_fx_bound;

    for (std::size_t i = 0; i < num_fx_bound + 1; ++i) {
      double s = sampling_knots.front() + i * ds;
      fx_knots.push_back(s);
      std::pair<double, double> boundary = std::make_pair(0.0, 0.0);
      _qp_frenet_frame->get_map_bound(s, &boundary);
      boundary_low.push_back(boundary.first);
      boundary_high.push_back(boundary.second);
      // calculate boundary here
      IDG_LOG(Level::INFO, "boundary info %f, %f, %f", s, boundary.first,
              boundary.second);
    }

    QUIT_IF(!spline_constraint->add_fx_boundary(fx_knots, boundary_low,
                                                boundary_high),
            false, Level::ERROR, "Add boundary constraint failed");
  }
  // add smooth jointness constraint
  QUIT_IF(!spline_constraint->add_third_derivative_smooth_constraint(), false,
          Level::ERROR, "Add spline jointness constraint failed!");

  return true;
}

bool QPSplinePathGenerator::setup_kernel() {
  Spline1dKernel* spline_kernel = _spline_generator->mutable_spline_kernel();

  if (_qp_spline_path_configuration.regularization_weight() > 0) {
    spline_kernel->add_regularization(
        _qp_spline_path_configuration.regularization_weight());
  }

  if (_qp_spline_path_configuration.derivative_weight() > 0) {
    spline_kernel->add_derivative_kernel_matrix(
        _qp_spline_path_configuration.derivative_weight());
  }

  if (_qp_spline_path_configuration.second_derivative_weight() > 0) {
    spline_kernel->add_second_order_derivative_matrix(
        _qp_spline_path_configuration.second_derivative_weight());
  }

  if (_qp_spline_path_configuration.third_derivative_weight() > 0) {
    spline_kernel->add_third_order_derivative_matrix(
        _qp_spline_path_configuration.third_derivative_weight());
  }

  // TODO: Add reference line kernel here
  return true;
}

bool QPSplinePathGenerator::solve() {
  if (!_spline_generator->solve()) {
    IDG_LOG(Level::ERROR,
            "Could not solve the qp problem in spline path generator.");
    return false;
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
