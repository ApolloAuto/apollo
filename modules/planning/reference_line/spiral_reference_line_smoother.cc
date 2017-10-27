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

/*
 * spiral_reference_line_smoother.cc
 */

#include "modules/planning/reference_line/spiral_reference_line_smoother.h"

#include <algorithm>
#include <iomanip>
#include <utility>

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include "glog/logging.h"

#include "modules/planning/math/curve1d/quintic_spiral_path.h"
#include "modules/planning/reference_line/spiral_problem_interface.h"

namespace apollo {
namespace planning {

bool SpiralReferenceLineSmoother::Smooth(
    const ReferenceLine& raw_reference_line,
    ReferenceLine* const smoothed_reference_line) const {
  const double piecewise_length = 10.0;
  const double length = raw_reference_line.Length();
  ADEBUG << "Length = " << length;
  uint32_t num_of_pieces =
      std::max(1u, static_cast<uint32_t>(length / piecewise_length));

  const double delta_s = length / num_of_pieces;
  double s = 0.0;

  std::vector<Eigen::Vector2d> raw_point2d;
  for (std::uint32_t i = 0; i <= num_of_pieces;
       ++i, s = std::fmin(s + delta_s, length)) {
    ReferencePoint rlp = raw_reference_line.GetReferencePoint(s);
    raw_point2d.emplace_back(rlp.x(), rlp.y());
  }

  std::vector<common::PathPoint> smoothed_point2d;
  Smooth(raw_point2d, &smoothed_point2d);

  std::vector<ReferencePoint> ref_points;
  for (const auto& p : smoothed_point2d) {
    const double heading = p.theta();
    const double kappa = p.kappa();
    const double dkappa = p.dkappa();

    common::SLPoint ref_sl_point;
    if (!raw_reference_line.XYToSL({p.x(), p.y()}, &ref_sl_point)) {
      return false;
    }
    if (ref_sl_point.s() < 0 ||
        ref_sl_point.s() > raw_reference_line.Length()) {
      continue;
    }

    ReferencePoint rlp = raw_reference_line.GetReferencePoint(ref_sl_point.s());
    ref_points.emplace_back(
        ReferencePoint(hdmap::MapPathPoint(common::math::Vec2d(p.x(), p.y()),
                                           heading, rlp.lane_waypoints()),
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

bool SpiralReferenceLineSmoother::Smooth(
    std::vector<Eigen::Vector2d> point2d,
    std::vector<common::PathPoint>* ptr_smoothed_point2d) const {
  CHECK_GT(point2d.size(), 1);

  SpiralProblemInterface* ptop = new SpiralProblemInterface(point2d);
  ptop->set_max_point_deviation(max_point_deviation_);

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  //  app->Options()->SetStringValue("jacobian_approximation",
  //  "finite-difference-values");
  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  //  app->Options()->SetStringValue("derivative_test", "first-order");
  //  app->Options()->SetNumericValue("derivative_test_perturbation", 1.0e-7);
  //  app->Options()->SetStringValue("derivative_test", "second-order");
  app->Options()->SetIntegerValue("print_level", 0);
  int num_iterations = 10;
  app->Options()->SetIntegerValue("max_iter", num_iterations);

  //  app->Options()->SetNumericValue("acceptable_tol", 0.5);
  //  app->Options()->SetNumericValue("acceptable_obj_change_tol", 0.5);
  //  app->Options()->SetNumericValue("constr_viol_tol", 0.01);
  //  app->Options()->SetIntegerValue("acceptable_iter", 10);
  //  app->Options()->SetIntegerValue("print_level", 0);
  // app->Options()->SetStringValue("fast_step_computation", "yes");

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    ADEBUG << "*** Error during initialization!";
    return static_cast<int>(status);
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    ADEBUG << "*** The problem solved in " << iter_count << " iterations!";

    Ipopt::Number final_obj = app->Statistics()->FinalObjective();
    ADEBUG << "*** The final value of the objective function is " << final_obj
           << '.';
  } else {
    ADEBUG << "Return status: " << int(status);
  }

  std::vector<double> theta;
  std::vector<double> kappa;
  std::vector<double> dkappa;
  std::vector<double> s;
  std::vector<double> x;
  std::vector<double> y;

  ptop->get_optimization_results(&theta, &kappa, &dkappa, &s, &x, &y);

  double start_s = 0.0;
  common::PathPoint first_point =
      to_path_point(x.front(), y.front(), start_s, theta.front(), kappa.front(),
                    dkappa.front());
  ptr_smoothed_point2d->push_back(first_point);

  for (std::size_t i = 0; i + 1 < theta.size(); ++i) {
    double start_x = x[i];
    double start_y = y[i];

    auto path_point_seg =
        to_path_points(start_x, start_y, start_s, theta[i], kappa[i], dkappa[i],
                       theta[i + 1], kappa[i + 1], dkappa[i + 1], s[i], 0.1);

    ptr_smoothed_point2d->insert(ptr_smoothed_point2d->end(),
                                 path_point_seg.begin(), path_point_seg.end());

    start_s = ptr_smoothed_point2d->back().s();
  }

  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}

void SpiralReferenceLineSmoother::set_max_point_deviation(const double d) {
  CHECK(d >= 0.0);
  max_point_deviation_ = d;
}

std::vector<common::PathPoint> SpiralReferenceLineSmoother::to_path_points(
    const double start_x, const double start_y, const double start_s,
    const double theta0, const double kappa0, const double dkappa0,
    const double theta1, const double kappa1, const double dkappa1,
    const double delta_s, const double resolution) const {
  std::vector<common::PathPoint> path_points;

  QuinticSpiralPath spiral_curve(theta0, kappa0, dkappa0, theta1, kappa1,
                                 dkappa1, delta_s);
  std::size_t num_of_points = std::ceil(delta_s / resolution) + 1;
  for (std::size_t i = 1; i <= num_of_points; ++i) {
    double inter_s = delta_s / num_of_points * i;
    double dx = spiral_curve.ComputeCartesianDeviationX<10>(inter_s);
    double dy = spiral_curve.ComputeCartesianDeviationY<10>(inter_s);

    double theta = spiral_curve.Evaluate(0, inter_s);  // need to be normalized.
    double kappa = spiral_curve.Evaluate(1, inter_s);
    double dkappa = spiral_curve.Evaluate(2, inter_s);

    auto path_point = to_path_point(start_x + dx, start_y + dy,
                                    start_s + inter_s, theta, kappa, dkappa);
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

common::PathPoint SpiralReferenceLineSmoother::to_path_point(
    const double x, const double y, const double s, const double theta,
    const double kappa, const double dkappa) const {
  common::PathPoint point;
  point.set_x(x);
  point.set_y(y);
  point.set_s(s);
  point.set_theta(theta);
  point.set_kappa(kappa);
  point.set_dkappa(dkappa);
  return point;
}

}  // namespace planning
}  // namespace apollo
