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

#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>

#include <algorithm>
#include <utility>

#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/curve1d/quintic_spiral_path.h"
#include "modules/planning/reference_line/spiral_problem_interface.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;

SpiralReferenceLineSmoother::SpiralReferenceLineSmoother(
    const ReferenceLineSmootherConfig& config)
    : ReferenceLineSmoother(config) {}

bool SpiralReferenceLineSmoother::Smooth(
    const ReferenceLine& raw_reference_line,
    ReferenceLine* const smoothed_reference_line) {
  const double start_timestamp = Clock::NowInSeconds();
  std::vector<double> opt_x;
  std::vector<double> opt_y;
  std::vector<double> opt_theta;
  std::vector<double> opt_kappa;
  std::vector<double> opt_dkappa;
  std::vector<double> opt_s;

  if (anchor_points_.empty()) {
    const double piecewise_length = config_.spiral().piecewise_length();
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

    Smooth(raw_point2d, &opt_theta, &opt_kappa, &opt_dkappa, &opt_s, &opt_x,
           &opt_y);
  } else {
    size_t start_index = 0;
    for (const auto& anchor_point : anchor_points_) {
      if (anchor_point.enforced) {
        start_index++;
      } else {
        break;
      }
    }

    std::vector<Eigen::Vector2d> raw_point2d;
    if (start_index == 0) {
      for (const auto& anchor_point : anchor_points_) {
        raw_point2d.emplace_back(anchor_point.path_point.x(),
                                 anchor_point.path_point.y());
      }
    } else {
      std::vector<double> overhead_s;
      for (size_t i = 0; i + 1 < start_index; ++i) {
        const auto& p0 = anchor_points_[i];
        const auto& p1 = anchor_points_[i + 1];
        overhead_s.push_back(p1.path_point.s() - p0.path_point.s());
      }

      std::vector<double> overhead_theta;
      std::vector<double> overhead_kappa;
      std::vector<double> overhead_dkappa;
      std::vector<double> overhead_x;
      std::vector<double> overhead_y;
      for (size_t i = 0; i < anchor_points_.size(); ++i) {
        const auto& p = anchor_points_[i];
        if (i + 1 < start_index) {
          overhead_theta.push_back(p.path_point.theta());
          overhead_kappa.push_back(p.path_point.kappa());
          overhead_dkappa.push_back(p.path_point.dkappa());
          overhead_x.push_back(p.path_point.x());
          overhead_y.push_back(p.path_point.y());
        } else {
          raw_point2d.emplace_back(p.path_point.x(), p.path_point.y());
        }
      }

      const auto& start_anchor_point = anchor_points_[start_index - 1];
      fixed_start_point_ = true;
      fixed_start_x_ = start_anchor_point.path_point.x();
      fixed_start_y_ = start_anchor_point.path_point.y();
      fixed_start_theta_ =
          common::math::NormalizeAngle(start_anchor_point.path_point.theta());
      fixed_start_kappa_ = start_anchor_point.path_point.kappa();
      fixed_start_dkappa_ = start_anchor_point.path_point.dkappa();

      const auto& end_anchor_point = anchor_points_.back();
      fixed_end_x_ = end_anchor_point.path_point.x();
      fixed_end_y_ = end_anchor_point.path_point.y();

      Smooth(raw_point2d, &opt_theta, &opt_kappa, &opt_dkappa, &opt_s, &opt_x,
             &opt_y);

      opt_theta.insert(opt_theta.begin(), overhead_theta.begin(),
                       overhead_theta.end());
      opt_kappa.insert(opt_kappa.begin(), overhead_kappa.begin(),
                       overhead_kappa.end());
      opt_dkappa.insert(opt_dkappa.begin(), overhead_dkappa.begin(),
                        overhead_dkappa.end());
      opt_s.insert(opt_s.begin(), overhead_s.begin(), overhead_s.end());
      opt_x.insert(opt_x.begin(), overhead_x.begin(), overhead_x.end());
      opt_y.insert(opt_y.begin(), overhead_y.begin(), overhead_y.end());

      std::for_each(opt_x.begin(), opt_x.end(),
                    [this](double& x) { x += zero_x_; });

      std::for_each(opt_y.begin(), opt_y.end(),
                    [this](double& y) { y += zero_y_; });
    }
  }

  std::vector<common::PathPoint> smoothed_point2d =
      Interpolate(opt_theta, opt_kappa, opt_dkappa, opt_s, opt_x, opt_y,
                  config_.resolution());

  std::vector<ReferencePoint> ref_points;
  for (const auto& p : smoothed_point2d) {
    const double heading = p.theta();
    const double kappa = p.kappa();
    const double dkappa = p.dkappa();

    common::SLPoint ref_sl_point;
    if (!raw_reference_line.XYToSL(p, &ref_sl_point)) {
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
                       kappa, dkappa));
  }

  ReferencePoint::RemoveDuplicates(&ref_points);
  if (ref_points.size() < 2) {
    AERROR << "Fail to generate smoothed reference line.";
    return false;
  }
  *smoothed_reference_line = ReferenceLine(ref_points);
  const double end_timestamp = Clock::NowInSeconds();
  ADEBUG << "Spiral reference line smoother time: "
         << (end_timestamp - start_timestamp) * 1000 << " ms.";
  return true;
}

int SpiralReferenceLineSmoother::SmoothStandAlone(
    std::vector<Eigen::Vector2d> point2d, std::vector<double>* ptr_theta,
    std::vector<double>* ptr_kappa, std::vector<double>* ptr_dkappa,
    std::vector<double>* ptr_s, std::vector<double>* ptr_x,
    std::vector<double>* ptr_y) const {
  CHECK_GT(point2d.size(), 1);

  SpiralProblemInterface* ptop = new SpiralProblemInterface(point2d);

  ptop->set_default_max_point_deviation(config_.spiral().max_deviation());
  ptop->set_element_weight_curve_length(config_.spiral().weight_curve_length());
  ptop->set_element_weight_kappa(config_.spiral().weight_kappa());
  ptop->set_element_weight_dkappa(config_.spiral().weight_dkappa());

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  app->Options()->SetIntegerValue("max_iter", config_.spiral().max_iteration());
  app->Options()->SetNumericValue("tol", config_.spiral().opt_tol());
  app->Options()->SetNumericValue("acceptable_tol",
                                  config_.spiral().opt_acceptable_tol());

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    ADEBUG << "*** Error during initialization!";
    return -1;
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

  ptop->get_optimization_results(ptr_theta, ptr_kappa, ptr_dkappa, ptr_s, ptr_x,
                                 ptr_y);

  if (!(status == Ipopt::Solve_Succeeded) &&
      !(status == Ipopt::Solved_To_Acceptable_Level)) {
    return -1;
  }
  return app->Statistics()->IterationCount();
}

bool SpiralReferenceLineSmoother::Smooth(std::vector<Eigen::Vector2d> point2d,
                                         std::vector<double>* ptr_theta,
                                         std::vector<double>* ptr_kappa,
                                         std::vector<double>* ptr_dkappa,
                                         std::vector<double>* ptr_s,
                                         std::vector<double>* ptr_x,
                                         std::vector<double>* ptr_y) const {
  CHECK_GT(point2d.size(), 1);

  SpiralProblemInterface* ptop = new SpiralProblemInterface(point2d);

  ptop->set_default_max_point_deviation(config_.spiral().max_deviation());
  if (fixed_start_point_) {
    ptop->set_start_point(fixed_start_x_, fixed_start_y_, fixed_start_theta_,
                          fixed_start_kappa_, fixed_start_dkappa_);
  }

  ptop->set_end_point_position(fixed_end_x_, fixed_end_y_);
  ptop->set_element_weight_curve_length(config_.spiral().weight_curve_length());
  ptop->set_element_weight_kappa(config_.spiral().weight_kappa());
  ptop->set_element_weight_dkappa(config_.spiral().weight_dkappa());

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  app->Options()->SetIntegerValue("print_level", 0);
  app->Options()->SetIntegerValue("max_iter", config_.spiral().max_iteration());
  app->Options()->SetIntegerValue("acceptable_iter",
                                  config_.spiral().opt_acceptable_iteration());
  app->Options()->SetNumericValue("tol", config_.spiral().opt_tol());
  app->Options()->SetNumericValue("acceptable_tol",
                                  config_.spiral().opt_acceptable_tol());

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    ADEBUG << "*** Error during initialization!";
    return false;
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

  ptop->get_optimization_results(ptr_theta, ptr_kappa, ptr_dkappa, ptr_s, ptr_x,
                                 ptr_y);

  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}

std::vector<common::PathPoint> SpiralReferenceLineSmoother::Interpolate(
    const std::vector<double>& theta, const std::vector<double>& kappa,
    const std::vector<double>& dkappa, const std::vector<double>& s,
    const std::vector<double>& x, const std::vector<double>& y,
    const double resolution) const {
  std::vector<common::PathPoint> smoothed_point2d;
  double start_s = 0.0;
  common::PathPoint first_point =
      to_path_point(x.front(), y.front(), start_s, theta.front(), kappa.front(),
                    dkappa.front());
  smoothed_point2d.push_back(first_point);

  for (size_t i = 0; i + 1 < theta.size(); ++i) {
    double start_x = x[i];
    double start_y = y[i];

    auto path_point_seg = Interpolate(
        start_x, start_y, start_s, theta[i], kappa[i], dkappa[i], theta[i + 1],
        kappa[i + 1], dkappa[i + 1], s[i], resolution);

    smoothed_point2d.insert(smoothed_point2d.end(), path_point_seg.begin(),
                            path_point_seg.end());

    start_s = smoothed_point2d.back().s();
  }
  return smoothed_point2d;
}

std::vector<common::PathPoint> SpiralReferenceLineSmoother::Interpolate(
    const double start_x, const double start_y, const double start_s,
    const double theta0, const double kappa0, const double dkappa0,
    const double theta1, const double kappa1, const double dkappa1,
    const double delta_s, const double resolution) const {
  std::vector<common::PathPoint> path_points;

  const auto angle_diff = common::math::AngleDiff(theta0, theta1);

  QuinticSpiralPath spiral_curve(theta0, kappa0, dkappa0, theta0 + angle_diff,
                                 kappa1, dkappa1, delta_s);
  size_t num_of_points =
      static_cast<size_t>(std::ceil(delta_s / resolution) + 1);
  for (size_t i = 1; i <= num_of_points; ++i) {
    const double inter_s =
        delta_s / static_cast<double>(num_of_points) * static_cast<double>(i);
    const double dx = spiral_curve.ComputeCartesianDeviationX<10>(inter_s);
    const double dy = spiral_curve.ComputeCartesianDeviationY<10>(inter_s);

    const double theta =
        common::math::NormalizeAngle(spiral_curve.Evaluate(0, inter_s));
    const double kappa = spiral_curve.Evaluate(1, inter_s);
    const double dkappa = spiral_curve.Evaluate(2, inter_s);

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

void SpiralReferenceLineSmoother::SetAnchorPoints(
    const std::vector<AnchorPoint>& anchor_points) {
  anchor_points_ = std::move(anchor_points);

  CHECK_GT(anchor_points_.size(), 1);
  zero_x_ = anchor_points_.front().path_point.x();
  zero_y_ = anchor_points_.front().path_point.y();

  std::for_each(anchor_points_.begin(), anchor_points_.end(),
                [this](AnchorPoint& p) {
                  auto curr_x = p.path_point.x();
                  auto curr_y = p.path_point.y();
                  p.path_point.set_x(curr_x - zero_x_);
                  p.path_point.set_y(curr_y - zero_y_);
                });
}

}  // namespace planning
}  // namespace apollo
