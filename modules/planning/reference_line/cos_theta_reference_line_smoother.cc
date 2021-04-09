/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/reference_line/cos_theta_reference_line_smoother.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <utility>

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/math/smoothing_spline/spline_2d_solver.h"
#include "modules/planning/reference_line/cos_theta_problem_interface.h"
#include "modules/planning/reference_line/qp_spline_reference_line_smoother.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;

CosThetaReferenceLineSmoother::CosThetaReferenceLineSmoother(
    const ReferenceLineSmootherConfig& config)
    : ReferenceLineSmoother(config) {
  CHECK(common::util::GetProtoFromFile(FLAGS_reopt_smoother_config_filename,
                                       &reopt_smoother_config_))
      << "Failed to load smoother config file "
      << FLAGS_reopt_smoother_config_filename;

  reopt_qp_smoother_.reset(
      new QpSplineReferenceLineSmoother(reopt_smoother_config_));

  max_point_deviation_ = config.cos_theta().max_point_deviation();

  num_of_iterations_ = config.cos_theta().num_of_iteration();

  weight_cos_included_angle_ = config.cos_theta().weight_cos_included_angle();

  acceptable_tol_ = config.cos_theta().acceptable_tol();

  relax_ = config.cos_theta().relax();

  reopt_qp_bound_ = config.cos_theta().reopt_qp_bound();
}

bool CosThetaReferenceLineSmoother::Smooth(
    const ReferenceLine& raw_reference_line,
    ReferenceLine* const smoothed_reference_line) {
  const double start_timestamp = Clock::NowInSeconds();

  std::vector<common::PathPoint> smoothed_point2d;
  std::vector<Eigen::Vector2d> raw_point2d;
  std::vector<double> anchorpoints_lateralbound;

  for (const auto& anchor_point : anchor_points_) {
    raw_point2d.emplace_back(anchor_point.path_point.x(),
                             anchor_point.path_point.y());
    anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
  }
  if (anchor_points_.front().enforced == true) {
    has_start_point_constraint_ = true;

    start_x_derivative_ = anchor_points_.front().path_point.x_derivative();

    start_y_derivative_ = anchor_points_.front().path_point.y_derivative();
  }
  if (anchor_points_.back().enforced == true) {
    has_end_point_constraint_ = true;
  }

  Smooth(raw_point2d, anchorpoints_lateralbound, &smoothed_point2d);

  // load the results by cosTheta as anchor points and put it into qp_spline to
  // do the interpolation
  reopt_anchor_points_.clear();

  for (const auto& p : smoothed_point2d) {
    common::SLPoint ref_sl_point;
    if (!raw_reference_line.XYToSL({p.x(), p.y()}, &ref_sl_point)) {
      return false;
    }
    if (ref_sl_point.s() < 0 ||
        ref_sl_point.s() > raw_reference_line.Length()) {
      continue;
    }
    double heading = p.theta();
    double s = p.s();
    AnchorPoint anchor;
    anchor.longitudinal_bound = reopt_qp_bound_;
    anchor.lateral_bound = reopt_qp_bound_;
    anchor.path_point = apollo::common::util::MakePathPoint(
        p.x(), p.y(), 0.0, heading, 0.0, 0.0, 0.0);
    anchor.path_point.set_s(s);
    reopt_anchor_points_.emplace_back(anchor);
  }
  reopt_qp_smoother_->SetAnchorPoints(reopt_anchor_points_);
  if (!reopt_qp_smoother_->Smooth(raw_reference_line,
                                  smoothed_reference_line)) {
    AERROR << "Failed to reopt smooth reference line with anchor points by "
              "cosTheta";
    return false;
  }

  const double end_timestamp = Clock::NowInSeconds();
  AINFO << "cos_theta reference line smoother time: "
        << (end_timestamp - start_timestamp) * 1000 << " ms.";
  return true;
}

bool CosThetaReferenceLineSmoother::Smooth(
    const std::vector<Eigen::Vector2d>& scaled_point2d,
    const std::vector<double>& lateral_bounds,
    std::vector<common::PathPoint>* ptr_smoothed_point2d) {
  std::vector<double> x;
  std::vector<double> y;

  CosThetaProbleminterface* ptop =
      new CosThetaProbleminterface(scaled_point2d, lateral_bounds);

  ptop->set_default_max_point_deviation(max_point_deviation_);
  ptop->set_weight_cos_included_angle(weight_cos_included_angle_);
  ptop->set_relax_end_constraint(relax_);

  if (has_start_point_constraint_) {
    ptop->set_start_point(scaled_point2d.front().x(),
                          scaled_point2d.front().y());
  }

  if (has_end_point_constraint_) {
    ptop->set_end_point(scaled_point2d.back().x(), scaled_point2d.back().y());
  }

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->Options()->SetIntegerValue("print_level", 1);
  app->Options()->SetIntegerValue("max_iter", num_of_iterations_);
  app->Options()->SetNumericValue("acceptable_tol", acceptable_tol_);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    AINFO << "*** Error during initialization!";
    return static_cast<int>(status);
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    ADEBUG << "*** The problem solved in " << iter_count << " iterations!";
  } else {
    AINFO << "Return status: " << int(status);
  }

  ptop->get_optimization_results(&x, &y);
  // load the point position and estimated derivatives at each point
  if (x.size() < 2 || y.size() < 2) {
    AINFO << "Return by IPOPT is wrong. Size smaller than 2 ";
    return false;
  }
  for (std::size_t i = 0; i < x.size(); ++i) {
    // reverse back to the unscaled points
    double start_x = x[i] + zero_x_;
    double start_y = y[i] + zero_y_;
    double x_derivative = 0.0;
    double y_derivative = 0.0;
    if (i == 0) {
      x_derivative = (x[i + 1] - x[i]);
      y_derivative = (y[i + 1] - y[i]);
      if (has_start_point_constraint_) {
        x_derivative = 0.5 * (start_x_derivative_ + x_derivative);
        y_derivative = 0.5 * (start_y_derivative_ + y_derivative);
      }
    } else if (i == x.size() - 1) {
      x_derivative = (x[i] - x[i - 1]);
      y_derivative = (y[i] - y[i - 1]);
    } else {
      x_derivative = 0.5 * (x[i + 1] - x[i - 1]);
      y_derivative = 0.5 * (y[i + 1] - y[i - 1]);
    }
    ptr_smoothed_point2d->emplace_back(
        to_path_point(start_x, start_y, x_derivative, y_derivative));
  }
  // load the accumulated s at each point
  ptr_smoothed_point2d->front().set_s(0.0);
  double accumulated_s = 0.0;
  double Fx = ptr_smoothed_point2d->front().x();
  double Fy = ptr_smoothed_point2d->front().y();
  double Nx = 0.0;
  double Ny = 0.0;
  for (std::size_t i = 1; i < ptr_smoothed_point2d->size(); i++) {
    Nx = ptr_smoothed_point2d->at(i).x();
    Ny = ptr_smoothed_point2d->at(i).y();
    double end_segment_s =
        std::sqrt((Fx - Nx) * (Fx - Nx) + (Fy - Ny) * (Fy - Ny));
    ptr_smoothed_point2d->at(i).set_s(end_segment_s + accumulated_s);
    accumulated_s += end_segment_s;
    Fx = Nx;
    Fy = Ny;
  }
  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}

common::PathPoint CosThetaReferenceLineSmoother::to_path_point(
    const double x, const double y, const double x_derivative,
    const double y_derivative) const {
  common::PathPoint point;
  point.set_x(x);
  point.set_y(y);
  point.set_x_derivative(x_derivative);
  point.set_y_derivative(y_derivative);
  return point;
}

void CosThetaReferenceLineSmoother::SetAnchorPoints(
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
