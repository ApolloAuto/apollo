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
#include <utility>

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
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
  print_level_ = config.cos_theta().print_level();

  max_point_deviation_ = config.cos_theta().max_point_deviation();

  max_num_of_iterations_ = config.cos_theta().max_num_of_iterations();

  weight_cos_included_angle_ = config.cos_theta().weight_cos_included_angle();

  weight_anchor_points_ = config.cos_theta().weight_anchor_points();

  weight_length_ = config.cos_theta().weight_length();

  tol_ = config.cos_theta().tol();

  acceptable_tol_ = config.cos_theta().acceptable_tol();

  acceptable_num_of_iterations_ =
      config.cos_theta().acceptable_num_of_iterations();

  relax_ = config.cos_theta().relax();

  use_automatic_differentiation_ =
      config.cos_theta().use_automatic_differentiation();
}

bool CosThetaReferenceLineSmoother::Smooth(
    const ReferenceLine& raw_reference_line,
    ReferenceLine* const smoothed_reference_line) {
  std::vector<Eigen::Vector2d> smoothed_point2d;
  std::vector<Eigen::Vector2d> raw_point2d;
  std::vector<double> anchorpoints_lateralbound;

  for (const auto& anchor_point : anchor_points_) {
    raw_point2d.emplace_back(anchor_point.path_point.x(),
                             anchor_point.path_point.y());
    anchorpoints_lateralbound.emplace_back(anchor_point.lateral_bound);
  }

  NormalizePoints(&raw_point2d);

  if (anchor_points_.front().enforced) {
    has_start_point_constraint_ = true;
  }
  if (anchor_points_.back().enforced) {
    has_end_point_constraint_ = true;
  }

  const auto start_timestamp = std::chrono::system_clock::now();

  bool status =
      CosThetaSmooth(raw_point2d, anchorpoints_lateralbound, &smoothed_point2d);

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  ADEBUG << "cos_theta reference line smoother time: " << diff.count() * 1000.0
         << " ms.";

  if (!status) {
    AERROR << "cos_theta reference line smoother fails";
    return false;
  }

  DeNormalizePoints(&smoothed_point2d);

  std::vector<ReferencePoint> ref_points;
  GenerateRefPointProfile(raw_reference_line, smoothed_point2d, &ref_points);

  ReferencePoint::RemoveDuplicates(&ref_points);
  if (ref_points.size() < 2) {
    AERROR << "Fail to generate smoothed reference line.";
    return false;
  }
  *smoothed_reference_line = ReferenceLine(ref_points);
  return true;
}

bool CosThetaReferenceLineSmoother::CosThetaSmooth(
    const std::vector<Eigen::Vector2d>& scaled_point2d,
    const std::vector<double>& lateral_bounds,
    std::vector<Eigen::Vector2d>* ptr_smoothed_point2d) {
  std::vector<double> x;
  std::vector<double> y;
  CosThetaProbleminterface* ptop =
      new CosThetaProbleminterface(scaled_point2d, lateral_bounds);

  ptop->set_default_max_point_deviation(max_point_deviation_);
  ptop->set_weight_cos_included_angle(weight_cos_included_angle_);
  ptop->set_weight_anchor_points(weight_anchor_points_);
  ptop->set_weight_length(weight_length_);
  ptop->set_relax_end_constraint(relax_);
  ptop->set_automatic_differentiation_flag(use_automatic_differentiation_);

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
  app->Options()->SetIntegerValue("print_level",
                                  static_cast<int>(print_level_));
  app->Options()->SetIntegerValue("max_iter",
                                  static_cast<int>(max_num_of_iterations_));
  app->Options()->SetIntegerValue(
      "acceptable_iter", static_cast<int>(acceptable_num_of_iterations_));
  app->Options()->SetNumericValue("tol", tol_);
  app->Options()->SetNumericValue("acceptable_tol", acceptable_tol_);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    AERROR << "*** Error during initialization!";
    return false;
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    ADEBUG << "*** The problem solved in " << iter_count << " iterations!";
  } else {
    AERROR << "Solver fails with return code: " << static_cast<int>(status);
    return false;
  }

  ptop->get_optimization_results(&x, &y);
  // load the point position and estimated derivatives at each point
  if (x.size() < 2 || y.size() < 2) {
    AERROR << "Return by IPOPT is wrong. Size smaller than 2 ";
    return false;
  }

  for (size_t i = 0; i < x.size(); ++i) {
    ptr_smoothed_point2d->emplace_back(x[i], y[i]);
  }

  return true;
}

void CosThetaReferenceLineSmoother::SetAnchorPoints(
    const std::vector<AnchorPoint>& anchor_points) {
  CHECK_GT(anchor_points.size(), 1);
  anchor_points_ = anchor_points;
}

void CosThetaReferenceLineSmoother::NormalizePoints(
    std::vector<Eigen::Vector2d>* xy_points) {
  zero_x_ = xy_points->front().x();
  zero_y_ = xy_points->front().y();

  std::for_each(xy_points->begin(), xy_points->end(),
                [this](Eigen::Vector2d& p) {
                  auto curr_x = p.x();
                  auto curr_y = p.y();
                  Eigen::Vector2d xy(curr_x - zero_x_, curr_y - zero_y_);
                  p = xy;
                });
}

void CosThetaReferenceLineSmoother::DeNormalizePoints(
    std::vector<Eigen::Vector2d>* xy_points) {
  std::for_each(xy_points->begin(), xy_points->end(),
                [this](Eigen::Vector2d& p) {
                  auto curr_x = p.x();
                  auto curr_y = p.y();
                  Eigen::Vector2d xy(curr_x + zero_x_, curr_y + zero_y_);
                  p = xy;
                });
}

bool CosThetaReferenceLineSmoother::GenerateRefPointProfile(
    const ReferenceLine& raw_reference_line,
    const std::vector<Eigen::Vector2d>& xy_points,
    std::vector<ReferencePoint>* reference_points) {
  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;
  std::vector<double> y_over_s_second_derivatives;
  std::vector<double> x_over_s_second_derivatives;
  std::vector<double> headings;
  std::vector<double> kappas;
  std::vector<double> accumulated_s;
  std::vector<double> dkappas;

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  size_t points_size = xy_points.size();
  for (size_t i = 0; i < points_size; ++i) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points[i + 1].x() - xy_points[i].x());
      y_delta = (xy_points[i + 1].y() - xy_points[i].y());
    } else if (i == points_size - 1) {
      x_delta = (xy_points[i].x() - xy_points[i - 1].x());
      y_delta = (xy_points[i].y() - xy_points[i - 1].y());
    } else {
      x_delta = 0.5 * (xy_points[i + 1].x() - xy_points[i - 1].x());
      y_delta = 0.5 * (xy_points[i + 1].y() - xy_points[i - 1].y());
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }

  // Heading calculation
  for (size_t i = 0; i < points_size; ++i) {
    headings.push_back(std::atan2(dys[i], dxs[i]));
  }

  // Get linear interpolated s for dkappa calculation
  double distance = 0.0;
  accumulated_s.push_back(distance);
  double fx = xy_points[0].x();
  double fy = xy_points[0].y();
  double nx = 0.0;
  double ny = 0.0;
  for (size_t i = 1; i < points_size; ++i) {
    nx = xy_points[i].x();
    ny = xy_points[i].y();
    double end_segment_s =
        std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    accumulated_s.push_back(end_segment_s + distance);
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  // Get finite difference approximated first derivative of y and x respective
  // to s for kappa calculation
  for (size_t i = 0; i < points_size; ++i) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (xy_points[i + 1].x() - xy_points[i].x()) /
            (accumulated_s[i + 1] - accumulated_s[i]);
      yds = (xy_points[i + 1].y() - xy_points[i].y()) /
            (accumulated_s[i + 1] - accumulated_s[i]);
    } else if (i == points_size - 1) {
      xds = (xy_points[i].x() - xy_points[i - 1].x()) /
            (accumulated_s[i] - accumulated_s[i - 1]);
      yds = (xy_points[i].y() - xy_points[i - 1].y()) /
            (accumulated_s[i] - accumulated_s[i - 1]);
    } else {
      xds = (xy_points[i + 1].x() - xy_points[i - 1].x()) /
            (accumulated_s[i + 1] - accumulated_s[i - 1]);
      yds = (xy_points[i + 1].y() - xy_points[i - 1].y()) /
            (accumulated_s[i + 1] - accumulated_s[i - 1]);
    }
    x_over_s_first_derivatives.push_back(xds);
    y_over_s_first_derivatives.push_back(yds);
  }

  // Get finite difference approximated second derivative of y and x respective
  // to s for kappa calculation
  for (size_t i = 0; i < points_size; ++i) {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (accumulated_s[i + 1] - accumulated_s[i]);
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (accumulated_s[i + 1] - accumulated_s[i]);
    } else if (i == points_size - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (accumulated_s[i] - accumulated_s[i - 1]);
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (accumulated_s[i] - accumulated_s[i - 1]);
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] -
              x_over_s_first_derivatives[i - 1]) /
             (accumulated_s[i + 1] - accumulated_s[i - 1]);
      ydds = (y_over_s_first_derivatives[i + 1] -
              y_over_s_first_derivatives[i - 1]) /
             (accumulated_s[i + 1] - accumulated_s[i - 1]);
    }
    x_over_s_second_derivatives.push_back(xdds);
    y_over_s_second_derivatives.push_back(ydds);
  }

  for (size_t i = 0; i < points_size; ++i) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double kappa =
        (xds * ydds - yds * xdds) /
        (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    kappas.push_back(kappa);
  }

  // Dkappa calculation
  for (size_t i = 0; i < points_size; ++i) {
    double dkappa = 0.0;
    if (i == 0) {
      dkappa = (kappas[i + 1] - kappas[i]) /
               (accumulated_s[i + 1] - accumulated_s[i]);
    } else if (i == points_size - 1) {
      dkappa = (kappas[i] - kappas[i - 1]) /
               (accumulated_s[i] - accumulated_s[i - 1]);
    } else {
      dkappa = (kappas[i + 1] - kappas[i - 1]) /
               (accumulated_s[i + 1] - accumulated_s[i - 1]);
    }
    dkappas.push_back(dkappa);
  }

  // Load into ReferencePoints
  for (size_t i = 0; i < points_size; ++i) {
    common::SLPoint ref_sl_point;
    if (!raw_reference_line.XYToSL({xy_points[i].x(), xy_points[i].y()},
                                   &ref_sl_point)) {
      return false;
    }
    const double kEpsilon = 1e-6;
    if (ref_sl_point.s() < -kEpsilon ||
        ref_sl_point.s() > raw_reference_line.Length()) {
      continue;
    }
    ref_sl_point.set_s(std::max(ref_sl_point.s(), 0.0));
    ReferencePoint rlp = raw_reference_line.GetReferencePoint(ref_sl_point.s());
    auto new_lane_waypoints = rlp.lane_waypoints();
    for (auto& lane_waypoint : new_lane_waypoints) {
      lane_waypoint.l = ref_sl_point.l();
    }
    reference_points->emplace_back(ReferencePoint(
        hdmap::MapPathPoint(
            common::math::Vec2d(xy_points[i].x(), xy_points[i].y()),
            headings[i], new_lane_waypoints),
        kappas[i], dkappas[i]));
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
