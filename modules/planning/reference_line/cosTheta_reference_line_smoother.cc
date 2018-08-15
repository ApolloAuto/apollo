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

#include "modules/planning/reference_line/cosTheta_reference_line_smoother.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <utility>

#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/reference_line/cosTheta_problem_interface.h"

namespace apollo {
namespace planning {

using apollo::common::time::Clock;

CosThetaReferenceLineSmoother::CosThetaReferenceLineSmoother(
    const ReferenceLineSmootherConfig& config)
    : ReferenceLineSmoother(config) {
  max_point_deviation_ = config.cos_theta().max_point_deviation();

  num_of_iterations_ = config.cos_theta().num_of_iteration();

  weight_cos_included_angle_ = config.cos_theta().weight_cos_included_angle();

  acceptable_tol_ = config.cos_theta().acceptable_tol();

  resolution_ = config.resolution();

  relax_ = config.cos_theta().relax();

  kappa_filter_ = config.cos_theta().kappa_filter();

  dkappa_filter_ = config.cos_theta().dkappa_filter();
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

    start_x_2nd_derivative_ =
        anchor_points_.front().path_point.x_2nd_derivative();

    start_y_derivative_ = anchor_points_.front().path_point.y_derivative();

    start_y_2nd_derivative_ =
        anchor_points_.front().path_point.y_2nd_derivative();
  }
  if (anchor_points_.back().enforced == true) {
    has_end_point_constraint_ = true;
  }

  Smooth(raw_point2d, &smoothed_point2d, anchorpoints_lateralbound);

  std::vector<ReferencePoint> ref_points;

  for (const auto& p : smoothed_point2d) {
    double heading = p.theta();
    double kappa = p.kappa();
    double dkappa = p.dkappa();
    // if (dkappa > dkappa_filter_ || dkappa < -dkappa_filter_) {
    //   continue;
    // }
    // if (kappa > kappa_filter_ || kappa < -kappa_filter_) {
    //   continue;
    // }
    if (dkappa > dkappa_filter_) {
      dkappa = dkappa_filter_;
    }
    if (dkappa < -dkappa_filter_) {
      dkappa = -dkappa_filter_;
    }
    if (kappa > kappa_filter_) {
      kappa = kappa_filter_;
    }
    if (kappa < -kappa_filter_) {
      kappa = -kappa_filter_;
    }
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
                       kappa, dkappa));
  }

  if (ref_points.size() < 2) {
    AERROR << "Fail to generate smoothed reference line.";
    return false;
  }
  *smoothed_reference_line = ReferenceLine(ref_points);
  const double end_timestamp = Clock::NowInSeconds();
  ADEBUG << "cosTheta reference line smoother time: "
         << (end_timestamp - start_timestamp) * 1000 << " ms.";
  AINFO << "cosTheta reference line smoother time: "
        << (end_timestamp - start_timestamp) * 1000 << " ms.";
  return true;
}
bool CosThetaReferenceLineSmoother::Smooth(
    std::vector<Eigen::Vector2d> scaled_point2d,
    std::vector<common::PathPoint>* ptr_interpolated_point2d,
    std::vector<double> lateral_bounds) {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<common::PathPoint> ptr_smoothed_point2d;

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
  // app->Options()->SetStringValue("derivative_test", "second-order");
  app->Options()->SetIntegerValue("print_level", 0);
  // app->Options()->SetStringValue("print_info_string", "yes");
  // app->Options()->SetStringValue("check_derivatives_for_naninf", "yes");
  app->Options()->SetIntegerValue("max_iter", num_of_iterations_);
  // app->Options()->SetStringValue("warm_start_init_point", "yes");
  // app->Options()->SetStringValue("mu_strategy", "adaptive");
  // app->Options()->SetNumericValue("warm_start_bound_push", 1e-6);
  // app->Options()->SetNumericValue("warm_start_mult_bound_push", 1e-6);
  // app->Options()->SetNumericValue("mu_init", 1e-6);
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
    ptr_smoothed_point2d.emplace_back(
        to_path_point(start_x, start_y, x_derivative, y_derivative));
  }

  // load second derivative information for each points

  for (std::size_t i = 0; i < ptr_smoothed_point2d.size(); ++i) {
    double x_2nd_derivative = 0.0;
    double y_2nd_derivative = 0.0;
    if (i == 0) {
      x_2nd_derivative = ptr_smoothed_point2d.at(i + 1).x_derivative() -
                         ptr_smoothed_point2d.at(i).x_derivative();
      y_2nd_derivative = ptr_smoothed_point2d.at(i + 1).y_derivative() -
                         ptr_smoothed_point2d.at(i).y_derivative();
      if (has_start_point_constraint_) {
        x_2nd_derivative = 0.5 * (start_x_2nd_derivative_ + x_2nd_derivative);
        y_2nd_derivative = 0.5 * (start_y_2nd_derivative_ + y_2nd_derivative);
      }
    } else if (i == ptr_smoothed_point2d.size() - 1) {
      x_2nd_derivative = ptr_smoothed_point2d.at(i).x_derivative() -
                         ptr_smoothed_point2d.at(i - 1).x_derivative();
      y_2nd_derivative = ptr_smoothed_point2d.at(i).y_derivative() -
                         ptr_smoothed_point2d.at(i - 1).y_derivative();
    } else {
      x_2nd_derivative = 0.5 * (ptr_smoothed_point2d.at(i + 1).x_derivative() -
                                ptr_smoothed_point2d.at(i - 1).x_derivative());
      y_2nd_derivative = 0.5 * (ptr_smoothed_point2d.at(i + 1).y_derivative() -
                                ptr_smoothed_point2d.at(i - 1).y_derivative());
    }
    ptr_smoothed_point2d.at(i).set_x_2nd_derivative(x_2nd_derivative);
    ptr_smoothed_point2d.at(i).set_y_2nd_derivative(y_2nd_derivative);
  }

  // insert first interpolated point
  double first_interpolated_point_info[8] = {};
  quintic_hermite_point(0, ptr_smoothed_point2d.at(0),
                        ptr_smoothed_point2d.at(1),
                        first_interpolated_point_info);
  ptr_interpolated_point2d->emplace_back(
      to_path_point(first_interpolated_point_info));

  // as the anchor points are equally spaced, density_ is calculated only once
  density_ = std::ceil(arclength_integration(1.0, ptr_smoothed_point2d.at(0),
                                             ptr_smoothed_point2d.at(1)) /
                       resolution_) -
             1;
  double t_increament = 1.0 / (density_ + 1);

  for (std::size_t i = 0; i < ptr_smoothed_point2d.size() - 1; i++) {
    double t = 0.0;
    // interpolation by quintic hermite.
    for (std::size_t j = 1; j <= density_; j++) {
      double seg_mid_point_info[8] = {};
      quintic_hermite_point(t + j * t_increament, ptr_smoothed_point2d.at(i),
                            ptr_smoothed_point2d.at(i + 1), seg_mid_point_info);
      ptr_interpolated_point2d->emplace_back(to_path_point(seg_mid_point_info));
    }
    double seg_end_point_info[8] = {};
    quintic_hermite_point(1.0, ptr_smoothed_point2d.at(i),
                          ptr_smoothed_point2d.at(i + 1), seg_end_point_info);
    ptr_interpolated_point2d->emplace_back(to_path_point(seg_end_point_info));
  }

  // use precise definition of kappa on a parametric curve
  for (std::size_t i = 0; i < ptr_interpolated_point2d->size(); i++) {
    ptr_interpolated_point2d->at(i).set_kappa(CurveMath::ComputeCurvature(
        ptr_interpolated_point2d->at(i).x_derivative(),
        ptr_interpolated_point2d->at(i).x_2nd_derivative(),
        ptr_interpolated_point2d->at(i).y_derivative(),
        ptr_interpolated_point2d->at(i).y_2nd_derivative()));
  }
  // use precise definition of dkappa on a parametric curve
  for (std::size_t i = 0; i < ptr_interpolated_point2d->size(); i++) {
    ptr_interpolated_point2d->at(i).set_dkappa(
        CurveMath::ComputeCurvatureDerivative(
            ptr_interpolated_point2d->at(i).x_derivative(),
            ptr_interpolated_point2d->at(i).x_2nd_derivative(),
            ptr_interpolated_point2d->at(i).x_3rd_derivative(),
            ptr_interpolated_point2d->at(i).y_derivative(),
            ptr_interpolated_point2d->at(i).y_2nd_derivative(),
            ptr_interpolated_point2d->at(i).y_3rd_derivative()));
  }
  return status == Ipopt::Solve_Succeeded ||
         status == Ipopt::Solved_To_Acceptable_Level;
}

void CosThetaReferenceLineSmoother::quintic_hermite_point(
    const double t, const common::PathPoint front_point,
    const common::PathPoint back_point, double* quintic_hermite_point_info) {
  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;

  double q1 = -6 * t5 + 15 * t4 - 10 * t3 + 1;
  double q2 = -3 * t5 + 8 * t4 - 6 * t3 + t;
  double q3 = -0.5 * t5 + 1.5 * t4 - 1.5 * t3 +
              0.5 * t2;
  double q4 = 6 * t5 - 15 * t4 + 10 * t3;
  double q5 = -3 * t5 + 7 * t4 - 4 * t3;
  double q6 = 0.5 * t5 - t4 + 0.5 * t3;

  double q1_d = -30 * t4 + 60 * t3 - 30 * t2;
  double q2_d = -15 * t4 + 32 * t3 - 18 * t2 + 1;
  double q3_d = -2.5 * t4 + 6 * t3 - 4.5 * t2 + t;
  double q4_d = 30 * t4 - 60 * t3 + 30 * t2;
  double q5_d = -15 * t4 + 28 * t3 - 12 * t2;
  double q6_d = 2.5 * t4 - 4 * t3 + 1.5 * t2;

  double q1_dd = -120 * t3 + 180 * t2 - 60 * t;
  double q2_dd = -60 * t3 + 96 * t2 - 36 * t;
  double q3_dd = -10 * t3 + 18 * t2 - 9 * t + 1;
  double q4_dd = 120 * t3 - 180 * t2 + 60 * t;
  double q5_dd = -60 * t3 + 84 * t2 - 24 * t;
  double q6_dd = 10 * t3 - 12 * t2 + 3 * t;

  double q1_ddd = -360 * t2 + 360 * t - 60;
  double q2_ddd = -180 * t2 + 192 * t - 36;
  double q3_ddd = -30 * t2 + 36 * t - 9;
  double q4_ddd = 360 * t2 - 360 * t + 60;
  double q5_ddd = -180 * t2 + 168 * t - 24;
  double q6_ddd = 30 * t2 - 24 * t + 3;

  double x_p = q1 * front_point.x() + q2 * front_point.x_derivative() +
               q3 * front_point.x_2nd_derivative() + q4 * back_point.x() +
               q5 * back_point.x_derivative() +
               q6 * back_point.x_2nd_derivative();
  double y_p = q1 * front_point.y() + q2 * front_point.y_derivative() +
               q3 * front_point.y_2nd_derivative() + q4 * back_point.y() +
               q5 * back_point.y_derivative() +
               q6 * back_point.y_2nd_derivative();
  double x_d = q1_d * front_point.x() + q2_d * front_point.x_derivative() +
               q3_d * front_point.x_2nd_derivative() + q4_d * back_point.x() +
               q5_d * back_point.x_derivative() +
               q6_d * back_point.x_2nd_derivative();
  double y_d = q1_d * front_point.y() + q2_d * front_point.y_derivative() +
               q3_d * front_point.y_2nd_derivative() + q4_d * back_point.y() +
               q5_d * back_point.y_derivative() +
               q6_d * back_point.y_2nd_derivative();
  double x_2d = q1_dd * front_point.x() + q2_dd * front_point.x_derivative() +
                q3_dd * front_point.x_2nd_derivative() +
                q4_dd * back_point.x() + q5_dd * back_point.x_derivative() +
                q6_dd * back_point.x_2nd_derivative();
  double y_2d = q1_dd * front_point.y() + q2_dd * front_point.y_derivative() +
                q3_dd * front_point.y_2nd_derivative() +
                q4_dd * back_point.y() + q5_dd * back_point.y_derivative() +
                q6_dd * back_point.y_2nd_derivative();

  double x_3d = q1_ddd * front_point.x() + q2_ddd * front_point.x_derivative() +
                q3_ddd * front_point.x_2nd_derivative() +
                q4_ddd * back_point.x() + q5_ddd * back_point.x_derivative() +
                q6_ddd * back_point.x_2nd_derivative();
  double y_3d = q1_ddd * front_point.y() + q2_ddd * front_point.y_derivative() +
                q3_ddd * front_point.y_2nd_derivative() +
                q4_ddd * back_point.y() + q5_ddd * back_point.y_derivative() +
                q6_ddd * back_point.y_2nd_derivative();

  quintic_hermite_point_info[0] = x_p;
  quintic_hermite_point_info[1] = y_p;
  quintic_hermite_point_info[2] = x_d;
  quintic_hermite_point_info[3] = y_d;
  quintic_hermite_point_info[4] = x_2d;
  quintic_hermite_point_info[5] = y_2d;
  quintic_hermite_point_info[6] = x_3d;
  quintic_hermite_point_info[7] = y_3d;
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

common::PathPoint CosThetaReferenceLineSmoother::to_path_point(
    const double* point_info) const {
  common::PathPoint point;
  point.set_x(point_info[0]);
  point.set_y(point_info[1]);
  point.set_x_derivative(point_info[2]);
  point.set_y_derivative(point_info[3]);
  point.set_theta(std::atan2(point_info[3], point_info[2]));
  point.set_x_2nd_derivative(point_info[4]);
  point.set_y_2nd_derivative(point_info[5]);
  point.set_x_3rd_derivative(point_info[6]);
  point.set_y_3rd_derivative(point_info[7]);
  return point;
}

double CosThetaReferenceLineSmoother::arclength_integration(
    const double t, common::PathPoint front_point,
    common::PathPoint back_point) {
  // integration interval from 0 to t
  // estimation tolerance to be 0.001
  // estimation limit to be 1024
  std::size_t n = 1;
  double a = 0;
  double b = t;
  double TOLERANCE = 0.001;
  std::size_t n_limit = 1024;
  double multiplier = (b - a) / 6.0;
  double endsum = quintic_hermite_s(a, front_point, back_point) +
                  quintic_hermite_s(b, front_point, back_point);
  double interval = (b - a) / 2.0;
  double asum = 0;
  double bsum = quintic_hermite_s(a + interval, front_point, back_point);
  double est1 = multiplier * (endsum + 2 * asum + 4 * bsum);
  double est0 = 2 * est1;

  while (n < n_limit &&
         (std::abs(est1) > 0 && std::abs((est1 - est0) / est1) > TOLERANCE)) {
    n *= 2;
    multiplier /= 2;
    interval /= 2;
    asum += bsum;
    bsum = 0;
    est0 = est1;
    double interval_div_2n = interval / (2.0 * n);

    for (std::size_t i = 1; i < 2 * n; i += 2) {
      double t = a + i * interval_div_2n;
      bsum += quintic_hermite_s(t, front_point, back_point);
    }
    est1 = multiplier * (endsum + 2 * asum + 4 * bsum);
  }
  return est1;
}

double CosThetaReferenceLineSmoother::quintic_hermite_s(
    const double t, common::PathPoint front_point,
    common::PathPoint back_point) {
  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double q1 = -30 * t4 + 60 * t3 - 30 * t2;
  double q2 = -15 * t4 + 32 * t3 - 18 * t2 + 1;
  double q3 = -2.5 * t4 + 6 * t3 - 4.5 * t2 + t;
  double q4 = 30 * t4 - 60 * t3 + 30 * t2;
  double q5 = -15 * t4 + 28 * t3 - 12 * t2;
  double q6 = 2.5 * t4 - 4 * t3 + 1.5 * t2;
  double x_d = q1 * front_point.x() + q2 * front_point.x_derivative() +
               q3 * front_point.x_2nd_derivative() + q4 * back_point.x() +
               q5 * back_point.x_derivative() +
               q6 * back_point.x_2nd_derivative();
  double y_d = q1 * front_point.y() + q2 * front_point.y_derivative() +
               q3 * front_point.y_2nd_derivative() + q4 * back_point.y() +
               q5 * back_point.y_derivative() +
               q6 * back_point.y_2nd_derivative();
  return std::sqrt(x_d * x_d + y_d * y_d);
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
