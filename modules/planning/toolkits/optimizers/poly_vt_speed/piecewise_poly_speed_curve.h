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
 * @file piecewise_poly_speed_curve.h
 **/

#pragma once

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/math/curve1d/cubic_polynomial_curve1d.h"
#include "modules/planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

/**
 * @brief: Piecewise Polynomial Speed Profile (v(t) is a quartic piecewise
 * polynomial function with two polynomial segments
 * the inter-connectted point has acceleration equal to zero
 */

class PiecewisePolySpeedCurve {
 public:
  /**
   * @brief constructor
   */
  PiecewisePolySpeedCurve() = default;
  PiecewisePolySpeedCurve(const common::SpeedPoint& init_point,
                          const common::SpeedPoint& connect_point,
                          const double end_t);
  PiecewisePolySpeedCurve(const common::SpeedPoint& init_point,
                          const common::SpeedPoint& connect_point,
                          const common::SpeedPoint& end_point);
  PiecewisePolySpeedCurve(const double init_v, const double init_a,
                          const double connect_v, const double connect_a,
                          const double connect_j, const double connect_t,
                          const double end_t);

  /**
   * @brief: initialize the piecewise polynomial speed curve with stop point
   */
  void InitWithStopPoint(const common::SpeedPoint& init_point,
                         const common::SpeedPoint& stop_point,
                         const double end_t);
  /**
   * @brief: sampling points from curve with equal time resolution
   * @param num_points   :number of points
   * @param speed_points :outputs
   */
  void SampleSpeedPoints(
      const size_t num_points,
      std::vector<common::SpeedPoint>* const speed_points) const;

  /**
   * @brief: sampling by time with resolution
   */
  void SampleSpeedPointsWithTime(
      const double unit_t,
      std::vector<common::SpeedPoint>* const speed_points) const;

  /**
   * @brief: evaluate curve at time
   */
  void Evaluate(const double t, common::SpeedPoint* const speed_point) const;

  const common::SpeedPoint& init_point() const;
  const common::SpeedPoint& connect_point() const;
  const common::SpeedPoint& end_point() const;

  /**
   * @brief : total time
   */
  double param_t() const;

 private:
  // acc profile
  CubicPolynomialCurve1d a_curve_;
  CubicPolynomialCurve1d extended_a_curve_;

  // velocity profile
  QuarticPolynomialCurve1d v_curve_;
  QuarticPolynomialCurve1d extended_v_curve_;

  // station w.r.t time profile
  QuinticPolynomialCurve1d s_curve_;
  QuinticPolynomialCurve1d extended_s_curve_;

  // init point
  common::SpeedPoint init_point_;

  // interconnected point
  common::SpeedPoint connect_point_;

  // end point
  common::SpeedPoint end_point_;

  // total time
  double param_t_ = 1.0;
};

}  // namespace planning
}  // namespace apollo
