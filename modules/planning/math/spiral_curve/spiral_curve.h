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
 * @file: spiral_curve.h
 * @brief: spiral path base class
 **/

#ifndef MODULES_PLANNING_MATH_SPIRAL_CURVE_SPIRAL_CURVE_H_
#define MODULES_PLANNING_MATH_SPIRAL_CURVE_SPIRAL_CURVE_H_

#include <cmath>
#include <vector>

#include "modules/common/proto/path_point.pb.h"
#include "modules/common/status/status.h"
#include "modules/planning/proto/spiral_curve_config.pb.h"

namespace apollo {
namespace planning {

class SpiralCurve {
 public:
  explicit SpiralCurve(const common::PathPoint& s, const common::PathPoint& e,
                       const std::uint32_t order);
  virtual ~SpiralCurve();

  /**
  *   @brief: set configuration if desired (default setting was defined in
  *constructor)
  **/
  void set_spiral_config(const SpiralCurveConfig& spiral_config);
  /**
  *   @brief : default process of calculating path without lookup table
  *   @return: errors of final state: fitted value vs true end point
  **/
  virtual bool calculate_path() = 0;

  /**
  *   @brief: output methods
  **/
  const std::vector<double>& p_params() const;
  const SpiralCurveConfig& spiral_config() const;
  const common::PathPoint& start_point() const;
  const common::PathPoint& end_point() const;
  double sg() const;
  double error() const;

  /**
  *   @brief : get path vector with sampling size n
  *   @return: sequence of sampling points
  **/
  virtual common::Status get_path_vec(
      const std::uint32_t n,
      std::vector<common::PathPoint>* path_points) const = 0;
  /**
  *   @brief: calcualate quintic path point at s locations
  *   along the whole path.
  *   @return: vector of path points
  **/
  virtual common::Status get_path_vec_with_s(
      const std::vector<double>& vec_s,
      std::vector<common::PathPoint>* path_points) const = 0;

 private:
  const common::PathPoint* start_point_;
  const common::PathPoint* end_point_;
  std::vector<double> p_params_;
  double sg_;
  double error_;
  SpiralCurveConfig spiral_config_;

 protected:
  void set_sg(const double sg);
  void set_error(const double error);
  bool result_sanity_check() const;
  template <typename T>
  void prepend_to_p_params(T begin, T end) {
    std::copy(begin, end, p_params_.begin());
  }
  static constexpr double s_two_pi_ = 2 * M_PI;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_MATH_SPIRAL_CURVE_SPIRAL_CURVE_H_
