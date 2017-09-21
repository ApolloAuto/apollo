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
 * @file: quintic_spiral_curve.h
 * @brief: header file for path class
 * @model description:
 *           x_p (s) = int_0^s cos( theta_p (s)) ds
 *           y_p (s) = int_0^s sin( theta_p (s)) ds
 *
 *           quintic version:
 *           theta_p (s) = a s + b s^2 / 2 + c s^3 / 3 + d s^4 / 4 + e s^5 / 5 +
 *f s^6 /6
 *           quintic version:
 *           kappa_p (s) = a + b s + c s^2 + d s^3 + e s^4 + f s^5
 *
 * @problem Description:
 *           Solve boundary shooting problem with newton raphson method
 **/

#ifndef MODULES_PLANNING_MATH_SPIRAL_CURVE_QUINTIC_SPIRAL_CURVE_H_
#define MODULES_PLANNING_MATH_SPIRAL_CURVE_QUINTIC_SPIRAL_CURVE_H_

#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/math/spiral_curve/spiral_curve.h"

namespace apollo {
namespace planning {

class QuinticSpiralCurve : public SpiralCurve {
 public:
  QuinticSpiralCurve(const common::PathPoint& s, const common::PathPoint& e);
  ~QuinticSpiralCurve() = default;
  bool CalculatePath();
  common::Status GetPathVec(
      const std::uint32_t n,
      std::vector<common::PathPoint>* path_points) const override;
  common::Status GetPathVecWithS(
      const std::vector<double>& vec_s,
      std::vector<common::PathPoint>* path_points) const override;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_MATH_SPIRAL_CURVE_QUINTIC_SPIRAL_CURVE_H_
