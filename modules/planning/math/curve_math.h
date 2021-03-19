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
 * @file curve_math.h
 **/

#pragma once

namespace apollo {
namespace planning {

class CurveMath {
 public:
  CurveMath() = delete;
  /**
   * @brief Compute the curvature (kappa) given curve X = (x(t), y(t))
   *        which t is an arbitrary parameter.
   * @param dx dx / dt
   * @param d2x d(dx) / dt
   * @param dy dy / dt
   * @param d2y d(dy) / dt
   * @return the curvature
   */
  static double ComputeCurvature(const double dx, const double d2x,
                                 const double dy, const double d2y);

  /**
   * @brief Compute the curvature change rate w.r.t. curve length (dkappa) given
   * curve X = (x(t), y(t))
   *        which t is an arbitrary parameter.
   * @param dx dx / dt
   * @param d2x d(dx) / dt
   * @param dy dy / dt
   * @param d2y d(dy) / dt
   * @param d3x d(d2x) / dt
   * @param d3y d(d2y) / dt
   * @return the curvature change rate
   */
  static double ComputeCurvatureDerivative(const double dx, const double d2x,
                                           const double d3x, const double dy,
                                           const double d2y, const double d3y);
};

}  // namespace planning
}  // namespace apollo
