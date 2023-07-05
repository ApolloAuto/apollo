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
 * @file curvature_math_test.cc
 **/
#include "modules/planning/math/curve_math.h"

#include <cmath>

#include "gtest/gtest.h"

namespace apollo {
namespace planning {

TEST(TestSuite, curvature_math_test) {
  // Straight line
  double curvature = CurveMath::ComputeCurvature(1.0, 0.0, 1.0, 0.0);
  EXPECT_NEAR(curvature, 0.0, 1e-6);

  double curvature_derivative =
      CurveMath::ComputeCurvatureDerivative(1.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  EXPECT_NEAR(curvature_derivative, 0.0, 1e-6);

  // Unit circle X = (std::cos(t), std::sin(t)), at t = 0.0
  curvature = CurveMath::ComputeCurvature(0.0, -1, 1.0, 0.0);
  EXPECT_NEAR(curvature, 1.0, 1e-6);

  // Unit circle X = (std::cos(t), std::sin(t)), at t = PI/4,
  double cos_angle = std::cos(M_PI / 4);
  double sin_angle = std::sin(M_PI / 4);
  curvature = CurveMath::ComputeCurvature(-sin_angle, -cos_angle, cos_angle,
                                          -sin_angle);
  EXPECT_NEAR(curvature, 1.0, 1e-6);

  curvature_derivative = CurveMath::ComputeCurvatureDerivative(
      -sin_angle, -cos_angle, sin_angle, cos_angle, -sin_angle, -cos_angle);
  EXPECT_NEAR(curvature_derivative, 0.0, 1e-6);
}

}  // namespace planning
}  // namespace apollo
