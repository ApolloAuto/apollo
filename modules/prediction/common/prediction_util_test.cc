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

#include "modules/prediction/common/prediction_util.h"

#include "gtest/gtest.h"

namespace apollo {
namespace prediction {
namespace math_util {

TEST(PredictionUtilTest, normalize) {
  double value = 3.0;
  double mean = 2.0;
  double std_dev = 0.01;
  EXPECT_DOUBLE_EQ(Normalize(value, mean, std_dev), 99.999999);
}

TEST(PredictionUtilTest, relu) {
  double value = 2.0;
  EXPECT_DOUBLE_EQ(Relu(value), 2.0);

  value = -3.0;
  EXPECT_DOUBLE_EQ(Relu(value), 0.0);
}

TEST(PredictionUtilTest, sigmoid) {
  double value = 2.0;
  EXPECT_DOUBLE_EQ(Sigmoid(value), 0.88079707797788231);
}

TEST(PredictionUtilTest, solvable_quadratic_equation) {
  std::vector<double> coefficients = {5.0, 6.0, 1.0};
  std::pair<double, double> roots;
  EXPECT_EQ(SolveQuadraticEquation(coefficients, &roots), 0);
  EXPECT_DOUBLE_EQ(roots.first, -0.2);
  EXPECT_DOUBLE_EQ(roots.second, -1.0);
}

TEST(PredictionUtilTest, non_solvable_quadratic_equation) {
  std::vector<double> coefficients = {5.0, 2.0, 1.0};
  std::pair<double, double> roots;
  EXPECT_EQ(SolveQuadraticEquation(coefficients, &roots), -1);
}

}  // namespace math_util

namespace predictor_util {

using ::apollo::common::TrajectoryPoint;

TEST(PredictionUtilTest, translate_point) {
  double x = 1.0;
  double y = 2.0;
  TrajectoryPoint trajectory_point;
  TranslatePoint(x, y, &trajectory_point);
  EXPECT_DOUBLE_EQ(trajectory_point.path_point().x(), 1.0);
  EXPECT_DOUBLE_EQ(trajectory_point.path_point().y(), 2.0);
}

}  // namespace predictor_util
}  // namespace prediction
}  // namespace apollo
