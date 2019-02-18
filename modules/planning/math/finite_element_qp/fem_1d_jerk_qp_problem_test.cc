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
 * @file
 **/

#include <chrono>

#include "cyber/common/log.h"
#include "gtest/gtest.h"

#include "modules/planning/common/planning_gflags.h"

#define private public
#define protected public
#include "modules/planning/math/finite_element_qp/fem_1d_jerk_qp_problem.h"

namespace apollo {
namespace planning {

TEST(Fem1dJerkQpProblemTest, basic_test) {
  FLAGS_enable_osqp_debug = true;
  Fem1dQpProblem* fem_qp = new Fem1dJerkQpProblem();
  std::array<double, 3> x_init = {1.5, 0.01, 0.001};
  double delta_s = 0.5;
  size_t n = 400;
  std::vector<std::tuple<double, double, double>> x_bounds;
  for (size_t i = 0; i < n; ++i) {
    if (i != 2) {
      x_bounds.emplace_back(
          std::make_tuple(static_cast<double>(i), -1.81, 1.95));
    } else {
      x_bounds.emplace_back(
          std::make_tuple(static_cast<double>(i), 0.81, 1.95));
    }
  }
  std::array<double, 5> w = {1.0, 2.0, 3.0, 4.0, 1.45};
  double max_x_third_order_derivative = 1.25;
  EXPECT_TRUE(
      fem_qp->Init(n, x_init, delta_s, w, max_x_third_order_derivative));
  EXPECT_FALSE(fem_qp->Optimize());

  fem_qp->SetVariableBounds(x_bounds);

  auto start_time = std::chrono::system_clock::now();
  EXPECT_TRUE(fem_qp->Optimize());
  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  AINFO << "qp_optimizer used time: " << diff.count() * 1000 << " ms.";

  const std::vector<double> x = fem_qp->x();
  AINFO << "x.size() = " << x.size();
  for (size_t i = 0; i < x.size(); ++i) {
    EXPECT_LE(x[i], fem_qp->x_bounds_[i].second);
    EXPECT_GE(x[i], fem_qp->x_bounds_[i].first);
  }
}

TEST(Fem1dJerkQpProblemTest, add_bounds_test) {
  FLAGS_enable_osqp_debug = false;
  Fem1dQpProblem* fem_qp = new Fem1dJerkQpProblem();
  std::array<double, 3> x_init = {1.5, 0.01, 0.001};
  double delta_s = 0.5;
  size_t n = 400;
  std::array<double, 5> w = {1.0, 2.0, 3.0, 4.0, 1.45};
  double max_x_third_order_derivative = 0.25;
  EXPECT_TRUE(
      fem_qp->Init(n, x_init, delta_s, w, max_x_third_order_derivative));

  std::vector<std::tuple<double, double, double>> x_bounds;
  for (size_t i = 10; i < 20; ++i) {
    x_bounds.emplace_back(std::make_tuple(static_cast<double>(i), -1.81, 1.95));
  }

  fem_qp->SetVariableBounds(x_bounds);
  const auto& x = fem_qp->x_bounds_;

  CHECK_EQ(n, x.size());

  for (size_t i = 20; i < 40; i += 2) {
    EXPECT_DOUBLE_EQ(std::get<0>(x[i]), -1.81);
    EXPECT_DOUBLE_EQ(std::get<1>(x[i]), 1.95);
  }
}

TEST(Fem1dJerkQpProblemTest, derivative_constraint_test) {
  FLAGS_enable_osqp_debug = true;
  Fem1dQpProblem* fem_qp = new Fem1dJerkQpProblem();
  std::array<double, 3> x_init = {4.5, 0.00, 0.0};
  double delta_s = 0.5;
  size_t n = 200;
  std::vector<std::tuple<double, double, double>> x_bounds;
  for (size_t i = 0; i < n; ++i) {
    x_bounds.emplace_back(std::make_tuple(static_cast<double>(i), -6.0, 6.0));
  }
  std::array<double, 5> w = {1.0, 100.0, 1000.0, 1000.0, 0.0};
  double max_x_third_order_derivative = 2.0;
  EXPECT_TRUE(
      fem_qp->Init(n, x_init, delta_s, w, max_x_third_order_derivative));

  fem_qp->SetVariableBounds(x_bounds);

  const double dx_max = std::sqrt(0.5) / 15.0;
  std::vector<std::tuple<double, double, double>> dx_bounds;
  for (size_t i = 0; i < 20; ++i) {
    dx_bounds.emplace_back(
        std::make_tuple(static_cast<double>(i), -dx_max, dx_max));
  }
  fem_qp->SetVariableDerivativeBounds(dx_bounds);

  auto start_time = std::chrono::system_clock::now();
  EXPECT_TRUE(fem_qp->Optimize());
  auto end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_time - start_time;
  AINFO << "qp_optimizer used time: " << diff.count() * 1000 << " ms.";

  const std::vector<double>& x = fem_qp->x();
  AINFO << "x.size() = " << x.size();
  for (size_t i = 0; i < x.size(); ++i) {
    EXPECT_LT(x[i], fem_qp->x_bounds_[i].second);
    EXPECT_GT(x[i], fem_qp->x_bounds_[i].first);
  }

  const std::vector<double>& dx = fem_qp->x_derivative();
  AINFO << "dx.size() = " << dx.size();
  for (size_t i = 0; i < dx.size(); ++i) {
    EXPECT_LT(dx[i] - 1e-12, fem_qp->dx_bounds_[i].second);
    EXPECT_GT(dx[i] + 1e-12, fem_qp->dx_bounds_[i].first);
  }
}

}  // namespace planning
}  // namespace apollo
